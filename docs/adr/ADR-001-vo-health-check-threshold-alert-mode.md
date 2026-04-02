# ADR-001: VO 健康度檢查、閾值策略調整與告警模式

- **狀態**: 提議中 (Proposed)
- **日期**: 2026-04-02
- **決策者**: 專案開發團隊

## 背景 (Context)

目前的 `visual_localization_node` 在 ABSOLUTE_MODE 下，當偵測到視覺定位與 LiDAR SLAM
的誤差超過閾值時，會**直接以 VO 推算的絕對座標呼叫 SLAM 的 initialpose service 進行修正**。

這個設計存在一個根本性的風險：

**VO 本身的可靠性是有條件的。** 視覺里程計在以下場景會產生嚴重漂移甚至失效：
- 白牆、無紋理環境 → 特徵匹配不足
- 光照劇變（進出隧道、窗邊強光） → 特徵描述子失效
- 快速旋轉 → motion blur 導致 tracking lost
- 距離 anchor 時間/距離過長 → 累積漂移超過可接受範圍

當 VO 自身不可靠時，用它的座標去覆蓋 LiDAR SLAM，反而會**把正常運作的 SLAM 帶偏**，
造成比不修正更嚴重的後果。

此外，目前的閾值設計（平移 0.5m、旋轉 0.3rad）瞄準的是「小幅漂移」，
但 LiDAR SLAM 的典型失敗模式是**突發跳飛（catastrophic jump）**而非緩慢漂移。
0.5m 的閾值與 VO 本身的漂移量級重疊，容易產生 false positive。

## 決策 (Decision)

### 變更 1：VO 健康度評估機制

在每次比對座標之前，先評估 VO 當前的健康狀態。**只有 VO 健康時才允許觸發修正。**

健康度由以下三個指標共同決定：

| 指標 | 來源 | 判定邏輯 | 預設閾值 |
|------|------|---------|---------|
| VO covariance | `nav_msgs/Odometry.pose.covariance` | 對角線元素超過閾值 → 不健康 | 位移 var > 0.5, 旋轉 var > 0.3 |
| Anchor 時效 | 距上次 `set_absolute_pose` 的時間/距離 | 超過上限 → VO 不再可信 | 120 秒 或 50 公尺 |
| VO 資料時效 | 最後一筆 VO odom 的 timestamp | 資料過舊代表 tracking 可能已 lost | > 0.5 秒 |

三項指標全部通過才視為健康。健康狀態將透過 `~/vo_health` topic 發布，
供外部系統（如 fleet management）監控。

**選擇此方案而非其他替代方案的理由：**

- *替代方案 A：僅用 covariance* — 不足。部分場景下 RTAB-Map 的 covariance
  估計過於樂觀，單一指標無法全面反映退化情況。
- *替代方案 B：用 feature inlier 數量* — 可行但需要修改 RTAB-Map 或額外解析其
  內部 topic，耦合度較高。未來可作為健康度的擴充指標加入。
- *選擇的方案：多指標聯合判定* — 使用 odom 訊息中已有的 covariance
  加上可在 node 內部自主計算的時效指標，不增加外部依賴。

### 變更 2：閾值策略調整 — 區分「跳飛偵測」與「漂移偵測」

將現有的單一閾值替換為**雙層閾值架構**：

```
                    ┌──────────────────────────┐
                    │     誤差量測               │
                    │  trans_error, rot_error   │
                    └──────────┬───────────────┘
                               │
                ┌──────────────┴──────────────┐
                │                              │
    ┌───────────▼──────────┐     ┌─────────────▼────────────┐
    │  Layer 1: 跳飛偵測    │     │  Layer 2: 漂移偵測        │
    │  trans > 3.0m         │     │  trans > 1.0m             │
    │  OR rot > 1.0rad      │     │  OR rot > 0.5rad          │
    │                       │     │                            │
    │  連續次數: 2           │     │  連續次數: 10              │
    │  → 高信心，立即修正    │     │  → 低信心，僅告警          │
    └───────────────────────┘     └────────────────────────────┘
```

**Layer 1（跳飛偵測）**— 閾值大、連續次數少、允許自動修正：
- LiDAR SLAM 跳飛通常是瞬間 3-10m 的位移，VO 即使漂移 1m 也能明確識別
- 連續 2 次（在 2Hz 比對頻率下約 1 秒）即可確認，避免單幀 outlier

**Layer 2（漂移偵測）**— 閾值小、連續次數多、僅告警不修正：
- 偵測 1-3m 的漸進漂移，但由於此量級與 VO 漂移重疊，不具備修正信心
- 僅發布 diagnostic 告警，交由上層系統或操作員決策

**選擇此方案而非其他替代方案的理由：**

- *替代方案 A：單純加大閾值到 2-3m* — 會漏掉所有非跳飛類型的異常。
- *替代方案 B：用 SLAM 速度異常檢測（Δpose/Δt）* — 有效但無法偵測
  「SLAM 緩慢飄走」的情境，且需要額外維護速度歷史。可作為未來 Layer 1 的補充。
- *選擇的方案：雙層閾值* — 分離高信心事件（可自動處理）和低信心事件（需人工判斷），
  避免過度自動化帶來的風險。

### 變更 3：告警模式 — Diagnostic 發布

新增 diagnostic 發布機制，使外部系統可監控定位健康狀態：

| Topic | Type | 內容 |
|-------|------|------|
| `~/vo_health` | `diagnostic_msgs/DiagnosticStatus` | VO 健康狀態（OK / WARN / ERROR） |
| `~/slam_deviation` | `visual_localization/SlamDeviation`（新增 msg） | 偏差量測、偏差等級、是否已修正 |

`SlamDeviation.msg` 定義：

```
Header header
float64 translation_error      # meters
float64 rotation_error         # radians
uint8   level                  # 0=NORMAL, 1=DRIFT_WARNING, 2=JUMP_DETECTED, 3=CORRECTED
bool    vo_healthy             # VO 是否在健康狀態
bool    correction_attempted   # 是否嘗試了自動修正
```

修正決策流程變更：

```
偵測到偏差
    │
    ├─ Layer 2 (漂移) ──→ 發布 DRIFT_WARNING ──→ 不自動修正，等待外部決策
    │
    └─ Layer 1 (跳飛) ──→ 檢查 VO 健康度
                              │
                              ├─ VO 健康 ──→ 檢查 cooldown ──→ 自動修正 + 發布 CORRECTED
                              │
                              └─ VO 不健康 ──→ 發布 JUMP_DETECTED ──→ 不修正，僅告警
```

## 實作影響 (Consequences)

### 需修改的檔案

| 檔案 | 變更內容 |
|------|---------|
| `src/visual_localization_node.cpp` | 新增健康度評估邏輯、雙層閾值比對、diagnostic 發布 |
| `config/params.yaml` | 新增健康度參數、雙層閾值參數 |
| `msg/SlamDeviation.msg` | 新增（偏差資訊 message 定義） |
| `CMakeLists.txt` | 加入 `diagnostic_msgs` 依賴、新增 msg 生成 |
| `package.xml` | 加入 `diagnostic_msgs` 依賴 |

### 新增參數（`config/params.yaml`）

```yaml
# VO Health Check
vo_max_covariance_position: 0.5     # Max acceptable position variance
vo_max_covariance_rotation: 0.3     # Max acceptable rotation variance
vo_max_age: 0.5                     # seconds - max staleness of VO data
vo_max_trust_duration: 120.0        # seconds - max time since anchor before VO untrusted
vo_max_trust_distance: 50.0         # meters - max distance from anchor before VO untrusted

# Dual-Layer Thresholds
jump_translation_threshold: 3.0     # meters (Layer 1)
jump_rotation_threshold: 1.0        # radians (Layer 1)
jump_consecutive_threshold: 2       # count (Layer 1)
drift_translation_threshold: 1.0    # meters (Layer 2)
drift_rotation_threshold: 0.5       # radians (Layer 2)
drift_consecutive_threshold: 10     # count (Layer 2)
```

### 移除的參數

| 舊參數 | 替代 |
|--------|------|
| `translation_threshold: 0.5` | 由 `jump_translation_threshold` 與 `drift_translation_threshold` 取代 |
| `rotation_threshold: 0.3` | 由 `jump_rotation_threshold` 與 `drift_rotation_threshold` 取代 |
| `consecutive_threshold: 5` | 由 `jump_consecutive_threshold` 與 `drift_consecutive_threshold` 取代 |

### 正面影響

- **消除 VO 不可靠時的誤修正風險** — 最關鍵的安全性改善
- **降低 false positive** — 跳飛閾值 3m 遠離 VO 漂移範圍
- **提供可觀測性** — 外部系統可透過 diagnostic topic 監控並介入
- **保守設計** — 不確定時選擇不行動（告警而非修正）

### 負面影響 / 取捨

- **複雜度增加** — 從單閾值變雙層 + 健康度檢查，程式碼與參數量增加
- **Layer 2 漂移不會被自動修正** — 需要外部系統或操作員處理，增加運維負擔
- **VO 健康度判定可能過於保守** — 在 covariance 估計準確的場景下，時效限制可能
  導致本可修正的情境被拒絕。可透過調大 `vo_max_trust_duration` 緩解
- **新增 `diagnostic_msgs` 依賴** — 但此為 ROS 標準套件，幾乎所有環境都已安裝

## 備註

- 此 ADR 不改變 RELATIVE_ONLY / ABSOLUTE_MODE 的雙模式架構
- 未來可擴充健康度指標（如 RTAB-Map 的 feature inlier 數、SLAM 速度一致性檢查）
- 閾值預設值基於典型室內 AMR 場景（速度 < 2m/s），室外或高速場景需重新調校
