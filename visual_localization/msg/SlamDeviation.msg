Header header

float64 translation_error       # meters - distance between VO and SLAM poses
float64 rotation_error          # radians - angular difference between VO and SLAM poses

uint8 LEVEL_NORMAL=0
uint8 LEVEL_DRIFT_WARNING=1
uint8 LEVEL_JUMP_DETECTED=2
uint8 LEVEL_CORRECTED=3
uint8 level                     # deviation severity level

bool vo_healthy                 # whether VO passed all health checks at this moment
bool correction_attempted       # whether an automatic SLAM correction was attempted
