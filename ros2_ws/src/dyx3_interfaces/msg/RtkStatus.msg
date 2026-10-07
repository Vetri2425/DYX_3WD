builtin_interfaces/Time stamp
# All fields default to zero; FIX_UNKNOWN is safe. correction_age_s is seconds,
# horizontal_accuracy_m is metres, satellites_used is a non-negative count.
uint8 FIX_UNKNOWN=0
uint8 FIX_NONE=1
uint8 FIX_3D=2
uint8 FIX_RTK_FLOAT=5
uint8 FIX_RTK_FIXED=6
uint8 fix_type
bool corrections_fresh
float32 correction_age_s
float32 horizontal_accuracy_m
uint8 satellites_used
