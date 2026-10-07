builtin_interfaces/Time stamp
# All fields default to zero; RESULT_NONE is safe. Position is local NED metres
# and error_m is a non-negative Euclidean error in metres.
uint32 mission_id
uint32 point_index
uint8 RESULT_NONE=0
uint8 RESULT_COMPLETED=1
uint8 RESULT_SKIPPED=2
uint8 RESULT_FAILED=3
uint8 result_code
float32 north_m
float32 east_m
float32 error_m
