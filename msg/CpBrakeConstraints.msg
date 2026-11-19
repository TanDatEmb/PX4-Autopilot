# Per-sector speed caps from cp_brake to the limiter hook (collision-prevention ICD-CP-04)
uint64 timestamp		# [us] publication time, time since system start
uint64 sample_timestamp		# [us] acquisition time of the map the caps come from, PX4 time

float32 heading_rad		# [rad] vehicle heading at sample_timestamp, rotates the body FRD caps to local NED
bool heading_valid		# false when no heading exists for sample_timestamp

float32[72] caps_m_s		# [m/s] speed cap per sector, body FRD, sector 0 forward, clockwise, >= 0
bool enforcement_active		# the hook may act on the caps
bool valid			# the caps may be used; when false the hook follows the ICD-CP-04 fallbacks

uint8 state
uint8 STATE_DISABLED = 0
uint8 STATE_OBSERVER_ONLY = 1
uint8 STATE_NOT_READY = 2
uint8 STATE_ACTIVE_CLEAR = 3
uint8 STATE_BRAKING = 4
uint8 STATE_SAFE_STOP = 5

uint32 reason_flags		# why the state is what it is (the lib/cp_brake ReasonFlags)
uint32 REASON_INVALID_CONFIG = 1
uint32 REASON_FRAME_INVALID = 2
uint32 REASON_DATA_INVALID = 4
