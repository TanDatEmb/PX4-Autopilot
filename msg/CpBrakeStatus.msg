# Status of cp_brake for logging and operator information (collision-prevention ICD-CP-04, ICD-CP-05)
uint64 timestamp		# [us] status time, time since system start
uint64 sample_timestamp		# [us] acquisition time of the latest accepted map
uint64 constraint_timestamp	# [us] publication time of the latest constraint

uint32 sample_age_us		# [us] status time minus sample_timestamp
uint32 constraint_age_us	# [us] status time minus constraint_timestamp

uint8 state			# see CpBrakeConstraints STATE_*
uint32 reason_flags		# see CpBrakeConstraints REASON_*
uint32 config_fault_flags	# lib/cp_brake ConfigFaults, non-zero when the configuration is invalid

uint32 input_count		# obstacle_distance messages received
uint32 input_rejected_count	# obstacle_distance messages rejected by the input check

uint8 heading_lookup		# why the heading for sample_timestamp is there or not, HEADING_LOOKUP_*
uint8 HEADING_LOOKUP_OK = 0
uint8 HEADING_LOOKUP_INVALID_PARAMS = 1	# CPB_HDG_DEPTH or CPB_HDG_GAP is not set (OPEN-032)
uint8 HEADING_LOOKUP_EMPTY = 2		# no heading sample yet, or no map yet
uint8 HEADING_LOOKUP_BEFORE_FIRST = 3	# sample_timestamp is older than the oldest kept heading
uint8 HEADING_LOOKUP_AFTER_LAST = 4	# sample_timestamp is newer than the newest heading
uint8 HEADING_LOOKUP_RESET = 5		# the heading reset counter changed around sample_timestamp
uint8 HEADING_LOOKUP_GAP = 6		# the heading samples around sample_timestamp are too far apart
uint8 HEADING_LOOKUP_NOT_GOOD = 7	# a heading sample around sample_timestamp was not good for control

uint8 time_sync_verdict		# time alignment of the latest map that passed the input check, TIME_SYNC_*
uint8 TIME_SYNC_GOOD = 0
uint8 TIME_SYNC_INVALID_LIMITS = 1	# CPB_SYNC_RTT, CPB_SYNC_OFF or CPB_SYNC_AGE is not set (OPEN-033)
uint8 TIME_SYNC_NO_STATUS = 2		# no uXRCE-DDS timesync_status was received
uint8 TIME_SYNC_STALE = 3		# the timesync_status is older than CPB_SYNC_AGE, or from the future
uint8 TIME_SYNC_ROUND_TRIP_TOO_LONG = 4
uint8 TIME_SYNC_OFFSET_DEVIATION = 5
uint8 TIME_SYNC_NOT_JUDGED = 255	# UXRCE_DDS_SYNCT is off (simulation profile): there is nothing to judge

uint8 heading_history_size	# heading samples in the history
uint32 heading_sample_count	# heading samples accepted into the history
uint32 heading_rejected_count	# heading samples the history refused (no time, or a time that did not advance)
uint32 time_sync_rejected_count	# maps not used because their time alignment was not good
