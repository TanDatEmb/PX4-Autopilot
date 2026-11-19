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
