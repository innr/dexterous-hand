# S6-03: dedicated simulation command. Do not use JointState.header.frame_id for epochs.
# All fields required. Joint names use the Menagerie right-hand joint mapping.
builtin_interfaces/Time stamp
uint64 epoch
uint64 sequence
string[16] joint_names
float64[16] position
