# S6-03/S7-00 dedicated simulation command. session_id is a process boot nonce; 0 is invalid.
# Joint names use canonical LEAP order (index_mcp_side .. thumb_ip).
builtin_interfaces/Time stamp
uint64 session_id
uint64 epoch
uint64 sequence
string[16] joint_names
float64[16] position
