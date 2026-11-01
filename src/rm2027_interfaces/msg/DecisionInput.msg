# Atomic policy input: observation and mask always belong to the same sequence.
std_msgs/Header header
uint32 sequence
string schema_hash
float64[] observation
int32[] mask
