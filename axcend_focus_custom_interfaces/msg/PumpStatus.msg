std_msgs/Header header

# Index 0 is sensor 0, index 1 is sensor 1
int32[2] pressure # In PSI
int32[2] position # In % of full stroke
int32 flow_rate # In uL/min * 100. ie 100 = 1 uL/min
uint8 phase