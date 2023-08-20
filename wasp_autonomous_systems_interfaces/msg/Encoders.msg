std_msgs/Header header

# Total number of ticks
int64 encoder_left
int64 encoder_right
# The number of ticks since the last reading
int32 delta_encoder_left
int32 delta_encoder_right
# The time elapsed since the last reading in milliseconds
float64 delta_time_left
float64 delta_time_right
