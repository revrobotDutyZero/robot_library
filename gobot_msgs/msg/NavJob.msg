# I really wanted to call this an action, but afraid it may cause confusion
# with ros actionlib
# Equivalent to the current point that we have. A chore is going somewhere
# doing something
# For now it is just going there then wait for the GUI to do something

string name                         # name of the point
# uint16 delay_after                  # delay AFTER the action
gobot_msgs/PoseWithTolerance pose   # Final pose that the robot should be
                                    # Add covariance for allowed tolerance?
string media_path       # path to the media to be played at the point
bool wait_media         # whether the robot wait the media before continue moving

# geometry_msgs::Pose p
bool is_home
float64 delay_point
string text
float64 delay_sound
int8 audio_index
