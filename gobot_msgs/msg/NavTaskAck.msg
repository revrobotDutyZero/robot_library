# This message to signal ACK/NACK of the navigation task
# The different compared to the progress message is that this will also
# cover problems like type leading to not able to call service etc
# Who should send this message? The UdpControl can 
# string sender_id    # generally speaking this should be the id of the robot
#                     # if this field left empty, the viewer node will autofill
#                     # it with common robot_id
#                     # if it is not empty then the viewer should respect

# string receiver_id  # id of the f*cker that gave us work

# string cmd_id

TaskMetadata metadata

uint8 ack

string msg