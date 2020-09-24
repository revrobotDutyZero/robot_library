### This message hold the basic info of a task. Most likely this will not
### be used alone but in other message, service and action

# For now we assume that the name is unique
string sender_id    # for cmd from wobo, it is the id of the GUI
string receiver_id  # id of the robot, check before executing cmd

string name     	# name of the task, for human
					# For path this can be name of the path
					# Or name of charging station in case of docking in case
					# there are multiple charging station
					# note that breaking down the task depending on the 
					# charging station that we want is the job of the 
					# mission_control, not here

string id       	# sender ID with timestamp for tracking purpose

uint8 priority  	# priority of the task, who should decide this? only ESCANOR desu

string desc     	# A short desciption of the task is encouraged for showing
                	# on GUI and for debugging

### This 2 from Wobo, dont know for what
string cmd_type     # for service type like delivery or customer service
                    # not important for now
string cmd_stuff    # additional information for the task
                    # not important for now
