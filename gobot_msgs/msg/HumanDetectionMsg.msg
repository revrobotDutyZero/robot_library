## This is the output of the human detection module (which actually only face detection for now)
## name is the name of the human detected, while confidence level help with responding
## behaviour
 
int8 CONFIDENCE_NONE = 0   # no human detected, ignore name
int8 CONFIDENCE_LOW  = 2   # not very sure, will stop the robot but will not speak
int8 CONFIDENCE_HIGH = 4   # very sure, will greet

string name
int8 confidence