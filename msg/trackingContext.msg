# message per object - tracking object context when detected or when expecting item but missing from DNN

Header header
int16 object_id #uid of object
string object_name #name of object
float32 object_confidence #dnn confidence of uid object
int16 object_detected #times uid object was detected

float64 object_weighting #uid object weighting
float64 object_uniqueness #uid object uniqueness
float64 object_score #score result from object weight and uniqueness
int16 object_instances #instances of object name in env

float64 object_timestamp #timestamp when object was tracked (detected or expected but missing
float64 duration #time since start of node

int16 detected_or_missing #1 for object detected, 0 for object expected but missing