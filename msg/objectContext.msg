# Object context, decided by detected object instances and uniqueness

Header header
int16[] object_id #uid of object
string[] object_name #name of object
float32[] object_confidence #dnn confidence of uid object
int16[] object_detected #times uid object was detected

float64[] object_weighting #uid object weighting
float64[] object_uniqueness #uid object uniqueness
float64[] object_score #score result from object weight and uniqueness
int16[] object_instances #instances of object name in env

int16 totalObjects #total objects in struct