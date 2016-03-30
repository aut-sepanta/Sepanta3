uint8 STATUS_UNKNOWN = 0
uint8 STATUS_RECOGNIZED = 1

sensor_msgs/PointCloud2 points # All of the object's points
string label # Label if object is recognized
uint8 status
