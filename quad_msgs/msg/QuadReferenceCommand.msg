# Quadrotor outerloop reference
# This Message is published by a controller

Header header

#
int8 estimator_id

# 3D position in world frame [m]
geometry_msgs/Vector3 position

# 3D velocity in world frame [m/s]
geometry_msgs/Vector3 velocity

# Acc command
geometry_msgs/Vector3 acceleration

