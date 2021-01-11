
#!/bin/bash
rostopic pub -1 reference_pos geometry_msgs/Point -- 1.0 -1.0 1.0

sleep 2.0
# wp2
rostopic pub -1 reference_pos geometry_msgs/Point -- 2.0 -1.0 1.0

sleep 2.0

# wp3
rostopic pub -1 reference_pos geometry_msgs/Point -- 3.0 -1.0 1.0

sleep 2.0

# wp3.5
rostopic pub -1 reference_pos geometry_msgs/Point -- 3.0 0.0 1.0

sleep 2.0
# wp4
rostopic pub -1 reference_pos geometry_msgs/Point -- 3.0 1.0 1.0

sleep 2.0

rostopic pub -1 reference_pos geometry_msgs/Point -- 2.0 1.0 1.0

sleep 2.0

rostopic pub -1 reference_pos geometry_msgs/Point -- 1.0 1.0 1.0

sleep 2.0

rostopic pub -1 reference_pos geometry_msgs/Point -- 1.0 0.0 1.0

sleep 2.0

rostopic pub -1 reference_pos geometry_msgs/Point -- 1.0 -1.0 1.0

sleep 5.0

rostopic pub -1 reference_pos geometry_msgs/Point -- 1.0 -1.0 0.0

sleep 2.0
# kill
rostopic pub -1 emergency_kill std_msgs/Empty --


