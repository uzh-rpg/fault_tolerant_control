
#!/bin/bash

rostopic pub -1 $4/reference_pos geometry_msgs/Point -- $1 $2 $3

