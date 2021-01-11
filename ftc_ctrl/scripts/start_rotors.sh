#!/bin/bash

rostopic pub -1 $1/start_rotors std_msgs/Empty --

./reference.sh 1.75 -0.0 1.0 $1

#sleep 20s


