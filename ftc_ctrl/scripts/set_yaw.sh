#!/bin/bash

rostopic pub -1 $2/heading_design std_msgs/Float64 -- $1


