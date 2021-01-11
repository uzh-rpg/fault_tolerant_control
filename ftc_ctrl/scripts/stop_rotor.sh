#!/bin/bash

rostopic pub -1 $1/stop_rotors std_msgs/Empty --
