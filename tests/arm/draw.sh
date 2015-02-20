#!/bin/bash

# vals x y z gp gr gripval delta

rostopic pub /armcontrol/target std_msgs/String '.1 .2 0. .0 0. 1. 255' -1


#rostopic pub /armcontrol/target std_msgs/String '0. .35 0. 0. 0. 1. -1' -1
#rostopic pub /armcontrol/target std_msgs/String '0. .35 0. 0. 0. 1. -2' -1

