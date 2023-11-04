ag_n=$1

source devel/setup.bash

roslaunch msg_utils send.launch ag_n:=${ag_n};
