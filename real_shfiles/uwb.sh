ag_n=$1

roslaunch msg_utils uwb_transfer.launch ag_n:=${ag_n} & sleep 2;
roslaunch nlink_parser linktrack.launch;
