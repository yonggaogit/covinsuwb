ag_n=$1

sudo chmod 777 /dev/ttyACM1 & sleep 2;
source devel/setup.bash;

roslaunch msg_utils uwb_transfer.launch ag_n:=${ag_n} & sleep 2;
roslaunch nlink_parser linktrack.launch  & sleep 2;
