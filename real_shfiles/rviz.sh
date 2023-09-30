source devel/setup.bash;
roslaunch covins_backend tf.launch & sleep 10;
rviz -d ~/project/covinsg_ws/src/covinsg/covins_backend/config/covins.rviz;