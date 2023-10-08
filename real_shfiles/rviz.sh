source devel/setup.bash;
roslaunch covins_backend tf.launch & sleep 10;
rviz -d ~/project/covinsuwb_ws/src/covinsuwb/covins_backend/config/covins.rviz;