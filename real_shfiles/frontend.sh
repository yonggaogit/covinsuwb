ag_n=$1

source devel/setup.bash;
roslaunch covins_frontend covins_frontend_real.launch ag_n:=${ag_n};