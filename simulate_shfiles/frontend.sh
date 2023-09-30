ag_n=$1

source devel/setup.bash;
roslaunch covins_frontend vins_euroc_agent.launch ag_n:=${ag_n};