#!/bin/bash


shift "$((OPTIND-1))"

if [ $# -eq 0 ]
then
    NR_JOBS=""
    CATKIN_JOBS=""
else
    NR_JOBS=${1:-}
    CATKIN_JOBS="-j${NR_JOBS}"
fi

FILEDIR=$(readlink -f ${BASH_SOURCE})
BASEDIR=$(dirname ${FILEDIR})

echo "File directory: ${BASEDIR}"
cd ${BASEDIR}/..
wstool init
wstool merge covinsuwb/dependencies.rosinstall
wstool up
chmod +x covinsuwb/fix_eigen_deps.sh
./covinsuwb/fix_eigen_deps.sh

set -e
cd ${BASEDIR}/../..
catkin build ${CATKIN_JOBS} msg_utils
catkin build ${CATKIN_JOBS} nlink_parser
catkin build ${CATKIN_JOBS} eigen_catkin opencv3_catkin
cd ${BASEDIR}/../..
source devel/setup.bash

cd ${BASEDIR}/covins_backend/
#DBoW2
cd thirdparty/DBoW2
if [ ! -d "build" ]
then
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j${NR_JOBS}
fi

cd ${BASEDIR}/../..
catkin build ${CATKIN_JOBS} covins_frontend
catkin build ${CATKIN_JOBS} covins_backend

#Extract vocabulary
cd ${BASEDIR}/covins_backend/config
unzip ORBvoc.txt.zip

cd ${BASEDIR}/../..
source devel/setup.bash

cd ${BASEDIR}/../pangolin
if [ ! -d "build" ]
then
  mkdir build
  cd build
  cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
  make -j${NR_JOBS}
fi

exit 0