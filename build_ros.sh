echo "Building ROS nodes"

cd Examples_old/ROS/ORB_SLAM3
mkdir build
cd build
sudo rosdep init
rosdep update
cmake .. -DROS_BUILD_TYPE=Release
make
