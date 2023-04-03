cmake_minimum_required(VERSION 2.8.3)
project(bb_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    geometry_msgs
    message_generation
    )

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES battleCmd.msg
    bbVision2point.msg
    robotStatus.msg
    networkStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)


# cmake -D CMAKE_BUILD_TYPE=RELEASE \
#       -D CMAKE_INSTALL_PREFIX=/usr/local \
#       -D WITH_CUDA=ON \
#       -D ENABLE_FAST_MATH=1 \
#       -D CUDA_FAST_MATH=1 \
#       -D WITH_CUBLAS=1 \
#       -D CUDA_ARCH_BIN="6.1 7.5 8.6" \
#       -D CUDA_ARCH_PTX="" \
#       -D WITH_CUDNN=ON \
#       -D OPENCV_DNN_CUDA=ON \
#       -D BUILD_opencv_python2=OFF \
#       -D BUILD_opencv_python3=ON \
#       -D INSTALL_PYTHON_EXAMPLES=ON \
#       -D INSTALL_C_EXAMPLES=OFF \
#       -D BUILD_EXAMPLES=ON \
#       -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
#       ../
