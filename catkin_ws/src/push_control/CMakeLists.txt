cmake_minimum_required(VERSION 2.8.3)
project(push_control)

##############
## Compiler ##
##############
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
###################
## Find packages ##
###################
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    tf
)

###############
## Set Paths ##
###############
set(EIGEN_INCLUDE_DIR "$ENV{HOME}/software/Eigen")
set(INCLUDE_DIR "$ENV{HOME}/fpush/catkin_ws/src/push_control/include")
set(OOQP_INCLUDE_DIR "$ENV{HOME}/software/OOQP/include")
set(OOQP_LIBRARIES
        "$ENV{HOME}/software/OOQP/lib/libooqpgensparse.a"
        "$ENV{HOME}/software/OOQP/lib/libooqpsparse.a"
        "$ENV{HOME}/software/OOQP/lib/libooqpgondzio.a"
        "$ENV{HOME}/software/OOQP/lib/libooqpbase.a"
                   )
set(MA27_LIBRARIES "$ENV{HOME}/software/OOQP/extras/MA27/libma27.a")
set(JSON_INCLUDE_DIR "$ENV{HOME}/software/json")
set(GUROBI_HOME "$ENV{HOME}/software/gurobi702/linux64")
set(GUROBI_INCLUDE_DIR "${GUROBI_HOME}/include")
set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )

find_library( GUROBI_LIBRARY
              NAMES gurobi70
              PATHS "${GUROBI_HOME}/lib"
              )

find_library( GUROBI_CXX_LIBRARY
              NAMES gurobi_c++
              PATHS "${GUROBI_HOME}/lib"
              )

#find_library(BLAS REQUIRED)
#find_library(GFORTRAN REQUIRED)

#find_library( MA27_LIBRARY
#              NAMES MA27
#              PATHS "${software/OOQP_INCLUDE_DIR}/lib"
#              )

#find_library( software/OOQP_LIBRARY
#              NAMES software/OOQPbase
#              PATHS "${software/OOQP_INCLUDE_DIR}/lib"
#              )



#~ set(software/OOQPEI_LIBS software/OOQPgensparse software/OOQPsparse software/OOQPgondzio software/OOQPbase blas MA27 gfortran)

set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )

## System dependencies are found with CMake's conventions
find_package(Protobuf REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate services in the 'srv' folder
add_service_files(
FILES
MODE_SRV.srv
rosbag.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES package
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

##################
## Proto Folder ##
##################

## Specify additional locations of header files

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${INCLUDE_DIR}
    ${OOQP_INCLUDE_DIR}
    ${GUROBI_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIR}
    src/egm
    src/PracticalSocket
    ${PROTOBUF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

#################
## Executables ##
#################
#src/PracticalSocket/PracticalSocket.cpp src/egm/egm.pb.cc
add_executable(push_control  src/Helper.cpp src/ikfast_interface_python.cpp src/PusherSlider.cpp src/Friction.cpp src/LinePusher.cpp src/OptProgram.cpp src/jsoncpp.cpp src/LoopControl.cpp src/MPC.cpp src/LModes.cpp src/FOM.cpp src/ABBRobot.cpp src/main.cpp src/PracticalSocket/PracticalSocket.cpp src/egm/egm.pb.cc src/Pusher.h src/PointPusher.h src/PointPusher.cpp src/MPC_thread.h src/MPC_thread.cpp src/ik_fast_interface_python.h)
#~ add_executable(push_control  src/Helper.cpp src/PusherSlider.cpp src/Friction.cpp src/LinePusher.cpp src/OptProgram.cpp src/jsoncpp.cpp src/LoopControl.cpp src/MPC.cpp src/FOM.cpp src/ABBRobot.cpp src/main.cpp src/PracticalSocket/PracticalSocket.cpp)

###########################
## Target link libraries ##
###########################
target_link_libraries(push_control
    ${catkin_LIBRARIES}
    ${OOQP_LIBRARIES}
    /etc/alternatives/libblas.so
    ${MA27_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libgfortran.so.3
    ${GUROBI_LIBRARIES}
    dl
    pthread
    ${PROTOBUF_LIBRARY}
        )

#############
## Install ##
#############

#############
## Testing ##
#############
