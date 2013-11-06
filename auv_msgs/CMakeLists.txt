cmake_minimum_required(VERSION 2.8.3)
project(auv_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs roscpp std_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    Belief.msg       MapPoint.msg        ParkingTask.msg     PWMValues.msg
    BufferComm.msg   MarkerDetails.msg   Path.msg            SetPoint.msg
    BuoyDetails.msg  McbData.msg         PlankDetails.msg    TaskState.msg
    FramePoint.msg   McbDebug.msg        Pneumatic.msg       TorpedoDetails.msg
    GateDetails.msg  OctagonDetails.msg  Pose.msg            Velocity.msg
    HydStatus.msg    Orientation.msg     PressureSensor.msg
    Map.msg          ParkingDetails.msg  PwbStatus.msg
)

## Generate services in the 'srv' folder
add_service_files(
    DIRECTORY srv
    FILES
    dropMarker.srv  IMUFilter.srv  shootTorpedo.srv  UpdateParams.srv
    HueColors.srv   setBelief.srv  SimInterrupt.srv
)

add_action_files(
    DIRECTORY action
    FILES
    ExecutionState.action ScanState.action TransitionState.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs roscpp std_msgs
)

