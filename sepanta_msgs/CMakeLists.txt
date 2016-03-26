cmake_minimum_required(VERSION 2.8.3)
project(sepanta_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs genmsg actionlib_msgs geometry_msgs sensor_msgs nav_msgs nav_msgs message_generation)

add_message_files(
    FILES 
    arm.msg 
    coordinate.msg 
    head.msg 
    irsensor.msg
    joint.msg 
    motortorques.msg 
    omnidata.msg 
    user.msg 
    users.msg
)

add_service_files(
    FILES 
    command.srv 
    maptools.srv 
    stop.srv
    windows.srv
    follow.srv
)

add_action_files(
    FILES 
    avoidThat.action
    avoidthataction.action
    grip.action
    grip_emergency.action
    grip_party.action
    logic_grip.action
    object.action
object_emergency.action
    gesture.action
    object_party.action
    pickAndPlace.action
    sepanta.action
    slamaction.action
    whatDidYouSay.action
    grip.action
)

generate_messages(
    DEPENDENCIES 
    std_msgs 
    actionlib_msgs
)


