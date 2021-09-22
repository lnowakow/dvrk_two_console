cmake_minimum_required(VERSION 3.0.2)
project(dvrk_two_console)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        roscpp
        rospy
        sensor_msgs
        geometry_msgs
        std_msgs
        tf2_eigen
        crtk_msgs
        )

set(HEADER_FILES
        include/stpTeleOperationCursor.h
        include/stpStateMachine.h
        include/stpCallableVoidMethod.h
        include/stpCallableVoidBase.h
        include/stpMatrix.h
        )
set(LIBRARY_FILES
        libs/stpTeleOperationCursor.cpp
        libs/stpStateMachine.cpp
        libs/stpCallableVoidMethod.cpp
        libs/stpCallableVoidBase.cpp
        libs/stpMatrix.cpp
        )

add_library(stpLibrary
        ${HEADER_FILES}
        ${LIBRARY_FILES})

catkin_package(
# INCLUDE_DIRS include
  LIBRARIES dvrk_two_console
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs crtk_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mtm_follow src/mtm_follow.cpp)
target_link_libraries(mtm_follow
        ${catkin_LIBRARIES})

add_executable(cursor src/cursor.cpp)
target_link_libraries(cursor
        ${catkin_LIBRARIES} stpLibrary)
