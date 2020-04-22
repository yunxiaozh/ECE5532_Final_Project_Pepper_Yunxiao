cmake_minimum_required(VERSION 2.8.3)
project(human_sim)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    ugv_course_libs
    visualization_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(human_sim src/human_sim.cpp)
target_link_libraries(human_sim ${catkin_LIBRARIES})

add_executable(human_sim1 src/human_sim1.cpp)
target_link_libraries(human_sim1 ${catkin_LIBRARIES})

add_executable(human_sim2 src/human_sim2.cpp)
target_link_libraries(human_sim2 ${catkin_LIBRARIES})

add_executable(human_sim3 src/human_sim3.cpp)
target_link_libraries(human_sim3 ${catkin_LIBRARIES})

add_executable(human_sim4 src/human_sim4.cpp)
target_link_libraries(human_sim4 ${catkin_LIBRARIES})

add_executable(human_sim5 src/human_sim5.cpp)
target_link_libraries(human_sim5 ${catkin_LIBRARIES})

#################
## Test Script ##
#################
#add_executable(test_gps_sim_project test/test_gps_project.cpp)
#target_link_libraries(test_gps_sim_project ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
