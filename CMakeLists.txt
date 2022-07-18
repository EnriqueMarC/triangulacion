cmake_minimum_required(VERSION 3.0)
project(triangulacion)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) 
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
  tf
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
)


add_message_files(
	FILES
	PoseAruco.msg
	VectorAruco.msg
)

#add_service_files(
#   FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime )
#include_directories( "/home/omar/aruco/src")
include_directories( ${OpenCV_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(detection src/detection.cpp)
target_link_libraries(detection  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_dependencies(detection triangulacion_generate_messages_cpp)

add_executable(Algoritmo_Triangulacion_Geometrica src/Algoritmo_Triangulacion_Geometrica.cpp)
target_link_libraries(Algoritmo_Triangulacion_Geometrica  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_dependencies(Algoritmo_Triangulacion_Geometrica triangulacion_generate_messages_cpp)

add_executable(trayectorias src/trayectorias.cpp)
target_link_libraries(trayectorias  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_dependencies(trayectorias triangulacion_generate_messages_cpp)

add_executable(fractal_tracker src/fractal_tracker.cpp)
target_link_libraries(fractal_tracker  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_dependencies(fractal_tracker triangulacion_generate_messages_cpp)

add_executable(control src/control.cpp)
target_link_libraries(control  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_dependencies(control triangulacion_generate_messages_cpp)

add_executable(image_subscriber src/image_subscriber.cpp)
target_link_libraries(image_subscriber  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
