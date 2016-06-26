project(SolvePnP)
cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )
#MESSAGE("俺们正在生成项目文件")
#Show Images?
#ADD_DEFINITIONS(-DSHOW_DEBUG_IMAGES)
#

link_libraries (pthread)
#link_directories(
#/home/ubuntu/lib
#/home/noah/lib
#)

#INCLUDE_DIRECTORIES( #相关头文件的目录
#${PROJET_SOURCE_DIR}
#/home/ubuntu/include
#/home/noah/include
#)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( 
    ${PROJECT_NAME} 
    ${OpenCV_LIBS} 

    )

add_library(camerareader SHARED
CameraCalibration.cpp
CameraCalibration.hpp
CameraReader.cpp
CameraReader.h
GeometryTypes.cpp
GeometryTypes.hpp
Marker.cpp
Marker.hpp
MarkerDetector.cpp
MarkerDetector.hpp
TinyLA.cpp
TinyLA.hpp
)
target_link_libraries(
    camerareader 
    ${OpenCV_LIBS} 
    pthread 

    )
set( CMAKE_BUILD_TYPE Debug )


