cmake_minimum_required(VERSION 3.17)
project(joinMap)

set( CMAKE_BUILD_TYPE Debug )
set(CMAKE_CXX_STANDARD 14)

# opencv
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen
include_directories( "/usr/include/eigen3/" )

# pcl
find_package( PCL REQUIRED COMPONENT common io visualization)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp CsvFileReader.cpp CsvFileReader.h Config.cpp Config.h FileUtil.cpp FileUtil.h )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )