cmake_minimum_required(VERSION 2.8)
project( DisplayImage )
find_package( OpenCV REQUIRED )
add_executable( displayImage Auv_controll.cpp)
target_link_libraries( displayImage ${OpenCV_LIBS} )

