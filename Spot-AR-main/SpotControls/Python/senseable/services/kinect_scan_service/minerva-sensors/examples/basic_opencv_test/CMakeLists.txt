cmake_minimum_required(VERSION 2.8)
project( basic_opencv_test )


SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 

add_executable( basic_opencv_test main.cpp )
target_link_libraries( basic_opencv_test ${OpenCV_LIBS} )