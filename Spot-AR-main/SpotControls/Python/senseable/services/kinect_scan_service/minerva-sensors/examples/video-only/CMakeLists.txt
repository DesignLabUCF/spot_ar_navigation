cmake_minimum_required(VERSION 2.8)
project( video_only )


SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 

add_executable( video_only main.cpp )
#link_directories(${CMAKE_BINARY_DIR})
link_directories(~/Documents/minerva_sensors/build)
target_link_libraries( video_only  FSLP FLIR_C_SDK UART_Connector ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so)
target_include_directories( video_only PRIVATE ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})

