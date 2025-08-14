cmake_minimum_required(VERSION 2.8)
project( basic_ici_test )


SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 

add_executable( basic_ici_test main.cpp )
#link_directories(${CMAKE_BINARY_DIR})
link_directories(~/Documents/minerva-sensors/build)
target_link_libraries( basic_ici_test   FSLP FLIR_C_SDK UART_Connector ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so)
target_include_directories( basic_ici_test PRIVATE ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})

