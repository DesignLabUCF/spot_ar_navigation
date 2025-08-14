cmake_minimum_required(VERSION 2.8)
project( multiple_cameras )


SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 

find_package( Threads )
add_executable( multiple_cameras main.cpp UART_Connector_wrapper.h UART_Connector_wrapper.cpp Boson_Camera.h Boson_Camera.cpp Frame.h)
target_compile_features(postprocessing PRIVATE cxx_std_11)

link_directories(${CMAKE_BINARY_DIR})
link_directories(~/Documents/minerva_sensors/build)
target_link_libraries( multiple_cameras ${CMAKE_BINARY_DIR}/libserialPortAdapter.a serialPort FSLP_static ${CMAKE_BINARY_DIR}/libFSLP.so FSLP FLIR_C_SDK UART_Connector ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so ${CMAKE_THREAD_LIBS_INIT})
target_include_directories( multiple_cameras PRIVATE ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})

