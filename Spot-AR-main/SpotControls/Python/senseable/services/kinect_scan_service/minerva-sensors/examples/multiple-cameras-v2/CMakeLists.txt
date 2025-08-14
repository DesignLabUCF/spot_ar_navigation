cmake_minimum_required(VERSION 3.0)
project( multiple_cameras-v2 )


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

target_compile_features(postprocessing PRIVATE cxx_std_11)

#set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -Wall -Wextra -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -Wall -Wextra -pthread")

SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 


FIND_PACKAGE( OpenMP )
      if(OPENMP_FOUND)
         message("OPENMP FOUND")
         set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
         set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
      endif()
include_directories(SYSTEM ${OpenMP_INCLUDE_PATH})



add_executable( multicap main.cpp UART_Connector_wrapper.h UART_Connector_wrapper.cpp Boson_Camera.h Boson_Camera.cpp Frame.h keyboard.hpp keyboard.cpp)


link_directories(~/Desktop/minerva_sensors/build)
link_directories(${CMAKE_BINARY_DIR})

target_link_libraries( multicap ${CMAKE_BINARY_DIR}/libserialPortAdapter.a serialPort FSLP_static ${CMAKE_BINARY_DIR}/libFSLP.so FSLP FLIR_C_SDK UART_Connector ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so ${CMAKE_THREAD_LIBS_INIT})

target_include_directories( multicap PRIVATE ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})

MESSAGE(" The boson directory is : " ${BOSON_INCLUDE_DIR})