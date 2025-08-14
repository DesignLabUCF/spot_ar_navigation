
MESSAGE(STATUS "Creating Project to Test CNPY...")

set(CNPY_LIBRARIES  cnpy)
set(ZLIB_LIBRARIES  z)
set(ZLIB_INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}/external_libraries/zlib)
set(CNPY_INCLUDE_DIR  ${CMAKE_SOURCE_DIR}/external_libraries/cnpy)

Message(STATUS " CNPY has been found... " ${ZLIB_LIBRARIES})
Message(STATUS " CNPY has been found... " ${CNPY_LIBRARIES})
Message(STATUS " CNPY has been found... " ${CNPY_INCLUDE_DIR})
Message(STATUS " CNPY has been found... " ${ZLIB_INCLUDE_DIRS})

SET(OpenCV_LIBS opencv_core opencv_highgui opencv_imgproc) 


add_executable(postprocessing main.cpp)
target_compile_features(postprocessing PRIVATE cxx_std_17)
target_link_libraries(postprocessing ${ZLIB_LIBRARIES} ${CNPY_LIBRARIES}  ${OpenCV_LIBS} ${CMAKE_BINARY_DIR}/libiciflirbosonrawcountstotemperatureincelsiusconverter.so)
target_include_directories(postprocessing PRIVATE ${CNPY_INCLUDE_DIR} ${ZLIB_INCLUDE_DIRS} ${ICI_INCLUDE_DIR} ${BOSON_INCLUDE_DIR})
add_dependencies(postprocessing cnpy zlib)
