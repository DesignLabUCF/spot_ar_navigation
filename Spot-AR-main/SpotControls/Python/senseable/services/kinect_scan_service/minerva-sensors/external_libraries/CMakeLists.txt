# =======================================================================
#   WARNING    WARNING    WARNING    WARNING    WARNING    WARNING
# =======================================================================
#   Remember to put on SAFETY GOGGLES before looking at this file. You
#   are most certainly not expected to read or understand any of it.
# =======================================================================
# This CMake file is responsible for compiling dependency libraries and
# setting up suitable compiler flags for various platforms. You do not
# need to read or change anything in this file; see CMakeLists.txt 


#Build Zlib ##############################################
IF(WIN32) 
   SET(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "Add a suffix, usually d on Windows" FORCE)
   set(ZLIB_ROOT ${CMAKE_BINARY_DIR}/Debug ${CMAKE_BINARY_DIR}/Release  ${CMAKE_CURRENT_SOURCE_DIR}/zlib)
else()
   set(ZLIB_ROOT ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/zlib)
endif()
include_directories(${CMAKE_SOURCE_DIR}/external_libraries/zlib)

ADD_SUBDIRECTORY(zlib)
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/zlib/zconf.h)
  message(STATUS "Moving zconf back since who knows why its moved in the first place...")
  file(RENAME ${CMAKE_CURRENT_SOURCE_DIR}/zlib/zconf.h.included ${CMAKE_CURRENT_SOURCE_DIR}/zlib/zconf.h)
endif()


IF(WIN32) 
  SET(ZLIB_LIBRARY_RELEASE "${CMAKE_BINARY_DIR}/$(CONFIGURATION)/zlib.lib")
  SET(ZLIB_LIBRARY_DEBUG "${CMAKE_BINARY_DIR}/$(CONFIGURATION)/zlibd.lib")
ENDIF()
find_package(ZLIB)
if(NOT ${ZLIB_FOUND})
  message(STATUS "Setting ZLIB variables manually...")
  IF(WIN32) 
    SET(ZLIB_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/zlib")
    include(${CMAKE_SOURCE_DIR}/CMake/SelectLibraryConfigurations.cmake)
    select_library_configurations(ZLIB)
    add_definitions(-DWIN32)
  ELSE()
     SET(ZLIB_LIBRARY z CACHE INTERNAL "" FORCE)
  ENDIF()
  SET(ZLIB_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/zlib" CACHE INTERNAL "" FORCE)
ENDIF()

set(CNPY_LIBRARIES  cnpy)
set(ZLIB_LIBRARIES  z)
set(ZLIB_INCLUDE_DIRS  ${CMAKE_SOURCE_DIR}/external_libraries/zlib)
set(CNPY_INCLUDE_DIR  ${CMAKE_SOURCE_DIR}/external_libraries/cnpy)

Message(STATUS " ZLIB has been found... " ${ZLIB_FOUND})
Message(STATUS " ZLIB version: " ${ZLIB_VERSION_STRING})
Message(STATUS " -ZLIB_INCLUDE_DIRS is set to: " ${ZLIB_INCLUDE_DIRS})
Message(STATUS " -ZLIB_LIBRARY is set to: " ${ZLIB_LIBRARY})
Message(STATUS " -ZLIB_LIBRARY-rr is set to: " ${ZLIB_LIBRARY_RELEASE})
Message(STATUS " -ZLIB_LIBRARY--dd is set to: " ${ZLIB_LIBRARY_DEBUG})
message(STATUS "ZLIB ROOT: " ${ZLIB_ROOT})

# NUMPY numerical file writer ##############################################################
SET(NUMPY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cnpy)
add_library(cnpy STATIC "${NUMPY_INCLUDE_DIR}/cnpy.cpp" "${NUMPY_INCLUDE_DIR}/cnpy.h")
target_include_directories(cnpy PRIVATE ${NUMPY_INCLUDE_DIR} ${ZLIB_INCLUDE_DIRS})
add_library(cnpy-shared SHARED "${NUMPY_INCLUDE_DIR}/cnpy.cpp" "${NUMPY_INCLUDE_DIR}/cnpy.h")
target_include_directories(cnpy-shared PRIVATE ${NUMPY_INCLUDE_DIR} ${ZLIB_INCLUDE_DIRS})
target_link_libraries(cnpy-shared ${ZLIB_LIBRARIES})


ADD_SUBDIRECTORY(BosonSDK)
ADD_SUBDIRECTORY(ici-flir)



#Build OPENCV ##############################################


if(BUILD_OPENCV_STATIC_LIBS)
    ADD_SUBDIRECTORY(opencv)
endif(BUILD_OPENCV_STATIC_LIBS)
