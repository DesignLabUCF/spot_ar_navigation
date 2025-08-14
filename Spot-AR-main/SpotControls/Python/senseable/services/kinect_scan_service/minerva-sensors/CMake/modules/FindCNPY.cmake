# - Try to find CNPY Numpy writer lib
#
# Once done this will define
#
#  CNPY_FOUND - system has cnpy lib with correct version
#  CNPY_INCLUDE_DIR - the cnpy include directory
#  CNPY_LIBRARIES - the cnpy library

# Other standarnd issue macros
include (FindPackageHandleStandardArgs)
include (SelectLibraryConfigurations)

set (GENERIC_INCLUDE_PATHS
    ${CNPY_CUSTOM_INCLUDE_DIR}
    ${CNPY_CUSTOM_INCLUDE_DIR}/cnpy
    ${CNPY_CUSTOM_INCLUDE_DIR}/cnpy/include
    /usr/local/include/cnpy
    /usr/include
    /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/local/include
    /sw/include
    /opt/local/include )

find_path (CNPY_INCLUDE_DIR   cnpy.h
           PATHS ${GENERIC_INCLUDE_PATHS}  NO_DEFAULT_PATH)


set (GENERIC_LIBRARY_PATHS
    ${GENERIC_LIBRARY_PATHS}
    ${CNPY_CUSTOM_LIB_DIR}/Debug
    ${CNPY_CUSTOM_LIB_DIR}/Release
    ${CNPY_CUSTOM_LIB_DIR}/lib
     ${CNPY_CUSTOM_LIB_DIR}/build
    /usr/lib
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/local/lib
    /usr/local/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /sw/lib
    /opt/local/lib )


include(${CMAKE_SOURCE_DIR}/CMake/SelectLibraryConfigurations.cmake)
set (_cnpy_components_release cnpy-static cnpy )
set (_cnpy_components_debug  cnpy-staticd cnpyd)

foreach (COMPONENT ${_cnpy_components_release})
    find_library (CNPY_LIBRARY_RELEASE ${COMPONENT}
                  PATHS ${GENERIC_LIBRARY_PATHS} NO_DEFAULT_PATH)
endforeach ()

foreach (COMPONENT ${_cnpy_components_debug})
    find_library (CNPY_LIBRARY_DEBUG ${COMPONENT}
                  PATHS ${GENERIC_LIBRARY_PATHS} NO_DEFAULT_PATH)
endforeach ()


if (CNPY_INCLUDE_DIR AND CNPY_LIBRARY_RELEASE)
    set (CNPY_FOUND TRUE)
    select_library_configurations(CNPY)
endif ()
