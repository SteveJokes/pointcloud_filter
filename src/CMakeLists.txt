cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_filter)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(DMCAM REQUIRED)
message(STATUS "DMCAM inc: ${DMCAM_INCLUDE_DIRS}")
message(STATUS "DMCAM lib: ${DMCAM_LIBRARIES}")
if (NOT WIN32)
    # - under linux find pthread and libusb
    find_package(PkgConfig)
    find_package(Threads)
    find_package(LibUSB REQUIRED)

    # - set rpath -
    set(CMAKE_INSTALL_RPATH "'\$ORIGIN'")
endif()

# - set include -
include_directories(${PROJECT_BINARY_DIR}
    ${DMCAM_INCLUDE_DIRS}
    )

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_filter main.cpp img_filter.c)
target_link_libraries (pcl_filter ${PCL_LIBRARIES} ${DMCAM_LIBRARIES} ${LibUSB_LIBRARIES})
