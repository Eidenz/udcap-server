cmake_minimum_required(VERSION 3.16)
project(udcap_steamvr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# OpenVR — we only need the driver header (the driver implements the interfaces;
# it does not link openvr_api), so just fetch the sources, don't build them.
include(FetchContent)
FetchContent_Declare(
        openvr
        GIT_REPOSITORY https://github.com/ValveSoftware/openvr.git
        GIT_TAG v2.5.1
        GIT_SHALLOW TRUE)
FetchContent_GetProperties(openvr)
if(NOT openvr_POPULATED)
        FetchContent_Populate(openvr)
endif()

add_library(driver_udcap MODULE
        src/driver_factory.cpp
        src/provider.cpp
        src/glove_device.cpp
        src/hand_simulation.cpp)

target_include_directories(driver_udcap PRIVATE
        ${openvr_SOURCE_DIR}/headers
        ${CMAKE_CURRENT_SOURCE_DIR}/../shm)

target_link_libraries(driver_udcap PRIVATE rt pthread)

# SteamVR loads <root>/udcap/bin/linux64/driver_udcap.so (no "lib" prefix).
set_target_properties(driver_udcap PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/udcap/bin/linux64)
