cmake_minimum_required(VERSION 3.18)
project(SerialReborn)

set(CMAKE_CXX_STANDARD 17)
option(COPY_HIDAPI_DLL "Copy hidapi.dll to the output directory" OFF)
file(
        DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
        EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

CPMAddPackage(
        NAME libusb
        GIT_REPOSITORY https://github.com/libusb/libusb-cmake
        VERSION 1.0.28-0
        OPTIONS "LIBUSB_BUILD_EXAMPLES=OFF" "LIBUSB_BUILD_TESTING=OFF" "LIBUSB_BUILD_SHARED_LIBS=ON"
)

CPMAddPackage(
        NAME hidapi
        GIT_REPOSITORY https://github.com/libusb/hidapi
        GIT_TAG hidapi-0.14.0
        OPTIONS "HIDAPI_BUILD_HIDTEST=OFF" "HIDAPI_WITH_TESTS=OFF"
)

CPMAddPackage(
        NAME libusbp
        GIT_REPOSITORY https://github.com/pololu/libusbp
        GIT_TAG master
        OPTIONS "ENABLE_EXAMPLES=OFF" "ENABLE_TESTS=OFF" "ENABLE_GCOV=OFF"
)

CPMAddPackage(
        NAME Boost
        VERSION 1.88.0 # Versions less than 1.85.0 may need patches for installation targets.
        URL https://github.com/boostorg/boost/releases/download/boost-1.88.0/boost-1.88.0-cmake.tar.xz
        OPTIONS "BOOST_ENABLE_CMAKE ON" "BOOST_SKIP_INSTALL_RULES ON" # Set `OFF` for installation
        "BUILD_SHARED_LIBS OFF" "BOOST_INCLUDE_LIBRARIES container\\\;asio\\\;optional\\\;date_time" # Note the escapes!
)

file(GLOB_RECURSE SRC_FILES
        src/*.cpp
        src/*.h
        src/*.hpp
)

add_library(SerialReborn STATIC ${SRC_FILES})
message(STATUS ${Boost_SOURCE_DIR}/libs/asio/include)
target_link_libraries(SerialReborn PUBLIC usb-1.0 hidapi::hidapi usbp Boost::asio Boost::container)
target_include_directories(SerialReborn PUBLIC src ${Boost_SOURCE_DIR}/libs/asio/include ${Boost_SOURCE_DIR}/libs/container/include ${hidapi_SOURCE_DIR}/hidapi)

IF (WIN32 AND COPY_HIDAPI_DLL)
    add_custom_command(TARGET SerialReborn POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_BINARY_DIR}/_deps/hidapi-build/src/windows/hidapi.dll"
            $<TARGET_FILE_DIR:SerialReborn>)
ENDIF ()