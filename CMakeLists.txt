cmake_minimum_required(VERSION 3.15)
project(cerberus CXX)

#conan stuff
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_BINARY_DIR}/generators/conan_toolchain.cmake" CACHE STRING "${CMAKE_CURRENT_BINARY_DIR}/generators/conan_toolchain.cmake")
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR}/generators ${CMAKE_MODULE_PATH})
set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR}/generators ${CMAKE_PREFIX_PATH})
set(CONAN_DISABLE_CHECK_COMPILER ON)


#3rd Party Setup


##Kinect360 lib
find_package(libfreenect)

#Intel TBB Threading
find_package(TBB COMPONENTS tbb REQUIRED)

##OpenCV
find_package(OpenCV REQUIRED core videoio imgproc imgcodecs highgui objdetect)
set(WITH_TBB ON)


##Google
enable_testing()
find_package(GTest)
include(GoogleTest)

find_package(absl REQUIRED)

##Boost
find_package(Boost REQUIRED)
set(Boost_USE_STATIC_LIBS ON)

##SI - Standard Units
find_package(si REQUIRED)

##Lib USB
find_package(libusb REQUIRED)


find_package(PkgConfig)

##LIB UCV
find_package(libuvc COMPONENTS UVC REQUIRED)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
add_library(LibUSB::LibUSB ALIAS libusb::libusb) #needed for LIBUVC 
# if(NOT LIBUVC_FOUND)
#     message(FATAL_ERROR "FAILED TO FIND LIB UVC")
# endif()
# if(PkgConfig_FOUND)
#     pkg_check_modules(uvc)
# endif()

#SPDLog
if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog CONFIG REQUIRED)
    if(PkgConfig_FOUND)
        pkg_check_modules(systemd libsystemd) #for systemd log sink
    endif()
endif()

#SPDLog
if(NOT TARGET CLI11)
    # Stand-alone build
    find_package(CLI11 REQUIRED)
endif()


##FTXUI
find_package(ftxui)


#global build setup independent of architcture/compiler 
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(Boost_DEBUG on)

    #Linting and Tooling Setup
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")



#Cerberus
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

##cerberus terminal files
file(GLOB terminal_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/terminal/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/terminal/*.c")

##cerberus camera files
file(GLOB camera_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/cameras/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/cameras/*.c")


add_executable(${PROJECT_NAME} ${terminal_srcs} ${camera_srcs} ${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/driver.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/cerberus.cpp )
target_include_directories(${PROJECT_NAME} PUBLIC 
                                                include/ 
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
                                                ${Boost_INCLUDE_DIRS}
                                                ${OpenCV_INCLUDE_DIRS}
)


target_link_libraries(${PROJECT_NAME}
                        PRIVATE 
                            absl::base
                            absl::strings
                            absl::str_format
                            absl::utility
                            absl::status
                            Boost::headers
                            opencv::opencv
                            freenect
                            ftxui::screen ftxui::dom ftxui::component
                            TBB::tbb
                            libusb::libusb
                            LibUVC::UVCStatic
                            SI::SI
                            spdlog::spdlog
                            CLI11::CLI11
                            ${systemd_LIBRARIES}
                            
)  

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20  ${TBB_CXX_STD_FLAG})
target_compile_options(${PROJECT_NAME} PRIVATE -ffast-math)


# set_target_properties(cerberus PROPERTIES PUBLIC_HEADER "include/cerberus.h")
# install(TARGETS cerberus)


if(UNIX)
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
        message(STATUS "BUILDING IN DEBUG")
        target_compile_options(cerberus PRIVATE -O0 -g -Wall -Wextra -Walloca -fstack-protector-strong -fasynchronous-unwind-tables)
    else()
        message(STATUS "BUILDING IN RELEASE")
        target_compile_options(cerberus PUBLIC -O2)
    endif()
endif()

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/libfreenect/cmake_modules/")



#Testing
# find_package(libusb-1.0 REQUIRED)
# add_executable(basic_test testing/test.cpp)
# target_link_libraries(basic_test GTest::gtest GTest::gtest_main)
# add_executable(opencv_ex testing/opencv.cpp)
# target_link_libraries(opencv_ex PUBLIC freenect ${OpenCV_LIBS})
# target_include_directories(opencv_ex PUBLIC ${LIBUSB_1_INCLUDE_DIRS} ${OpenCV_LIBS} )
