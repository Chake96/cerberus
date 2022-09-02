cmake_minimum_required(VERSION 3.15)
project(cerberus CXX)




#conan stuff
include(${CMAKE_CURRENT_BINARY_DIR}/conan_paths.cmake) #adds paths for dependencies
include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

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
# find_package(libusb-1.0 REQUIRED)
find_path(LIBUSB_INCLUDE_DIR
  NAMES libusb.h
  PATH_SUFFIXES "include" "libusb" "libusb-1.0")
find_library(LIBUSB_LIBRARY
  NAMES usb
  PATH_SUFFIXES "lib" "lib32" "lib64")
include_directories(${LIBUSB_1_INCLUDE_DIRS})

#SPDLog
if(NOT TARGET spdlog)
    # Stand-alone build
    find_package(spdlog REQUIRED)
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
        pkg_check_modules(systemd libsystemd) #for systemd log sink
    endif()
endif()

#SPDLog
if(NOT TARGET CLI11)
    # Stand-alone build
    find_package(CLI11 REQUIRED)
endif()

# --------------------


##FreeNect
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_BINARY_DIR}/conan_toolchain.cmake" CACHE STRING "")

##FTXUI
find_package(ftxui)


#global build setup independent of architcture/compiler 
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(Boost_DEBUG on)

    #Linting and Tooling Setup
    set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE CACHE BOOL true)
endif()

#Cerberus

##cerberus terminal files
file(GLOB terminal_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/terminal/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/terminal/*.c")

##cerberus camera files
file(GLOB camera_srcs "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/cameras/*.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/cameras/*.c")


add_executable(${PROJECT_NAME} ${terminal_srcs} ${camera_srcs} ${CMAKE_CURRENT_SOURCE_DIR}/src/cerberus/driver.cpp )
target_include_directories(${PROJECT_NAME}  PUBLIC include/ ${Boost_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}  PRIVATE ${OpenCV_INCLUDE_DIRS} )


target_link_libraries(${PROJECT_NAME} 
                        PUBLIC 
                            absl::absl
                            Boost::headers
                            ${OpenCV_LIBS} 
                            freenect
                            ftxui::screen ftxui::dom ftxui::component
                            TBB::tbb
                            SI::SI
                        PRIVATE
                            spdlog::spdlog
                            CLI11::CLI11
                            ${systemd_LIBRARIES}
)  

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20 PRIVATE ${TBB_CXX_STD_FLAG})
target_compile_options(${PROJECT_NAME} PRIVATE -ffast-math)


# set_target_properties(cerberus PROPERTIES PUBLIC_HEADER "include/cerberus.h")
# install(TARGETS cerberus)


if(UNIX)
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
        message(STATUS "BUILDING IN DEBUG")
        target_compile_options(cerberus PUBLIC -O0 -g -Wall -Wextra -Walloca -fstack-protector-strong -fasynchronous-unwind-tables)
    else()
        message(STATUS "BUILDING IN RELEASE")
        target_compile_options(cerberus PUBLIC -O2)
    endif()
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/libfreenect/cmake_modules/")



#Testing
# find_package(libusb-1.0 REQUIRED)
add_executable(basic_test testing/test.cpp)
target_link_libraries(basic_test GTest::gtest GTest::gtest_main)
add_executable(opencv_ex testing/opencv.cpp)
target_link_libraries(opencv_ex PUBLIC freenect ${OpenCV_LIBS})
target_include_directories(opencv_ex PUBLIC ${LIBUSB_1_INCLUDE_DIRS} ${OpenCV_LIBS} )
