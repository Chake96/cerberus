cmake_minimum_required(VERSION 3.15)
project(cerberus CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED 1)

#conan stuff
include(${CMAKE_BINARY_DIR}/conan_paths.cmake) #adds paths for dependencies

#3rd Party Setup
#Google
enable_testing()
find_package(GTest REQUIRED)
include(GoogleTest)


find_package(absl REQUIRED)


# Use the global target

#Boost


add_executable(${PROJECT_NAME}  src/cerberus.cpp src/driver.cpp)
target_include_directories(${PROJECT_NAME}  PUBLIC include)
target_link_libraries(${PROJECT_NAME} absl::absl)


set_target_properties(cerberus PROPERTIES PUBLIC_HEADER "include/cerberus.h")
set_target_properties(cerberus PROPERTIES CXX_STANDARD 20)
install(TARGETS cerberus)


if(UNIX)
    string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)
    if(BUILD_TYPE MATCHES "DEBUG")
        message(STATUS "BUILDING IN DEBUG")
        target_compile_options(cerberus PUBLIC -O0 -g -Wall -Wextra -Walloca -fstack-protector-strong -fasynchronous-unwind-tables)
    else()
        message(STATUS "BUILDING IN RELEASE")
        target_compile_options(cerberus PUBLIC -O2)
    endif()
endif()



#Testing
add_executable(basic_test testing/test.cpp)
target_link_libraries(basic_test GTest::gtest GTest::gtest_main)
gtest_discover_tests(basic_test)
