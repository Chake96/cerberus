cmake_minimum_required(VERSION 3.15)
project(cerberus CXX)

#conan stuff
include(${CMAKE_BINARY_DIR}/conan_paths.cmake) #adds paths for dependencies

#3rd Party Setup

#Intel TBB Threading
find_package(TBB COMPONENTS tbb REQUIRED)

message("${TBB_IMPORTED_TARGETS}")
##Google
enable_testing()
find_package(GTest)
include(GoogleTest)


find_package(absl REQUIRED)

##Boost
find_package(Boost COMPONENTS program_options REQUIRED)
set(Boost_USE_STATIC_LIBS ON)


##FreeNect
add_subdirectory(libfreenect)


##FTXUI
find_package(ftxui)


#global build setup independent of architcture/compiler 
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(Boost_DEBUG on)
endif()

#Cerberus

add_executable(${PROJECT_NAME}  src/cerberus.cpp src/driver.cpp src/terminal/menu.cpp)
target_include_directories(${PROJECT_NAME}  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include )
target_include_directories(${PROJECT_NAME}  PRIVATE ${Boost_INCLUDE_DIRS})


target_link_libraries(${PROJECT_NAME} PUBLIC absl::absl ${Boost_LIBRARIES} ftxui::screen ftxui::dom ftxui::component TBB::tbb)  

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20 PRIVATE ${TBB_CXX_STD_FLAG})


set_target_properties(cerberus PROPERTIES PUBLIC_HEADER "include/cerberus.h")
install(TARGETS cerberus)


if(UNIX)
    if(CMAKE_BUILD_TYPE MATCHES "Debug")
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
