project(mrmp)


cmake_minimum_required(VERSION 3.16)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )

set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
# set(CMAKE_CXX_FLAGS_RELEASE "-DNDEBUG  -O3")


file(GLOB DDM_src "./*.cpp")

add_executable(drp ${DDM_src})
target_include_directories(drp  PUBLIC "./libMultiRobotPlanning/include/" )
target_link_libraries(drp PUBLIC  stdc++fs)
target_compile_features(drp PUBLIC cxx_std_17)