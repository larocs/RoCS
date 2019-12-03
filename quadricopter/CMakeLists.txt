cmake_minimum_required(VERSION 3.9)

set(PROJECT_ALIAS quadricopter)

project(${PROJECT_ALIAS})

set(CMAKE_CXX_STANDARD 17)
add_compile_options("-lpthread")
link_libraries("-lpthread")
IF (CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")
ENDIF (CMAKE_COMPILER_IS_GNUCC)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -DBUILD_SHARED_LIBS=OFF")
# set(CMAKE_VERBOSE_MAKEFILE ON)

INCLUDE(FindPkgConfig)
include_directories(../remote_api_vrep ../FRoCS)
add_subdirectory(../remote_api_vrep vrep_build)
find_package(Threads)

file(GLOB all_SRCS  
  Visualizer/*.h
  Visualizer/*.cpp

  Actuators/*.h
  Actuators/*.cpp

  Sensors/*.h
  Sensors/*.cpp

  Connection/*.h
  Connection/*.cpp

  Robot/*.h
  Robot/*.cpp

  Knowledge/*.h
  Knowledge/*.cpp

  Monitor/*.h
  Monitor/*.cpp

  Analyze/*.h
  Analyze/*.cpp

  Plan/*.h
  Plan/*.cpp

  Execute/Actions/*.h
  Execute/Actions/*.cpp

  main.cpp
)

add_executable(${PROJECT_ALIAS} ${all_SRCS})
target_link_libraries(${PROJECT_ALIAS} ${CMAKE_THREAD_LIBS_INIT} remote_api pthread)
#install(TARGETS ${PROJECT_ALIAS} RUNTIME DESTINATION bin)
