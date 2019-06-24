cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Set build type.")
project(find_paths)

add_definitions(-D_FILE_OFSET_BITS=64 -Wno-deprecated)

#set (BOOST_ROOT "/anaconda/pkgs/boost-cpp-1.66.0-1")
#set (BOOST_INCLUDEDIR "/anaconda/pkgs/boost-cpp-1.66.0-1/include")
#set (BOOST_LIBRARYDIR "/anaconda/pkgs/boost-cpp-1.66.0-1/lib")

find_path(BOOST_ROOT boost)

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.42 COMPONENTS system iostreams filesystem thread timer REQUIRED)

include_directories(${Boost_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(find_paths main.cpp KdTrip.hpp CityMap.hpp CityMapInt.hpp CityMapFloat.hpp Cab.hpp CabTrip.hpp Simulator.hpp)
target_link_libraries(find_paths ${Boost_LIBRARIES})

add_executable(split split.cpp KdTrip.hpp)
target_link_libraries(split ${Boost_LIBRARIES})

add_executable(airports airports.cpp KdTrip.hpp CityMapFloat.hpp)
target_link_libraries(airports ${Boost_LIBRARIES})
