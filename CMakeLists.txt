# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required(VERSION 2.8.11)
project(libbh)

include("cmake/CheckC11.cmake")
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

# Enable local cmake macros
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Add mLib dependency
set(MLIB_ROOT "$ENV{HOME}/src/mLib")
set(MLIB_INCLUDE_DIR "${MLIB_ROOT}/include")
set(MLIB_SOURCE_DIR "${MLIB_ROOT}/src")
include_directories(${MLIB_INCLUDE_DIR} ${MLIB_SOURCE_DIR})

# Add Eigen dependency
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIR})

# Add boost dependency
set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.56 COMPONENTS program_options REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})
link_directories(${BOOST_LIBRARY_DIRS})
add_definitions(-DBOOST_ALL_DYN_LINK=1)

add_library(
	bh
	SHARED
	src/bh/utilities.cpp
	src/mLib/mLib.cpp
)
target_include_directories(
	bh
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS bh DESTINATION lib/${PROJECT_NAME})

file(GLOB HEADERS include/*.h include/*/*.h include/*/*/*.h)
install(FILES ${HEADERS} DESTINATION include/${PROJECT_NAME})
