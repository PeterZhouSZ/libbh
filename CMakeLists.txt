# CMakeLists files in this project can
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and
# to the root binary directory of the project as ${HELLO_BINARY_DIR}.
cmake_minimum_required(VERSION 2.8.11)
project(libbh)

set(LIBBH_MAJOR_VERSION 0)
set(LIBBH_MINOR_VERSION 1)
set(LIBBH_PATCH_VERSION 0)
set(LIBBH_VERSION
		${LIBBH_MAJOR_VERSION}.${LIBBH_MINOR_VERSION}.${LIBBH_PATCH_VERSION})

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
	set(DEF_INSTALL_CMAKE_DIR CMake)
else()
	set(DEF_INSTALL_CMAKE_DIR lib/CMake/libbh)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
	set(var INSTALL_${p}_DIR)
	if(NOT IS_ABSOLUTE "${${var}}")
		set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
	endif()
endforeach()

# C++11 support
include("cmake/CheckC11.cmake")

# G++ >= 5 ABI
set(USE_NEW_GCC_ABI True CACHE BOOL "Whether to compile with new GCC >= 5 ABI")
if(USE_NEW_GCC_ABI)
	add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)
endif()

# Enable local cmake macros
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Add mLib dependency
set(MLIB_ROOT "$ENV{HOME}/src/mLib" CACHE PATH "Root path of mLib library")
set(MLIB_INCLUDE_DIR "${MLIB_ROOT}/include")
set(MLIB_SOURCE_DIR "${MLIB_ROOT}/src")
include_directories(${MLIB_INCLUDE_DIR} ${MLIB_SOURCE_DIR})
# Add external mLib dependencies
#set(MLIB_EXTERNAL_ROOT "$ENV{HOME}/src/mLibExternal" CACHE PATH "Root path of mLib library dependencies")
#set(MLIB_EXTERNAL_INCLUDE_DIR "${MLIB_EXTERNAL_ROOT}/include")
#include_directories(${MLIB_INCLUDE_DIR} ${MLIB_SOURCE_DIR} ${MLIB_EXTERNAL_INCLUDE_DIR})

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
	src/bh/mLib/mLib.cpp
	src/bh/mLib/mLibExtra.cpp
)

#file(GLOB HEADERS include/bh/*.h include/bh/*.hxx)
#set_target_properties(bh PROPERTIES
##		PUBLIC_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/config.h")
#		PUBLIC_HEADER "${HEADERS}">
#)
target_include_directories(
	bh
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include/>
)

install(TARGETS bh
		EXPORT libbhTargets
		RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
		LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
		PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}"
		COMPONENT dev)


export(TARGETS bh
		FILE "${PROJECT_BINARY_DIR}/libbhTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE libbh)

# Create the libbhConfig.cmake and libbhConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
		"${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
configure_file(libbhConfig.cmake.in
		"${PROJECT_BINARY_DIR}/libbhConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${LIBBH_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(libbhConfig.cmake.in
		"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libbhConfig.cmake" @ONLY)
# ... for both
configure_file(libbhConfigVersion.cmake.in
		"${PROJECT_BINARY_DIR}/libbhConfigVersion.cmake" @ONLY)

# Install the libbhConfig.cmake and libbhConfigVersion.cmake
install(FILES
		"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/libbhConfig.cmake"
		"${PROJECT_BINARY_DIR}/libbhConfigVersion.cmake"
		DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT libbhTargets DESTINATION
		"${INSTALL_CMAKE_DIR}" COMPONENT dev)

#install(TARGETS bh
#		EXPORT nibbhTargets
#		DESTINATION lib/)

install(DIRECTORY include/ DESTINATION include/
		FILES_MATCHING PATTERN "*.h")

#file(GLOB HEADERS include/bh/*.h include/bh/*.hxx)
#install(FILES ${HEADERS} DESTINATION include/bh/)

#file(GLOB HEADERS include/*.h include/*/*.h include/*/*/*.h)
#install(FILES ${HEADERS} DESTINATION include/)
