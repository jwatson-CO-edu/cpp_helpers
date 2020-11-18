# ~ Project Setup ~
cmake_minimum_required( VERSION 3.0 )
project( cpp_helpers )

# ~ Build Settings ~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") # This will prevent g++ from getting upset about default values
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib" )
message( "Source Dir: __  ${PROJECT_SOURCE_DIR}" )
message( "Static Lib Dir: ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}" )

# ~ Source Dirs ~
# add_subdirectory( basic )
include_directories( basic )
set( CPP_HELP_CPP basic/Cpp_Helpers.cpp )
set( CPP_HELP_HPP basic/Cpp_Helpers.hpp )

# add_subdirectory( containers )

# ~~~ LIBS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

add_library( cpp_helpers STATIC ${CPP_HELP_CPP} ${CPP_HELP_HPP} )
message( "Sent target to ${ARCHIVE_OUTPUT_DIRECTORY}" )