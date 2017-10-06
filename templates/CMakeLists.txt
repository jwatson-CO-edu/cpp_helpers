# TOPLEVEL CMAKELISTS

# ~ Project Setup ~
cmake_minimum_required( VERSION 2.6 )
project( PROJECTNAME )

# ~ Build Settings ~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") # This will prevent g++ from getting upset about default values

# ~ Source Dirs ~
add_subdirectory( src ) # Or where you keep all the source files
