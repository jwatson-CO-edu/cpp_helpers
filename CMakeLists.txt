# ~ Project Setup ~
cmake_minimum_required( VERSION 3.0 )
project( cpp_helpers )

# ~ Build Settings ~
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") # This will prevent g++ from getting upset about default values
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/lib" )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin" )
message( "Source Dir: __  ${PROJECT_SOURCE_DIR}" )
message( "Static Lib Dir: ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}" )
message( "Exef Dir: _____ ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}" )

# ~ Source Dirs ~
set( CPP_HELP_CPP basic/Cpp_Helpers.cpp )
set( CPP_HELP_HPP basic/Cpp_Helpers.hpp )

set( CNTNR_CPP containers/Container_Helpers.cpp )
set( CNTNR_HPP containers/Container_Helpers.hpp )

# ~~~ LIBS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

set( TARGET cpp_helpers )
add_library( ${TARGET} STATIC ${CPP_HELP_CPP} ${CPP_HELP_HPP} )
target_include_directories( ${TARGET} PUBLIC ${PROJECT_SOURCE_DIR}/basic )
set_target_properties( ${TARGET} PROPERTIES PUBLIC_HEADER ${CPP_HELP_HPP} )

# set( TARGET container_helpers )
# add_library( ${TARGET} STATIC ${CNTNR_CPP} ${CNTNR_HPP} )
# target_include_directories( ${TARGET} PUBLIC ${PROJECT_SOURCE_DIR}/containers ${PROJECT_SOURCE_DIR}/basic )
# set_target_properties( ${TARGET} PROPERTIES PUBLIC_HEADER ${CNTNR_HPP} )


# ~~~ TESTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

set( TARGET basic_test )
add_executable( ${TARGET} test/${TARGET}.cpp )
target_link_libraries( ${TARGET} cpp_helpers )
