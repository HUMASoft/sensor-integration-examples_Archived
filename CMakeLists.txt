project(sensor-integration-examples)

cmake_minimum_required(VERSION 2.8)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(src SRC_LIST)
#aux_source_directory(tools SRC_LIST)


add_subdirectory(lib/sensor-integration)

#find_package(libplot)

INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})

foreach( testsourcefile ${SRC_LIST} )
    # I used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" testname ${testsourcefile} )
    string( REPLACE ".cpp" "" testname ${testname} )

    add_executable( ${testname} ${testsourcefile} )
    message ("Including executable: " ${testname})
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${SUBDIR_LINK_NAMES} )
endforeach( testsourcefile ${APP_SOURCES} )
