project(sensor-integration-examples)

cmake_minimum_required(VERSION 3.12)



set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(src SRC_LIST)


find_package(YARP REQUIRED) #Yarp libraries included



add_subdirectory(lib/sensor-integration)
INCLUDE_DIRECTORIES(${SUBDIR_INCLUDE_DIRECTORIES})








#set(SUBDIR_LINK_NAMES ${SUBDIR_LINK_NAMES} ws2_32)



foreach( testsourcefile ${SRC_LIST} )
    # I used a simple string replace, to cut off .cpp.
    string( REGEX REPLACE "[A-z]+[/]" "" testname ${testsourcefile} )
    string( REPLACE ".cpp" "" testname ${testname} )

    add_executable( ${testname} ${testsourcefile} )
    message ("Including executable: " ${testname})
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${SUBDIR_LINK_NAMES} ${Boost_LIBRARIES})
    target_link_libraries(${testname} ${YARP_LIBRARIES}) #Yarp libraries included



endforeach( testsourcefile ${APP_SOURCES} )


##functions
function(echo_all_cmake_variable_values)
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction()






#echo_all_cmake_variable_values()
