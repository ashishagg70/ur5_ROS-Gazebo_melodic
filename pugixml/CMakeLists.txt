cmake_minimum_required(VERSION 2.8.3)

project(pugixml)



################################################################################
# user build settings

#ROS USAGE
SET(USE_ROS True) # Set False if you want to build this package without Catkin

################################################################################



# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    #set(CMAKE_CXX_FLAGS "-std=c++11")
    add_definitions(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
    #set(CMAKE_CXX_FLAGS "-std=c++0x")
    add_definitions(-std=c++0x)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()



# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries




	
set(PUGIXML_LIBRARY_SOURCE_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/src/sources
)

set(PUGIXML_LIBRARY_SOURCE_FILES
	${PUGIXML_LIBRARY_SOURCE_DIR}/pugixml.cpp
)
	
set(PUGIXML_LIBRARY_HEADER_DIR
	${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

set(PUGIXML_LIBRARY_HEADER_FILES
	${PUGIXML_LIBRARY_HEADER_DIR}/pugixml/pugiconfig.hpp
	${PUGIXML_LIBRARY_HEADER_DIR}/pugixml/pugixml.hpp
)


if(USE_ROS)
  find_package(catkin REQUIRED)

  catkin_package(
      INCLUDE_DIRS ${PUGIXML_LIBRARY_HEADER_DIR}
      LIBRARIES ${PROJECT_NAME}
    )
endif()


include_directories(${PUGIXML_LIBRARY_HEADER_DIR} ${PUGIXML_LIBRARY_HEADER_DIR}/pugixml)

if(USE_ROS)
  include_directories(${catkin_INCLUDE_DIRS})
endif()


add_library(${PROJECT_NAME} ${PUGIXML_LIBRARY_SOURCE_FILES} ${PUGIXML_LIBRARY_HEADER_FILES})

if(USE_ROS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif()




################################################################################
# Create the pugixmlConfig.cmake file for other cmake projects.
IF(NOT USE_ROS)

  	set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib)
	  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

    SET(pugixml_LIBRARY_DIR ${LIBRARY_OUTPUT_PATH} )
    #Library with the full path
    SET(pugixml_LIBRARY "${LIBRARY_OUTPUT_PATH}/lib${PROJECT_NAME}.a" )

    SET(pugixml_INCLUDE_DIR
      ${PUGIXML_LIBRARY_HEADER_DIR}
    )

    CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake @ONLY IMMEDIATE )
    export( PACKAGE ${PROJECT_NAME} )

#TODO
#    INSTALL(DIRECTORY include/pugixml DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
#    INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
