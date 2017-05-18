cmake_minimum_required(VERSION 2.8.3)
project(ascension)

find_package(catkin REQUIRED  COMPONENTS roscpp tf geometry_msgs)


# build ACT3DG library

message("This package builds a library from GitHub on the fly. Might require re-running catkin build a couple of times.")

include(ExternalProject)

ExternalProject_Add(act3dg
    GIT_REPOSITORY https://github.com/marcoesposito1988/ATC3DGTracker.git
    PREFIX "${CMAKE_CURRENT_BINARY_DIR}"
    # Disable install step
    INSTALL_COMMAND ""
)

ExternalProject_Get_Property(act3dg source_dir)
set(ATC3DG_INCLUDE_DIRS ${source_dir}/include)

ExternalProject_Get_Property(act3dg binary_dir)
set(ATC3DG_LIBRARY_DIRS ${binary_dir}/lib/)

# find_library(ATC3DG_LIBRARY atclib ${ATC3DG_LIBRARY_DIRS})
set(ATC3DG_LIBRARY "${ATC3DG_LIBRARY_DIRS}/libatclib.so")

catkin_package(
    CATKIN_DEPENDS roscpp tf geometry_msgs 
)

###########
## Build ##
###########

include_directories(${ATC3DG_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/ascension_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${ATC3DG_LIBRARY}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ascension.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
