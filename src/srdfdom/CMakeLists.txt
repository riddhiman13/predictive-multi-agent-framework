cmake_minimum_required(VERSION 3.0.2)
project(srdfdom)

find_package(Boost REQUIRED)

find_package(console_bridge REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(catkin REQUIRED COMPONENTS cmake_modules urdf)

find_package(TinyXML2 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${TinyXML2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  )

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS TinyXML2 console_bridge urdfdom_headers
)

add_library(${PROJECT_NAME}
  src/model.cpp
  src/srdf_writer.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_headers_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS
  scripts/display_srdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/srdf_parser.test)

  add_definitions(-DTEST_RESOURCE_LOCATION="${CMAKE_SOURCE_DIR}/test/resources")
  execute_process(COMMAND bash -c "locale -a | grep -q ^nl_NL"
    RESULT_VARIABLE TEST_LOCALE
    OUTPUT_QUIET ERROR_QUIET)
  if (TEST_LOCALE)
    message(STATUS "Locale nl_NL not available. Locale test will not be meaningful.")
  endif()

  add_rostest_gtest(test_cpp test/srdf_parser_cpp.test test/test_parser.cpp)
  target_link_libraries(test_cpp ${catkin_LIBRARIES} ${PROJECT_NAME})
endif()
