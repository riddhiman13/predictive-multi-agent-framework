cmake_minimum_required(VERSION 3.0.2)
project(geometric_shapes)

add_compile_options(-std=c++11)

# Set compile options
set(PROJECT_COMPILE_OPTIONS
  -Wall
  -Wextra
  -Wwrite-strings
  -Wunreachable-code
  -Wpointer-arith
  -Wredundant-decls
  -Wno-unused-parameter
)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# used for finding Assimp and FCL
find_package(PkgConfig REQUIRED)

# TODO(henningkayser): Remove policy fix when assimp 5.1 is available
# Suppress policy warning in assimp (https://github.com/assimp/assimp/pull/2722)
set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
find_package(ASSIMP QUIET)
if(NOT ASSIMP_FOUND)
  # assimp is required, so REQUIRE the second attempt
  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
endif()

# find *absolute* paths to ASSIMP_LIBRARIES
# Both, pkg-config and assimp's cmake-config don't provide an absolute library path.
# For, pkg-config the path is in ASSIMP_PC_LIBRARY_DIRS, for cmake in ASSIMP_LIBRARY_DIRS.
find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

find_package(Boost REQUIRED system filesystem)

find_package(console_bridge REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(octomap REQUIRED)

pkg_check_modules(LIBFCL_PC REQUIRED fcl)
# find *absolute* paths to LIBFCL_* paths
find_path(LIBFCL_INCLUDE_DIRS fcl/config.h HINTS ${LIBFCL_PC_INCLUDE_DIR} ${LIBFCL_PC_INCLUDE_DIRS})
set(LIBFCL_LIBRARIES)
foreach(_lib ${LIBFCL_PC_LIBRARIES})
  find_library(_lib_${_lib} ${_lib} HINTS ${LIBFCL_PC_LIBRARY_DIRS})
  list(APPEND LIBFCL_LIBRARIES ${_lib_${_lib}})
endforeach()

find_package(catkin REQUIRED COMPONENTS
  eigen_stl_containers
  random_numbers
  resource_retriever
  shape_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
  CATKIN_DEPENDS
    eigen_stl_containers
    random_numbers
    shape_msgs
    visualization_msgs
  DEPENDS
    EIGEN3
    OCTOMAP
    console_bridge
  )

find_package(QHULL REQUIRED)

include_directories(include)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
  ${QHULL_INCLUDE_DIRS} ${LIBFCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/aabb.cpp
  src/bodies.cpp
  src/body_operations.cpp
  src/mesh_operations.cpp
  src/obb.cpp
  src/shape_extents.cpp
  src/shape_operations.cpp
  src/shape_to_marker.cpp
  src/shapes.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${PROJECT_NAME} ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${LIBFCL_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  # Unit tests
  add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
