cmake_minimum_required(VERSION 3.12)

####################################

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
IF (MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
ELSE ()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g")
ENDIF ()

####################################

project(am)

add_definitions(-DPATH_PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# eigen
FIND_PACKAGE(Eigen3 REQUIRED)

##################################

include_directories(
)

add_executable(${PROJECT_NAME}
    main.cpp
    ../adaptive_mesh.h
    )

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    )