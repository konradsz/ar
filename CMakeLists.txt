cmake_minimum_required(VERSION 3.6)

project(ar)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wall -Wextra -pedantic")

file(GLOB_RECURSE src_files ${PROJECT_SOURCE_DIR}/src/*)
add_executable(ar ${src_files})

find_package(OpenGL REQUIRED)
if (OpenGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIRS})
    target_link_libraries(ar ${OPENGL_LIBRARIES})
endif()

find_package(PkgConfig REQUIRED)
if (PkgConfig_FOUND)
    pkg_search_module(GLFW REQUIRED glfw3)
    if (GLFW_FOUND)
        include_directories(${GLFW_INCLUDE_DIRS})
        target_link_libraries(ar ${GLFW_LIBRARIES})
    endif()

    pkg_search_module(OPENCV REQUIRED opencv)
    if (OPENCV_FOUND)
        include_directories(${OPENCV_INCLUDE_DIRS})
        target_link_libraries(ar ${OPENCV_LIBRARIES})
    endif()
endif()

find_package(GLUT REQUIRED)
if (GLUT_FOUND)
    include_directories(${GLUT_INCLUDE_DIRS})
    target_link_libraries(ar ${GLUT_LIBRARY})
endif()

find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(ar ${GLEW_LIBRARIES})
endif()
