cmake_minimum_required(VERSION 3.0)
project(read_ply CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Boost COMPONENTS program_options)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(read_ply 
    main.cpp
    sim_io.cpp
)

target_link_libraries(read_ply
PUBLIC
    ${PCL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${GLUT_LIBRARIES}
    Boost::program_options
    pthread
)