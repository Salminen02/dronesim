cmake_minimum_required(VERSION 3.10)
project(MissileSimulation)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include directories
include_directories(include)
include_directories(/usr/local/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

# Source files
set(SOURCES
    src/main.cpp
    src/drone/controller.cpp
    src/drone/controller.hpp
    src/drone/hardDrone.cpp
    src/drone/hardDrone.hpp
    src/simulation/sim.cpp
    src/simulation/simThread.cpp
    src/raylibGraphics/raylibGraphics.cpp
    src/drone/controllerDevice.cpp
    src/drone/controllerDevice.hpp
       
)

# Add executable
add_executable(drone_sim ${SOURCES})

# Link libraries
target_link_libraries(drone_sim 
    Eigen3::Eigen
    /usr/local/lib/libraylib.a
    GL
    m
    pthread
    dl
    rt
    X11
)