cmake_minimum_required(VERSION 3.10)

project(rl-tools-example)
add_subdirectory(external/rl_tools)

set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
include(FetchContent)

add_executable(my_pendulum src/main.cpp)
target_link_libraries(my_pendulum PRIVATE RLtools::RLtools)

# The following target disables evaluations and checkpointing during training to assess the training time
add_executable(my_pendulum_benchmark src/main.cpp)
target_compile_definitions(my_pendulum_benchmark PRIVATE BENCHMARK)
target_link_libraries(my_pendulum_benchmark PRIVATE RLtools::RLtools)

# Fixed-wing velocity tracking example
add_executable(fixedwing_training src/fixedwing_training.cpp)
target_link_libraries(fixedwing_training PRIVATE RLtools::RLtools)

# Benchmark version for fixed-wing
add_executable(fixedwing_training_benchmark src/fixedwing_training.cpp)
target_compile_definitions(fixedwing_training_benchmark PRIVATE BENCHMARK)
target_link_libraries(fixedwing_training_benchmark PRIVATE RLtools::RLtools)

# Dynamics test for fixed-wing (zero control test)
add_executable(fixedwing_dynamics_test src/fixedwing_dynamics_test.cpp)
target_link_libraries(fixedwing_dynamics_test PRIVATE RLtools::RLtools)



if(NOT MSVC AND CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(my_pendulum PRIVATE -Ofast)
    target_compile_options(fixedwing_training PRIVATE -Ofast)
    if(NOT APPLE)
        target_compile_options(my_pendulum PRIVATE -march=native)
        target_compile_options(fixedwing_training PRIVATE -march=native)
    endif()
endif()