cmake_minimum_required(VERSION 3.10)
project(Sampling-basedMP)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ompl REQUIRED)
add_subdirectory(libs/GeneticRRT)


add_executable(${PROJECT_NAME}  src/main.cpp)

target_link_libraries(${PROJECT_NAME}  ${OMPL_LIBRARIES} GeneticRRT)
target_include_directories(${PROJECT_NAME}  PRIVATE ${OMPL_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME}  PUBLIC  libs/GeneticRRT/inc)