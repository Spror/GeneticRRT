add_library(GeneticRRT src/GeneticRRT.cpp inc/GeneticRRT.hpp)
target_include_directories(GeneticRRT PUBLIC inc)
target_include_directories(GeneticRRT  PRIVATE ${OMPL_INCLUDE_DIRS})