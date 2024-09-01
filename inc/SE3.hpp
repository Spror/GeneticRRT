#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <array>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include "GeneticRRT.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;


struct Building {
    double x, y, z, dx, dy, dz;
};


/**
 * @brief 
 * Creating the planning problem in SE3 -- without obstacles, 
 * start point: 10, 10, 10 ,0, 0, 0
 * end point: 400, 400, 400, 0, 0, 0 
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE3_1(int population, int generations, int p);

/**
 * @brief 
 * Creating the planning problem in SE3 -- with one obstacle (sphere), 
 * start point: 10, 10, 10 ,0, 0, 0
 * end point: 400, 400, 400, 0, 0, 0 
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE3_2(int population, int generations, int p);

/**
 * @brief 
 * Creating the planning problem in SE3 -- city simulation (multiple cuboids), 
 * start point: 10, 10, 10 ,0, 0, 0
 * end point: 490, 490, 100, 0, 0, 0 
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE3_3(int population, int generations, double p);

bool isStateValidSE3_1(const ob::State *state);
bool isStateValidSE3_2(const ob::State *state);
bool isStateValidSE3_3(const ob::State *state);

/**
 * @brief check if SE3 state is inside building 
 * 
 * @param x  
 * @param y 
 * @param z 
 * @param building - quboid to check
 * @return true - if state in building
 * @return false - otherwise
 */
bool isInsideBuilding(double x, double y, double z, const Building &building);




