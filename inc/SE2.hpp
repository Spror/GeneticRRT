#pragma once

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

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

/**
 * @brief 
 * Creating the planning problem in SE2 -- without obstacles, 
 * start point: 10, 10, 0 
 * end point: 400, 400, 0
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE2_1(int population, int generations, int p);

/**
 * @brief 
 * Creating the planning problem in SE2 -- with one obstacle, 
 * start point: 10, 10, 0 
 * end point: 400, 400, 0
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE2_2(int population, int generations, int p);

/**
 * @brief 
 * Creating the planning problem in SE2 -- maze (multiple obstacles), 
 * start point: 10, 10, 0 
 * end point: 100, 100, 0
 * area size (0,500)
 * 
 * @param population - number of population used in GA
 * @param generations - number of generations used in GA
 * @param p - mutation chance
 */
void planSE2_3(int population, int generations, int p);

bool isStateValidSE2_1(const ob::State *state);
bool isStateValidSE2_2(const ob::State *state);
bool isStateValidSE2_3(const ob::State *state);


struct Rectangle {
    double x, y, width, height;
};

struct Circle {
    double x, y, radius;
};

struct Triangle {
    std::vector<std::pair<double, double>> vertices;
};

struct Polygon {
    std::vector<std::pair<double, double>> vertices;
};


bool pointInRectangle(double x, double y, const Rectangle &rect);
bool pointInCircle(double x, double y, const Circle &circle);
double sign(double x1, double y1, double x2, double y2, double x3, double y3);
bool pointInTriangle(double x, double y, const Triangle &triangle);
bool pointInPolygon(double x, double y, const Polygon &polygon);