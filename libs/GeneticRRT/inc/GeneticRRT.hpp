#pragma once

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
 
namespace ompl
{
    class myNewPlanner : public geometric::RRT
    {

    };

}

void foo(int k);