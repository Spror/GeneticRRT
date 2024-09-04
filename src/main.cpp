#include <iostream>
#include <fstream>


#include "SE2.hpp"
#include "SE3.hpp"

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include "GeneticRRT.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;


int main(int, char **)
{
    try
    {
      planSE2_3(300, 10000, 0.1);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
   
    return 0;
}