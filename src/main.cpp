#include <iostream>
#include <fstream>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include "GeneticRRT.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{

    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    //  // extract the first component of the state and cast it to what we expect
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    //  // extract the second component of the state and cast it to what we expect
    // const auto *rot = se2state->as<ob::SO2StateSpace::StateSpace>(1);

    //  // check validity of state defined by pos & rot

    //  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return true;
}

void plan2()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);

     // create a random start state
     ob::ScopedState<> start(space);
     start.random();
  
     // create a random goal state
     ob::ScopedState<> goal(space);
     goal.random();
  

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(-4);
    bounds.setHigh(4);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state)
                               { return isStateValid(state); });

    // create a random start state
    ob::ScopedState<> start(space);
    start = (0, 0);

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal = (1, 3);

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRT>(ss.getSpaceInformation()));
    // this call is optional, but we put it in to get more output information
    ss.setup();
    // ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::ofstream fileStream;
        fileStream.open("plik.txt");
        if (!fileStream.is_open())
        {
            std::cerr << "File opening failed" << std::endl;
        }

        std::cout << "Found solution:" << std::endl;
        ss.getSolutionPath().printAsMatrix(fileStream);
        // print the path to screen
        ss.simplifySolution();
        // ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    foo(2);
    plan2();

    return 0;
}