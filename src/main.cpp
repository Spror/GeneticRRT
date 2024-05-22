#include <iostream>
#include <fstream>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/SimpleSetup.h>

#include "GeneticRRT.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;



bool isStateValid(const ob::State *state)
{

    // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto x = se2state->getX();
    const auto y = se2state->getY();

    if( ((x >= 50.0 && x <= 100.0) &&  (y >= 50 && y <= 100)) )
    {
        return false;
    }
    else if( ((x >= 150.0 && x <= 400.0) &&  (y >= 150 && y <= 200)) )
    {
        return false;
    }
    else if( ((x >= 250.0 && x <= 300.0) &&  (y >= 250 && y <= 300)) )
    {
        return false;
    }
    else if( ((x >= 100.0 && x <= 150.0) &&  (y >= 100 && y <= 400)) )
    {
        return false;
    }
    else
    {
        return true;
    }

}

void plan2()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(502);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValid);
    si->setStateValidityCheckingResolution(0.001); // 3%
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(0);
    start->as<ob::SE2StateSpace::StateType>()->setY(0);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(350);
    goal->as<ob::SE2StateSpace::StateType>()->setY(250);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(1000);
    planner->setGeneration(100000);
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
   

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(8.0);

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

    double measure = si->getSpaceMeasure();

    // Output the space measure
    std::cout << "Space measure: " << measure << std::endl;
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(502);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state)
                               { return isStateValid(state); });

                                
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.001);
    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(0);
    start->as<ob::SE2StateSpace::StateType>()->setY(0);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(350);
    goal->as<ob::SE2StateSpace::StateType>()->setY(250);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    // this call is optional, but we put it in to get more output information
    ss.setup();
    // ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::ofstream fileStream;
        fileStream.open("plik2.txt");
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
   // foo(2);
    plan2();
    plan();

    return 0;
}