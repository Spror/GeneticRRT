#include "SE3.hpp"

std::vector<Building> buildings = {
    {150, 250, 0, 40, 40, 200},
    {80, 20, 0, 40, 40, 250},
    {20, 80, 0, 40, 40, 350},
    {250, 250, 0, 60, 60, 400},
    {100, 100, 0, 30, 30, 150},
    {150, 150, 0, 50, 50, 300},
    {300, 100, 0, 40, 40, 200},
    {350, 200, 0, 50, 50, 250},
    {200, 350, 0, 70, 70, 100},
    {400, 100, 0, 50, 50, 150},
    {50, 400, 0, 50, 50, 200},
    {100, 300, 0, 40, 40, 250},
    {250, 100, 0, 60, 60, 300},
    {150, 350, 0, 70, 70, 350},
    {50, 50, 0, 30, 30, 120},
    {300, 300, 0, 50, 50, 250},
    {350, 50, 0, 40, 40, 180},
    {150, 50, 0, 50, 50, 220},
    {400, 400, 0, 60, 60, 270}
    };

bool isInsideBuilding(double x, double y, double z, const Building &building)
{
    return (x >= building.x && x <= building.x + building.dx &&
           y >= building.y && y <= building.y + building.dy &&
           z >= building.z && z <= building.z + building.dz);
         
}

void planSE3_1(int population, int generations, int p)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE3_1);
    si->setStateValidityCheckingResolution(0.01); 
    si->setup();

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(10.0, 10.0, 10.0); 
    start->rotation().setIdentity();

 
    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(400.0, 400.0, 400.0); 
    goal->rotation().setIdentity();   

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    planner->setProbability(p);
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
        std::ofstream fileStream;
        fileStream.open("file.txt");
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        path.get()->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
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

void planSE3_2(int population, int generations, int p)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE3_2);
    si->setStateValidityCheckingResolution(0.01); 
    si->setup();

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(10.0, 10.0, 10.0); 
    start->rotation().setIdentity(); 


    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(400.0, 400.0, 400.0); 
    goal->rotation().setIdentity();   

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    planner->setProbability(p);

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
        std::ofstream fileStream;
        fileStream.open("file.txt");
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        path.get()->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
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

void planSE3_3(int population, int generations, double p)
{

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE3_3);
    si->setStateValidityCheckingResolution(0.01); // 3%
    si->setup();

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(10.0, 10.0, 10.0);
    start->rotation().setIdentity(); 

    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(490.0, 490.0, 100.0);
    goal->rotation().setIdentity(); 

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    planner->setProbability(p);
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(80.0);

    if (solved)
    {
        std::ofstream fileStream;
        fileStream.open("file.txt");
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        path.get()->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
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

bool isStateValidSE3_1(const ob::State *state)
{
    return true;
}

bool isStateValidSE3_2(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto x = se3state->getX();
    const auto y = se3state->getY();
    const auto z = se3state->getZ();

    // Define the center and radius of the sphere
    const auto centerX = 250.0;
    const auto centerY = 250.0;
    const auto centerZ = 250.0;
    const auto radius = 150.0;

    // Calculate the distance from the state to the center of the sphere
    const auto distance = std::sqrt(std::pow(x - centerX, 2) +
                                    std::pow(y - centerY, 2) +
                                    std::pow(z - centerZ, 2));

    // Check if the state is inside the sphere
    if (distance <= radius)
    {
        return false;
    }

    // If not inside the sphere, the state is valid
    return true;
}

bool isStateValidSE3_3(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto x = se3state->getX();
    const auto y = se3state->getY();
    const auto z = se3state->getZ();

    for (const auto &building : buildings)
    {
        if (isInsideBuilding(x, y, z, building))
        {
            return false;
        }
    }
    return true;
}