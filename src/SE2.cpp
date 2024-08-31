#include "SE2.hpp"

std::vector<Rectangle> rectangles = {
    {0, 50, 200, 20},
    {200, 50, 20, 200},
    {250, 250, 50, 50},
    {400, 100, 80, 60},
    {250, 150, 40, 70}};

std::vector<Circle> circles = {
    {120, 200, 50}};

std::vector<Triangle> triangles = {
    {{{400, 400}, {350, 450}, {500, 500}}}};

std::vector<Polygon> polygons = {
    {{{50, 400}, {350, 320}, {320, 360}, {280, 350}, {200, 490}}}};

bool pointInRectangle(double x, double y, const Rectangle &rect)
{
    return x >= rect.x && x <= rect.x + rect.width &&
           y >= rect.y && y <= rect.y + rect.height;
}

bool pointInCircle(double x, double y, const Circle &circle)
{
    double distance = std::sqrt(std::pow(x - circle.x, 2) +
                                std::pow(y - circle.y, 2));
    return distance <= circle.radius;
}

double sign(double x1, double y1, double x2, double y2, double x3, double y3)
{
    return (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
}

bool pointInTriangle(double x, double y, const Triangle &triangle)
{
    double d1 = sign(x, y, triangle.vertices[0].first, triangle.vertices[0].second, triangle.vertices[1].first, triangle.vertices[1].second);
    double d2 = sign(x, y, triangle.vertices[1].first, triangle.vertices[1].second, triangle.vertices[2].first, triangle.vertices[2].second);
    double d3 = sign(x, y, triangle.vertices[2].first, triangle.vertices[2].second, triangle.vertices[0].first, triangle.vertices[0].second);

    bool has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

    return !(has_neg && has_pos);
}

bool pointInPolygon(double x, double y, const Polygon &polygon)
{
    int i, j, nvert = polygon.vertices.size();
    bool c = false;

    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((polygon.vertices[i].second >= y) != (polygon.vertices[j].second >= y)) &&
            (x <= (polygon.vertices[j].first - polygon.vertices[i].first) * (y - polygon.vertices[i].second) / (polygon.vertices[j].second - polygon.vertices[i].second) + polygon.vertices[i].first))
        {
            c = !c;
        }
    }

    return c;
}

bool isStateValidSE2_2(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE2StateSpace::StateType>();
    const auto x = se3state->getX();
    const auto y = se3state->getY();

    // Define the center and radius of the sphere
    const auto centerX = 250.0;
    const auto centerY = 250.0;
    const auto radius = 150.0;

    // Calculate the distance from the state to the center of the sphere
    const auto distance = std::sqrt(std::pow(x - centerX, 2) +
                                    std::pow(y - centerY, 2));

    // Check if the state is inside the sphere
    if (distance < radius + 1)
    {
        return false;
    }

    // If not inside the sphere, the state is valid
    return true;
}

bool isStateValidSE2_1(const ob::State *state)
{

    return true;
}

bool isStateValidSE2_3(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE2StateSpace::StateType>();
    const auto x = se3state->getX();
    const auto y = se3state->getY();

    // Check for collision with rectangles
    for (const auto &rect : rectangles)
    {
        if (pointInRectangle(x, y, rect))
        {
            return false;
        }
    }

    // Check for collision with circles
    for (const auto &circle : circles)
    {
        if (pointInCircle(x, y, circle))
        {
            return false;
        }
    }

    // Check for collision with triangles
    for (const auto &triangle : triangles)
    {
        if (pointInTriangle(x, y, triangle))
        {
            return false;
        }
    }

    // Check for collision with polygons
    for (const auto &polygon : polygons)
    {
        if (pointInPolygon(x, y, polygon))
        {
            return false;
        }
    }

    // If not inside any shape, the state is valid
    return true;
}

void planSE2_1(int population, int generations, int p)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE2_1);
    si->setStateValidityCheckingResolution(0.01); 
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(10);
    start->as<ob::SE2StateSpace::StateType>()->setY(10);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(450);
    goal->as<ob::SE2StateSpace::StateType>()->setY(440);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    planner->setP(p);
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

void planSE2_2(int population, int generations, int p)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE2_2);
    si->setStateValidityCheckingResolution(0.001); // 3%
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(10);
    start->as<ob::SE2StateSpace::StateType>()->setY(10);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(400);
    goal->as<ob::SE2StateSpace::StateType>()->setY(400);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    planner->setP(p);
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

void planSE2_3(int population, int generations, int p)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(500);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
    si->setStateValidityChecker(isStateValidSE2_3);
    si->setStateValidityCheckingResolution(0.001); 
    si->setup();

    // create a random start state
    ob::ScopedState<> start(space);
    start->as<ob::SE2StateSpace::StateType>()->setX(10);
    start->as<ob::SE2StateSpace::StateType>()->setY(10);
    start->as<ob::SE2StateSpace::StateType>()->setYaw(0);
    // create a random goal state
    ob::ScopedState<> goal(space);
    goal->as<ob::SE2StateSpace::StateType>()->setX(100);
    goal->as<ob::SE2StateSpace::StateType>()->setY(100);
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(0);

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::GeneticRRT>(si));
    planner->setPopulation(population);
    planner->setGeneration(generations);
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();
    planner->setP(p);

    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(80.0);

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

