#include "GeneticRRT.hpp"

void foo(int k)
{
    std::cout << "dzialam" << std::endl;
}

ompl::GeneticRRT::GeneticRRT(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "GeneticRRT"), RRTplanner_p{std::make_shared<ompl::geometric::RRT>(si)}
{
    specs_.approximateSolutions = true;

    Planner::declareParam<int>("population", this, &GeneticRRT::setPopulation, &GeneticRRT::getPopulation, "1:1:1000");
}

ompl::GeneticRRT::~GeneticRRT(void)
{
}

void ompl::GeneticRRT::getPlannerData(base::PlannerData &data) const
{
}

void ompl::GeneticRRT::clear(void)
{
}

void ompl::GeneticRRT::setup(void)
{
}

void ompl::GeneticRRT::setPopulation(int population)
{
    population_ = population;
}

int ompl::GeneticRRT::getPopulation() const
{
    return population_;
}

ompl::base::PlannerStatus ompl::GeneticRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    RRTplanner_p->setProblemDefinition(this->getProblemDefinition());
    ompl::base::PlannerStatus ss;

    for (auto i = 0; i < population_; i++)
    {
        ss = RRTplanner_p->solve(ptc);
        if (ss)
        {
            RRTplanner_p->clear();
        }
    }

    auto candidates = pdef_->getSolutions();

    // Random shuffle 
    auto rd = std::random_device{};
    auto rng = std::default_random_engine{rd()};
    std::shuffle(std::begin(candidates), std::end(candidates), rng);


    std::ofstream fileStream;
    fileStream.open("plik.txt");

    for (auto &it : candidates)
    {
        it.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
        fileStream << std::endl;
    }

    return ss;
}