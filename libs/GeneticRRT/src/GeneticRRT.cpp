#include "GeneticRRT.hpp"

ompl::GeneticRRT::Chromosome::Chromosome(ompl::base::PlannerSolution genes) : genes_(genes)
{
    this->calculateFintess();
}

bool ompl::GeneticRRT::Chromosome::isValid() const
{
    return genes_.path_->as<ompl::geometric::PathGeometric>()->check();
}

/*Method to calculate fitness value*/
void ompl::GeneticRRT::Chromosome::calculateFintess()
{
    fitness_ = genes_.path_->as<ompl::geometric::PathGeometric>()->length();
}

void foo(int k)
{
    std::cout << "dzialam" << std::endl;
}

ompl::GeneticRRT::GeneticRRT(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "GeneticRRT"), RRTplanner_p{std::make_shared<ompl::geometric::RRT>(si)}
{
    specs_.approximateSolutions = true;

    Planner::declareParam<int>("population num", this, &GeneticRRT::setPopulation, &GeneticRRT::getPopulation, "1:1:5000");
    Planner::declareParam<int>("generation num", this, &GeneticRRT::setGeneration, &GeneticRRT::getGeneration, "1:1:5000");
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
    populationNumber_ = population;
}

int ompl::GeneticRRT::getPopulation() const
{
    return populationNumber_;
}


void ompl::GeneticRRT::setGeneration(int generation)
{
    generationNumber_ = generation;
}

int ompl::GeneticRRT::getGeneration() const
{
    return generationNumber_;
}

auto ompl::GeneticRRT::findBestChromosome(std::vector<ompl::GeneticRRT::Chromosome> &chromosome_v) const
{
    auto compareByFitness = [](const ompl::GeneticRRT::Chromosome c1, const ompl::GeneticRRT::Chromosome c2)
    {
        return c1.fitness_ < c2.fitness_;
    };

    auto bestChromosome = std::min_element(chromosome_v.begin(), chromosome_v.end(), compareByFitness);

    return bestChromosome;
}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::GA(Chromosome father, Chromosome mother)
{
    std::ofstream fileStream;
    fileStream.open("plik.txt");

    auto fatherPath = father.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    auto motherPath = mother.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    father.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
    mother.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);

    auto halfFatherSize = fatherPath.size();
    auto halfMotherSize = motherPath.size();

    auto minHalfSize = std::min(halfFatherSize, halfMotherSize);

    std::swap_ranges(fatherPath.begin()+1, fatherPath.begin() + minHalfSize, motherPath.end()-minHalfSize);

    std::vector< const base::State * > states;
    for(auto const &it: fatherPath)
    {
        states.push_back(it);
    }
    
    
    auto newPath(std::make_shared<ompl::geometric::PathGeometric>(si_));
    for (int i = states.size() - 1; i >= 0; --i)
        newPath->append(states[i]);

    if(newPath->check())
    {
       newPath->printAsMatrix(fileStream);
       std::cout << "dad " << newPath->length() << std::endl;
    }
  
  return father;

}

ompl::base::PlannerStatus ompl::GeneticRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    RRTplanner_p->setProblemDefinition(this->getProblemDefinition());
    ompl::base::PlannerStatus ss;

    std::ofstream fileStream;
    fileStream.open("plik.txt");

    for (auto i = 0; i < populationNumber_; i++)
    {
        ss = RRTplanner_p->solve(ptc);
        if (ss)
        {
            RRTplanner_p->clear();
        }
        else
        {
            std::cerr <<"Population generating failed" <<std::endl;
            pdef_->clearSolutionPaths();
            RRTplanner_p->clear();
            return {false, false};
        }
    }

    std::vector<ompl::GeneticRRT::Chromosome> candidates;

    for(auto &solution : pdef_->getSolutions())
    {
        candidates.push_back((solution));
    }

    for(auto i = 0; i < 1; i++)
    {
        auto Father = findBestChromosome(candidates);

        std::cout << (*Father).fitness_ << std::endl;

        std::mt19937 rng(std::time(nullptr));
        std::uniform_int_distribution<int> dist(0, candidates.size() - 1);
        auto randomIndex = 0;

        do
        {
            randomIndex = dist(rng);;
        }
        while ((Father == candidates.begin() + randomIndex));
        auto k = GA(*Father, candidates[randomIndex]);
   


    }

    auto k = findBestChromosome(candidates);
    std::cout << k->fitness_ << "dupa!" << std::endl;
    // for (auto &it : candidates)
    // {
    //     it.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
    //     fileStream << std::endl;
    //     std::cout << 1/it.path_->as<ompl::geometric::PathGeometric>()->smoothness() << std::endl;
    // }
    // std::cout << candidates.size();

    return ss;
}