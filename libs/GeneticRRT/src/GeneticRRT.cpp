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

ompl::GeneticRRT::GeneticRRT(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "GeneticRRT"), RRTplanner_p{std::make_shared<ompl::geometric::RRT>(si)}, rng(std::time(nullptr))
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

int ompl::GeneticRRT::select(std::vector<Chromosome> chromosome_v)
{
    std::uniform_int_distribution<int> dist(0, chromosome_v.size() - 1);
    std::map<double, int> members;

    for (auto i = 0; i < chromosome_v.size() * 0.05; i++)
    {
        auto randomnumber = dist(rng);
        members[chromosome_v[randomnumber].fitness_] = randomnumber;
    }

    return members.begin()->second;
}

void ompl::GeneticRRT::deleteDuplicates(std::vector<base::State *> &states)
{
    if (states.size() > 3)
    {
        for (auto i = 0; i < states.size(); ++i)
        {
            for (auto j = i + 1; j < states.size();)
            {
                if (si_->equalStates(states[i], states[j]))
                {
                    states.erase(states.begin() + j);
                }
                else
                {
                    ++j;
                }
            }
        }
    }
}

std::vector<ompl::base::State *> ompl::GeneticRRT::mutation(std::vector<ompl::base::State *> states)
{

    auto size1 = states.size();
    if (size1 > 3)
    {
        std::uniform_int_distribution<int> dist(1, states.size() - 2);
        auto random = dist(rng);
        states.erase(states.begin() + random);
        auto size2 = states.size();
        std::cout << size2 << std::endl;
    }

    return states;
}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::GA(Chromosome father, Chromosome mother)
{
    auto fatherPath = father.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    auto motherPath = mother.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();

    auto fatherSize = fatherPath.size();
    auto motherSize = motherPath.size();

    auto minHalfSize = std::min(fatherSize / 2, motherSize / 2);

    std::uniform_int_distribution<int> dist(1, minHalfSize * 2 - 1);
    auto randomPosition = dist(rng);

    std::swap_ranges(fatherPath.end() - randomPosition, fatherPath.end(), motherPath.end() - randomPosition);

    std::vector<base::State *> statesFather, statesMother, tempVec;

    for (auto const &it : fatherPath)
    {
        statesFather.push_back(it);
    }
    for (auto const &it : motherPath)
    {
        statesMother.push_back(it);
    }

    std::reverse(statesFather.begin(), statesFather.end());
    std::reverse(statesMother.begin(), statesMother.end());

    deleteDuplicates(statesFather);
    deleteDuplicates(statesMother);

    std::uniform_int_distribution<int> dist_2(0, 20);
    auto randomNumber = dist_2(rng);

    if (randomNumber == 1)
    {
        statesFather = mutation(statesFather);
        statesMother = mutation(statesMother);
    }

    auto newPathFather(std::make_shared<ompl::geometric::PathGeometric>(si_));
    auto newPathMother(std::make_shared<ompl::geometric::PathGeometric>(si_));

    for (int i = statesFather.size() - 1; i >= 0; --i)
        newPathFather->append(statesFather[i]);

    for (int i = statesMother.size() - 1; i >= 0; --i)
        newPathMother->append(statesMother[i]);

    if (newPathFather->check() && newPathMother->check())
    {
        if (newPathFather->length() > newPathMother->length())
        {
            Chromosome child{{newPathFather}};
            return child;
        }
        else
        {
            Chromosome child{{newPathMother}};
            return child;
        }
    }
    else if (newPathFather->check())
    {
        Chromosome child{{newPathFather}};
        return child;
    }
    else if (newPathMother->check())
    {
        Chromosome child{{newPathMother}};
        return child;
    }
    else
    {
        return {{newPathFather}};
    }
}

ompl::base::PlannerStatus ompl::GeneticRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    RRTplanner_p->setProblemDefinition(this->getProblemDefinition());
    ompl::base::PlannerStatus ss;
    RRTplanner_p->setRange(30.0);
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
            std::cerr << "Population generating failed" << std::endl;
            // pdef_->clearSolutionPaths();
            RRTplanner_p->clear();
            i--;

            // return {false, false};
        }
    }

    std::vector<ompl::GeneticRRT::Chromosome> population;

    for (auto &solution : pdef_->getSolutions())
    {
        population.push_back((solution));
    }

    auto p = *findBestChromosome(population);

    p.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);

    for (auto i = 0; i < generationNumber_;)
    {
        auto father = select(population);
        auto mother = select(population);

        if (father != mother)
        {
            auto child = GA(population[father], population[mother]);

            if (child.isValid())
            {
                if (child.fitness_ < population[mother].fitness_)
                {
                    population[mother] = child;
                }

                i++;
                std::cout << child.fitness_ << std::endl;
            }
        }
    }

    auto k = findBestChromosome(population);

    k->calculateFintess();
    std::cout << k->fitness_ << "<- After // Before ->!" << p.fitness_ << std::endl;

    k->genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);

    pdef_->clearSolutionPaths();
    pdef_->addSolutionPath((*k).genes_);

    return ss;
}