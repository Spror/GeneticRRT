#include "GeneticRRT.hpp"

ompl::GeneticRRT::Chromosome::Chromosome(ompl::base::PlannerSolution genes) : genes_(genes)
{
    this->calculateFintess();
}

bool ompl::GeneticRRT::Chromosome::isValid() const
{
    return genes_.path_->as<ompl::geometric::PathGeometric>()->check();
}


void ompl::GeneticRRT::Chromosome::calculateFintess()
{
    fitness_ = genes_.path_->as<ompl::geometric::PathGeometric>()->length();
}

bool ompl::GeneticRRT::Chromosome::operator ==( const Chromosome & c )
{
    if(this->genes_ == c.genes_)
    {
        return true;
    }

    else
        return false;

}

ompl::GeneticRRT::GeneticRRT(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "GeneticRRT"),
    rng(std::time(nullptr))
{
    specs_.approximateSolutions = true;

    Planner::declareParam<int>("population num", this, &GeneticRRT::setPopulation, &GeneticRRT::getPopulation, "1:1:5000");
    Planner::declareParam<int>("generation num", this, &GeneticRRT::setGeneration, &GeneticRRT::getGeneration, "1:1:5000");
    Planner::declareParam<double>("probability", this, &GeneticRRT::setProbability, &GeneticRRT::getProbability, "0.0:0.01:1.0");
}

ompl::GeneticRRT::~GeneticRRT(void)
{
}

void ompl::GeneticRRT::getPlannerData(base::PlannerData &data) const
{
}

void ompl::GeneticRRT::clear(void)
{
    Planner::clear();
}

void ompl::GeneticRRT::setPopulation(int population)
{
    populationNumber_ = population;
}

int ompl::GeneticRRT::getPopulation() const
{
    return populationNumber_;
}

void ompl::GeneticRRT::setProbability(double probability)
{
    probability_=probability;
}
       
double  ompl::GeneticRRT::getProbability() const
{
    return probability_;
}

int64_t ompl::GeneticRRT::getDuration() const
{
    return duration;
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

ompl::GeneticRRT::Chromosome& ompl::GeneticRRT::select(std::vector<Chromosome> &chromosome_v)
{
    assert(!chromosome_v.empty() && "chromosome_v must not be empty");

    std::uniform_int_distribution<int> dist(0, chromosome_v.size() - 1);
    auto tournamentSize = std::max(1, static_cast<int>(chromosome_v.size() * 0.05));

    int winnerIndex = dist(rng);

    for (auto i = 1; i < tournamentSize; i++)
    {
        auto randomIndex = dist(rng);
        if (chromosome_v[randomIndex].fitness_ < chromosome_v[winnerIndex].fitness_)
        {
            winnerIndex = randomIndex;
        }
    }

    return chromosome_v[winnerIndex];
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

void ompl::GeneticRRT::mutationDeleteState(std::vector<ompl::base::State *> &states)
{
    auto size = states.size();
    if (size > 3)
    {
        std::uniform_int_distribution<int> dist(1, size - 2);
        auto random = dist(rng);
        states.erase(states.begin() + random);
    }
}

void ompl::GeneticRRT::mutationChangeState(std::vector<ompl::base::State *> &states)
{
    std::uniform_int_distribution<int> dist(1, states.size() - 2);
    auto randomPos = dist(rng);
 
    auto sampler = si_->allocStateSampler();
    base::State * newState = si_->allocState();
    sampler->sampleUniformNear(newState, states[randomPos], 50.0);

    states[randomPos] = newState;
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

   std::uniform_real_distribution<> uni_dist;
   

    if (uni_dist(rng) < probability_)
    {
        mutationDeleteState(statesFather);
        mutationDeleteState(statesMother);
    }


    if (uni_dist(rng) < probability_)
    {
        mutationChangeState(statesFather);
        mutationChangeState(statesMother);
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


bool ompl::GeneticRRT::generatePopulation(const base::PlannerTerminationCondition &ptc, std::vector<ompl::GeneticRRT::Chromosome>& population)
{
    std::shared_ptr<geometric::RRT> RRTplanner_p{std::make_shared<ompl::geometric::RRT>(this->getSpaceInformation())};

    RRTplanner_p->setProblemDefinition(this->getProblemDefinition());
    ompl::base::PlannerStatus ss;

    for (auto i = 0; i < populationNumber_; i++)
    {
        ss = RRTplanner_p->solve(ptc);
        if (ss)
        {
             RRTplanner_p->clear();
        }
        else
        {

            RRTplanner_p->clear();
            return false;
        }
    }

    for (auto &solution : pdef_->getSolutions())
    {
        population.push_back((solution));
    }

    return true;

}

ompl::base::PlannerStatus ompl::GeneticRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    assert(populationNumber_ > 1 && "populationNumber_ should be greater than one");

    std::chrono::steady_clock::time_point start, end;
    start = std::chrono::steady_clock::now();

    std::vector<ompl::GeneticRRT::Chromosome> population;

    if(!generatePopulation(ptc, population))
    {
        std::cerr << "Population generating failed" << std::endl;
        pdef_->clearSolutionPaths();
        clear();
        return {false, false};
    }

    auto bestResult = *findBestChromosome(population);

    for (auto i = 0; i < generationNumber_;)
    {
        auto &father = select(population);
        auto &mother = select(population);

        if (!(father == mother))
        {
            auto child = GA(father, mother);

            if (child.isValid())
            {
                if (child.fitness_ <= mother.fitness_)
                {
                    mother = child;
                }

                i++;
                std::cout << child.fitness_ << std::endl;
            }

            auto bestInPopulation = (*findBestChromosome(population));

            if(bestResult.fitness_ > bestInPopulation.fitness_)
            {
                bestResult = bestInPopulation;
            }
        }
    }
        
    pdef_->clearSolutionPaths();
    pdef_->addSolutionPath(bestResult.genes_);

    end = std::chrono::steady_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    return {true, true};
}