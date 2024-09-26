#include "GeneticRRT.hpp"

ompl::GeneticRRT::Chromosome::Chromosome(const ompl::base::PlannerSolution genes) : genes_(genes)
{
    if (!genes_.path_)
    {
        throw std::invalid_argument("Path in PlannerSolution cannot be null");
    }

    this->calculateFitness();
}

bool ompl::GeneticRRT::Chromosome::isValid() const
{
    return genes_.path_->as<ompl::geometric::PathGeometric>()->check();
}

void ompl::GeneticRRT::Chromosome::calculateFitness()
{
    fitness_ = genes_.path_->as<ompl::geometric::PathGeometric>()->length();
}

bool ompl::GeneticRRT::Chromosome::operator==(const Chromosome &c) const
{
    return this->genes_ == c.genes_;
}

ompl::GeneticRRT::GeneticRRT(const base::SpaceInformationPtr &si) : ompl::base::Planner(si, "GeneticRRT"),
                                                                    rng_(std::time(nullptr))
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
    if (population <= 1)
    {
        throw std::invalid_argument("Population size must be greater than 1.");
    }
    populationNumber_ = population;
}

int ompl::GeneticRRT::getPopulation() const
{
    return populationNumber_;
}

void ompl::GeneticRRT::setProbability(double probability)
{
    if (probability < 0.0 || probability > 1.0)
    {
        throw std::out_of_range("Mutation probability must be between 0.0 and 1.0.");
    }
    probability_ = probability;
}

double ompl::GeneticRRT::getProbability() const
{
    return probability_;
}

int64_t ompl::GeneticRRT::getDuration() const
{
    return duration_;
}

void ompl::GeneticRRT::setGeneration(int generation)
{
    if (generation < 1)
    {
        throw std::invalid_argument("Generation number must be at least 1.");
    }
    generationNumber_ = generation;
}

int ompl::GeneticRRT::getGeneration() const
{
    return generationNumber_;
}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::findBestChromosome(const std::vector<ompl::GeneticRRT::Chromosome> &chromosome_v) const
{
    auto compareByFitness = [](const ompl::GeneticRRT::Chromosome c1, const ompl::GeneticRRT::Chromosome c2)
    {
        return c1.fitness_ < c2.fitness_;
    };

    const auto bestChromosome = std::min_element(chromosome_v.begin(), chromosome_v.end(), compareByFitness);

    return *bestChromosome;
}

ompl::GeneticRRT::Chromosome &ompl::GeneticRRT::select(std::vector<Chromosome> &chromosome_v)
{
    if (chromosome_v.empty())
    {
        throw std::runtime_error("Selection failed: chromosome vector is empty.");
    }

    std::uniform_int_distribution<int> dist(0, chromosome_v.size() - 1);
    auto tournamentSize = std::max(1, static_cast<int>(chromosome_v.size() * 0.05));

    int winnerIndex = dist(rng_);

    for (auto i = 1; i < tournamentSize; i++)
    {
        auto randomIndex = dist(rng_);
        if (chromosome_v[randomIndex].fitness_ < chromosome_v[winnerIndex].fitness_)
        {
            winnerIndex = randomIndex;
        }
    }

    return chromosome_v[winnerIndex];
}

void ompl::GeneticRRT::deleteDuplicates(std::vector<base::State *> &states) const
{
    std::vector<base::State *> seen;

    auto it = std::remove_if(begin(states), end(states), [&](auto state){
                                for(const auto & ti_2 : seen)
                                {
                                    if(si_->equalStates(ti_2, state))
                                    {
                                        return true;
                                    }
                                }
                                seen.push_back(state);
                                return false;
                            });

    states.erase(it, end(states));
}

void ompl::GeneticRRT::mutationDeleteState(std::vector<ompl::base::State *> &states)
{
    if (states.size() < 2)
        throw std::invalid_argument("states size in mutationDeleteState should be greater than 2");

    auto size = states.size();
    if (size > 2)
    {
        std::uniform_int_distribution<int> dist(1, size - 2);
        auto random = dist(rng_);
        states.erase(states.begin() + random);
    }
}

void ompl::GeneticRRT::mutationChangeState(std::vector<ompl::base::State *> &states)
{
    if (states.size() < 2)
        throw std::invalid_argument("states size in mutationChangeState should be greater than 2");

    std::uniform_int_distribution<int> dist(1, states.size() - 2);
    auto randomPos = dist(rng_);

    auto sampler = si_->allocStateSampler();

    base::State *newState = si_->allocState();

    sampler->sampleUniformNear(newState, states[randomPos], 50.0);

    si_->copyState(states[randomPos], newState);
    si_->freeState(newState);
}

ompl::geometric::PathGeometricPtr ompl::GeneticRRT::createPathFromStates(const std::vector<ompl::base::State *> &states) const
{
    if (states.empty())
    {
        throw std::invalid_argument("States in createPathFromStates cannot be null");
    }

    auto path_ptr(std::make_shared<ompl::geometric::PathGeometric>(si_));

    for (int i = states.size() - 1; i >= 0; --i)
        path_ptr->append(states[i]);

    return path_ptr;
}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::chooseBetterPath(const std::vector<base::State *> &path_a, const std::vector<base::State *> &path_b) const
{
    auto pathFather = createPathFromStates(path_a);
    auto pathMother = createPathFromStates(path_b);

    if (pathFather->check() && pathMother->check())
    {
        return (pathFather->length() > pathMother->length()) ? Chromosome{{pathFather}} : Chromosome{{pathMother}};
    }
    else if (pathFather->check())
    {
        return Chromosome{{pathFather}};
    }
    else if (pathMother->check())
    {
        return Chromosome{{pathMother}};
    }
    else
    {
        return Chromosome{{pathFather}}; // Default, if none are valid, return path1
    }
}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::generateOffspring(Chromosome father, Chromosome mother)
{
    auto fatherPath = father.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    auto motherPath = mother.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    auto fatherSize = fatherPath.size();
    auto motherSize = motherPath.size();

    auto minHalfSize = std::min(fatherSize / 2, motherSize / 2);

    std::uniform_int_distribution<int> dist(1, minHalfSize * 2 - 1);
    auto randomPosition = dist(rng_);

    std::swap_ranges(fatherPath.end() - randomPosition, fatherPath.end(), motherPath.end() - randomPosition);

    std::vector<base::State *> statesFather, statesMother;

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

    std::uniform_real_distribution<> mutationDist;

    if (mutationDist(rng_) < probability_)
    {
        mutationDeleteState(statesFather);
        mutationDeleteState(statesMother);
    }

    if (mutationDist(rng_) < probability_)
    {
        mutationChangeState(statesFather);
        mutationChangeState(statesMother);
    }

    return chooseBetterPath(statesFather, statesMother);
}

bool ompl::GeneticRRT::generatePopulation(const base::PlannerTerminationCondition &ptc, std::vector<ompl::GeneticRRT::Chromosome> &population)
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
            throw std::runtime_error("RRT planner failed to find a solution.");
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

    if (!generatePopulation(ptc, population))
    {
        std::cerr << "Population generating failed" << std::endl;
        pdef_->clearSolutionPaths();
        clear();
        return {false, false};
    }

    auto bestResult = findBestChromosome(population);

    for (auto i = 0; i < generationNumber_;)
    {
        auto &father = select(population);
        auto &mother = select(population);

        if (!(father == mother))
        {
            auto child = generateOffspring(father, mother);

            if (child.isValid())
            {
                if (child.fitness_ <= mother.fitness_)
                {
                    mother = child;
                }

                i++;
            }

            auto bestInPopulation = findBestChromosome(population);

            if (bestResult.fitness_ > bestInPopulation.fitness_)
            {
                bestResult = bestInPopulation;
            }
        }
    }
    std::cout << "best result: " << bestResult.fitness_ << std::endl;
    pdef_->clearSolutionPaths();
    pdef_->addSolutionPath(bestResult.genes_);

    end = std::chrono::steady_clock::now();
    duration_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return {true, true};
}