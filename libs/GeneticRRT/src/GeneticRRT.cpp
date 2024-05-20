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

int ompl::GeneticRRT::select(std::vector<Chromosome>  chromosome_v)
{
    std::uniform_int_distribution<int> dist(0, chromosome_v.size() - 1);
    std::map<double,int> members;

    for(auto i = 0; i < chromosome_v.size()*0.05; i++)
    {
        auto randomnumber = dist(rng);
        members[chromosome_v[randomnumber].fitness_] = randomnumber;
    }

    return members.begin()->second;

}

void ompl::GeneticRRT::mutation(std::vector<ompl::base::State *> &path)
{

    for(auto i = 1; i<path.size()-1; i++)
    {
        si_->allocStateSampler()->sampleUniformNear(path[i], path[i], 5);
    }
        


}

ompl::GeneticRRT::Chromosome ompl::GeneticRRT::GA(Chromosome father, Chromosome mother)
{
    auto fatherPath = father.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    auto motherPath = mother.genes_.path_->as<ompl::geometric::PathGeometric>()->getStates();
    // std::ofstream fileStream;
    // fileStream.open("plik.txt");

    // father.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
    // mother.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);

    auto halfFatherSize = fatherPath.size() / 2;
    auto halfMotherSize = motherPath.size() / 2;

    auto minHalfSize = std::min(halfFatherSize, halfMotherSize);

    std::uniform_int_distribution<int> dist(1, minHalfSize*2 -1);
    auto random = dist(rng);

    std::swap_ranges(fatherPath.end() - random, fatherPath.end(), motherPath.end() - random);

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

    
    std::uniform_int_distribution<int> dist_2(0, 1000);
    auto randomNumber = dist_2(rng);

    if (randomNumber == 1)
    {
        //mutation(statesFather);
       // mutation(statesMother);
        std::cout << "udalo sie!" << std::endl;
    }

    auto newPathFather(std::make_shared<ompl::geometric::PathGeometric>(si_));
    auto newPathMother(std::make_shared<ompl::geometric::PathGeometric>(si_));

    for (int i = statesFather.size() - 1; i >= 0; --i)
        newPathFather->append(statesFather[i]);

    
    for (int i = statesMother.size() - 1; i >= 0; --i)
        newPathMother->append(statesMother[i]);


    if(newPathFather->check() && newPathMother->check())
    {
        if(newPathFather->length() > newPathMother->length())
        {
            Chromosome child{{newPathFather}};
            //child.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
            return child;
        }
        else
        {
            Chromosome child{{newPathMother}};
           // child.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
            return child;
        }
    }
    else if(newPathFather->check())
    {
        Chromosome child{{newPathFather}};
        //child.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
        return child;
    }
    else if(newPathMother->check())
    {
        Chromosome child{{newPathMother}};
        //child.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
        return child;
    }
    else
    {
        return {{newPathFather}};
    }


    // Chromosome child({newPathFather});
        
}

ompl::base::PlannerStatus ompl::GeneticRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    RRTplanner_p->setProblemDefinition(this->getProblemDefinition());
    ompl::base::PlannerStatus ss;
    //RRTplanner_p->setRange(30.0);
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
            pdef_->clearSolutionPaths();
            RRTplanner_p->clear();
            return {false, false};
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
    

        if(father != mother){
            auto child = GA(population[father], population[mother]);
            

            if (child.isValid())
            {
                if(child.fitness_ < population[mother].fitness_)
                {
                    population[mother] = child;
                    //std::cout << child.fitness_ << std::endl;

                }
                if(child.fitness_ < p.fitness_)
                {
                    child.genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);
                }
                i++;
                std::cout << child.fitness_ << std::endl;
            }
        }
    }

    auto  k = findBestChromosome(population);

    k->calculateFintess();
    std::cout << k->fitness_ << "dupa!" <<  p.fitness_ << std::endl;
    
    k->genes_.path_->as<ompl::geometric::PathGeometric>()->printAsMatrix(fileStream);

    pdef_->clearSolutionPaths();
    pdef_->addSolutionPath((*k).genes_);

    return ss;
}