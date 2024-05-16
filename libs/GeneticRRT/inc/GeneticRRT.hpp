#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <random>

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>

namespace ompl
{
    class GeneticRRT : public base::Planner
    {
    private:
        class Chromosome
        {
        public:
            base::PlannerSolution genes_;
            double fitness_;
    
            Chromosome(base::PlannerSolution genes);
            void calculateFintess();
            bool isValid() const;

        };

        std::shared_ptr<geometric::RRT> RRTplanner_p;
        int populationNumber_;
        int generationNumber_;
        
        auto findBestChromosome(std::vector<Chromosome> &chromosome_v) const;
        Chromosome GA(Chromosome father, Chromosome mother);
        

    public:
        GeneticRRT(const base::SpaceInformationPtr &si);
        virtual ~GeneticRRT(void);
        virtual void getPlannerData(base::PlannerData &data) const;
        virtual void clear(void);
        virtual void setup(void);
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

        void setPopulation(int population);
        int getPopulation() const;
        void setGeneration(int generation);
        int getGeneration() const;
    };

}

void foo(int k);