#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <random>
#include <chrono>

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

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
            void calculateFitness();
            bool isValid() const;
            bool operator ==(const Chromosome &c) const;
        };

        
        int populationNumber_;
        int generationNumber_;
        double  probability_;  // Probability of mutation (0.0 to 1.0)
        std::mt19937 rng;
        int64_t duration;
        
        auto findBestChromosome(std::vector<Chromosome>const &chromosome_v) const;
        void mutationDeleteState(std::vector<base::State *> &states);
        void mutationChangeState(std::vector<base::State *> &states);
        geometric::PathGeometricPtr createPathFromStates(std::vector<base::State *> const &states) const;
        Chromosome chooseBetterPath(std::vector<base::State *> const &path_a, std::vector<base::State *> const  &path_b) const;
        Chromosome& select(std::vector<Chromosome>  &chromosome_v);
        Chromosome generateOffspring(Chromosome father, Chromosome mother);
        void deleteDuplicates(std::vector<base::State *> &states);
        bool generatePopulation(const base::PlannerTerminationCondition &ptc, std::vector<ompl::GeneticRRT::Chromosome>& population);

    public:
        GeneticRRT(const base::SpaceInformationPtr &si);
        virtual ~GeneticRRT(void);
        virtual void getPlannerData(base::PlannerData &data) const;
        virtual void clear(void);
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

        void setPopulation(int population);
        int getPopulation() const;
        void setGeneration(int generation);
        int getGeneration() const;
        int64_t getDuration() const;

        void setProbability(double probability);
        double getProbability() const;

    };
}
