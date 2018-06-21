#ifndef MYSETUP_H_
#define MYSETUP_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>

// Environment parameters
#include "../systems/point_sys_def.h"

namespace ob = ompl::base;
using namespace ob;

vector<double> obs1 = OBS1;
vector<double> obs2 = OBS2;
vector<double> obs3 = OBS3;

/** Defines an optimization objective which attempts to minimize the length of the path in the R^2 X S configuration space
 *
 */
class LengthObjective : public ob::OptimizationObjective
{
public:
	LengthObjective(const ob::SpaceInformationPtr& si) : ob::OptimizationObjective(si) {
        // Setup a default cost-to-go heuristics:
        setCostToGoHeuristic(ob::goalRegionCostToGo);
    }

    ob::Cost stateCost(const ob::State* s) const
    {
      	return identityCost();
    }

    ob::Cost motionCost(const State *s1, const State *s2) const
    {
    	return ob::Cost(si_->distance(s1, s2));
    }

    ob::Cost motionCostHeuristic(const State *s1, const State *s2) const
    {
    	return motionCost(s1, s2);
    }
};


// Our collision checker. For this demo, our robot's state space
// lies in [0,1]x[0,1], with a circular obstacle of radius 0.25
// centered at (0.5,0.5). Any states lying in this circular region are
// considered "in collision".
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();
            double tol = 1.05;

            if ( pow(Q->values[0]-obs1[0], 2) + pow(Q->values[1]-obs1[1], 2) < pow(obs1[2]*tol, 2) )
                return false;
            if ( pow(Q->values[0]-obs2[0], 2) + pow(Q->values[1]-obs2[1], 2) < pow(obs2[2]*tol, 2) )
                return false;
            if ( pow(Q->values[0]-obs3[0], 2) + pow(Q->values[1]-obs3[1], 2) < pow(obs3[2]*tol, 2) )
                return false;

            return true;
    }

};

#endif