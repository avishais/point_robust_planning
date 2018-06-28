#ifndef MYSETUP_H_
#define MYSETUP_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>

// Standard libraries
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <vector>

#define LN cout << __LINE__ << endl;

// Environment parameters
#include "point_sys_def.h"

using namespace std;

namespace ob = ompl::base;
using namespace ob;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

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
        ob::StateValidityChecker(si) {
            obs1.resize(3);
            obs1 = OBS1;
            obs2.resize(3);
            obs2 = OBS2;
            obs3.resize(3);
            obs3 = OBS3;
        }
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {
        	const ob::RealVectorStateSpace::StateType *Q = state->as<ob::RealVectorStateSpace::StateType>();
            double tol = 1.0;

            if ( pow(Q->values[0]-obs1[0], 2) + pow(Q->values[1]-obs1[1], 2) < pow((obs1[2]+CLEARANCE)*tol, 2) )
                return false;
            if ( pow(Q->values[0]-obs2[0], 2) + pow(Q->values[1]-obs2[1], 2) < pow((obs2[2]+CLEARANCE)*tol, 2) )
                return false;
            if ( pow(Q->values[0]-obs3[0], 2) + pow(Q->values[1]-obs3[1], 2) < pow((obs3[2]+CLEARANCE)*tol, 2) )
                return false;

            return true;
    }

    Vector obs1, obs2, obs3;
};


/** Define dynamical system and propagation **/
class gen_system
{
private:

    // This is the f(.) function in \dot{x}=f(x,u)
    // Implemented for a 2D-state point
    Vector f_func(Vector x, Vector u) {
        Vector f(2);

        f[0] = u[0] * cos(u[1]);
        f[1] = u[0] * sin(u[1]);

        return f;
    }

    ompl::RNG rng_;

public:
    // Constructor
    gen_system() {};

    // Propogate state x with random u and dt
    Vector prop(Vector x, Vector u, double dt) {
        Vector x_next(2);

        u[0] += rng_.gaussian(0, .05);
        u[1] += rng_.gaussian(0, .05);   

        Vector f = f_func(x, u);

        x_next[0] = x[0] + f[0] * dt;
        x_next[1] = x[1] + f[1] * dt;

        return x_next;
    }

    // Print vector type
    void printVector(Vector x) {
        cout << "[ " << x[0] << ", " << x[1] << " ]" << endl;
    }
};

#endif