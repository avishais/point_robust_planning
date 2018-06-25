/*
 * plan_C_space.h
 *
 *  Created on: Nov 10, 2016
 *      Author: avishai
 */

#ifndef PLAN_C_SPACE_H_
#define PLAN_C_SPACE_H_

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
// #include <ompl/geometric/planners/rrt/RRT.h>
// #include <ompl/geometric/planners/sst/SST.h>

// Modified and custom planners
// #include "RRT.h"
#ifdef SSTA
	#include "SST.h"
#else
	#include "SSTbelief.h"
#endif

// Standard libraries
#include <iostream>
#include <fstream>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace std;

typedef vector< double > Vector;
typedef vector<vector< double >> Matrix;

// An enum of available planners
enum plannerType
{
	PLANNER_RRT,
	PLANNER_SST
};

bool isStateValid(const ob::State *state);

// Prototypes
class plan_C
{
public:

	void plan(Vector, Vector, double, plannerType = PLANNER_RRT, double = 2);

	// Construct the planner specified by our command line argument.
	// This helper function is simply a switch statement.
	ob::PlannerPtr allocatePlanner(ob::SpaceInformationPtr, plannerType);

	bool solved_bool;
	double total_runtime;
	int ode_count;

	double maxStep;

};

#endif /* PLAN_C_SPACE_H_ */
