/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Avishai Sintov */

#include "plan.h"

ob::PlannerPtr plan_C::allocatePlanner(ob::SpaceInformationPtr si, Matrix ref_path, plannerType p_type)
{
    switch (p_type)
    {
        // case PLANNER_RRT:
        // {
        //     return std::make_shared<og::RRT>(si); 
        //     break;
        // }
		case PLANNER_SST:
        {
            return std::make_shared<og::SST>(si, ref_path); 
            break;
        }
        default:
        {
            OMPL_ERROR("Planner-type enum is not implemented in allocation function.");
            return ob::PlannerPtr(); // Address compiler warning re: no return value.
            break;
        }
    }
}

void plan_C::plan(Matrix ref_path, double runtime, plannerType ptype, double max_step) {

	// construct the state space we are planning inz
	ob::StateSpacePtr Q(new ob::RealVectorStateSpace(2)); 

	// set the bounds for the Q=R^2 part of 'Cspace'
	ob::RealVectorBounds Qbounds(2);
	Qbounds.setLow(0, -10); // X
	Qbounds.setHigh(0, 10);
	Qbounds.setLow(1, -10); // Y
	Qbounds.setHigh(1, 10);

	// set the bound for the compound space
	Q->as<ob::RealVectorStateSpace>()->setBounds(Qbounds);

	// construct a compound state space using the overloaded operator+
	ob::StateSpacePtr Cspace(Q);

	// construct an instance of  space information from this state space
	ob::SpaceInformationPtr si(new ob::SpaceInformation(Cspace));

	// set state validity checking for this space
	si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
	// si->setStateValidityChecker(std::bind(&isStateValid, std::placeholders::_1));
	si->setStateValidityCheckingResolution(0.05);

	// create start state
	ob::ScopedState<ob::RealVectorStateSpace> start(Cspace);
	for (int i = 0; i < ref_path.front().size(); i++) {
		start->as<ob::RealVectorStateSpace::StateType>()->values[i] = ref_path.front()[i]; // Access the first component of the start a-state
	}

	// create goal state
	ob::ScopedState<ob::RealVectorStateSpace> goal(Cspace);
	for (int i = 0; i < ref_path.back().size(); i++) {
		goal->as<ob::RealVectorStateSpace::StateType>()->values[i] = ref_path.back()[i]; // Access the first component of the goal a-state
	}

	// create a problem instance
	ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

	// set the start and goal states
	pdef->setStartAndGoalStates(start, goal);
	pdef->print();

	// Set the optimization objective
	pdef->setOptimizationObjective( ob::OptimizationObjectivePtr(new LengthObjective(si, ref_path)) );

	maxStep = max_step;
	// create a planner for the defined space
	// To add a planner, the #include library must be added above
	ob::PlannerPtr planner = allocatePlanner(si, ref_path, ptype);

	// set the problem we are trying to solve for the planner
	planner->setProblemDefinition(pdef);

	// perform setup steps for the planner
	planner->setup();

	//planner->printSettings(std::cout); // Prints some parameters such as range
	//planner->printProperties(std::cout); // Prints some decisions such as multithreading, display approx solutions, and optimize?

	// print the settings for this space
	//si->printSettings(std::cout); // Prints state space settings such as check resolution, segmant count factor and bounds
	//si->printProperties(std::cout); // Prints state space properties, average length, dimension ...

	// print the problem settings
	//pdef->print(std::cout); // Prints problem definition such as start and goal states and optimization objective

	// attempt to solve the problem within one second of planning time
	auto sT = Clock::now();
	ob::PlannerStatus solved = planner->solve(runtime);
	cout << "Runtime: " << std::chrono::duration<double>(Clock::now() - sT).count() << endl;

	if (solved) {
		// get the goal representation from the problem definition (not the same as the goal state)
		// and inquire about the found path
		ob::PathPtr path = pdef->getSolutionPath();

		// print the path to screen
		//path->print(std::cout);  // Print as vectors

		// Save path to file
		std::ofstream myfile;
		myfile.open("./path/path.txt");
		og::PathGeometric& pog = static_cast<og::PathGeometric&>(*path); // Transform into geometric path class
		pog.printAsMatrix(myfile); // Print as matrix to file
		myfile << ref_path.back()[0] << " " << ref_path.back()[1] << endl;
		myfile.close();

		std::cout << "Found solution:" << std::endl;
		solved_bool = true;

		system("cd simulator && ./viz & cd .. &");
		system("python plotCost.py");

	}
	else {
		std::cout << "No solutions found" << std::endl;
		solved_bool = false;
	}
}

void extract_from_perf_file(ofstream &ToFile) {
	ifstream FromFile;
	FromFile.open("./paths/perf_log.txt");

	string line;
	while (getline(FromFile, line))
		ToFile << line << "\t";

	FromFile.close();
}

int main(int argn, char ** args) {
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	double runtime; // Maximum allowed runtime
	plannerType ptype; // Planner type
	string plannerName;

	if (argn == 1) {
		runtime = 1; // sec
		ptype = PLANNER_SST;
		plannerName = "SST";		
	}
	else if (argn == 2) {
		runtime = atof(args[1]);
		ptype = PLANNER_SST;
		plannerName = "SST";		
	}
	else if (argn > 2) {
		runtime = atof(args[1]);
		switch (atoi(args[2])) {
		case 2 :
			ptype = PLANNER_RRT;
			plannerName = "RRT";
			break;
		case 1 :
			ptype = PLANNER_SST;
			plannerName = "SST";
			break;
		default :
			cout << "Error: Requested planner not defined.";
			exit(1);
		}
	}

	plan_C Plan;

	srand (time(NULL));

	Matrix ref_path = {{-8, -7}, {-8, 0}, {-0.5, 6}, {7, 7}};
		
	Plan.plan(ref_path, runtime, ptype, 0.5);

	std::cout << std::endl << std::endl;

	return 0;
}

