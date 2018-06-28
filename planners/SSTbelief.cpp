/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

// #include "ompl/geometric/planners/sst/SST.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

#include "SSTbelief.h"

ompl::geometric::SST::SST(const base::SpaceInformationPtr &si) : base::Planner(si, "SST")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;
    prevSolution_.clear();

    Planner::declareParam<double>("range", this, &SST::setRange, &SST::getRange, ".1:.1:100");
    Planner::declareParam<double>("goal_bias", this, &SST::setGoalBias, &SST::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &SST::setSelectionRadius, &SST::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
    Planner::declareParam<double>("pruning_radius", this, &SST::setPruningRadius, &SST::getPruningRadius, "0.:.1:100");
}

ompl::geometric::SST::~SST()
{
    freeMemory();
}

void ompl::geometric::SST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    {
                                        return distanceFunction(a, b);
                                    });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::geometric::SST::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::geometric::SST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_)
                si_->freeState(motion->state_);
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            if (witness->state_)
                si_->freeState(witness->state_);
            delete witness;
        }
    }

    for (auto &i : prevSolution_)
    {
        if (i)
            si_->freeState(i);
    }
    prevSolution_.clear();
}

void ompl::geometric::SST::retrieveStateVector(const base::State *state, Vector &q) {
	// cast the abstract state type to the type we expect
	const base::RealVectorStateSpace::StateType *Q = state->as<base::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 2; i++) {
		q[i] = Q->values[i]; // Set state of robot1
	}
}

void ompl::geometric::SST::updateStateVector(const base::State *state, Vector q) {
	// cast the abstract state type to the type we expect
	const base::RealVectorStateSpace::StateType *Q = state->as<base::RealVectorStateSpace::StateType>();

	for (unsigned i = 0; i < 2; i++) {
		Q->values[i] = q[i];
	}
}

void ompl::geometric::SST::printStateVector(const base::State *state) {
	// cast the abstract state type to the type we expect
	const base::RealVectorStateSpace::StateType *Q = state->as<base::RealVectorStateSpace::StateType>();

    cout << "[ ";
	for (unsigned i = 0; i < 2; i++) 
		cout << Q->values[i] << " "; 
    cout << "]" << endl;
}

ompl::geometric::SST::Motion *ompl::geometric::SST::selectNode(ompl::geometric::SST::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);
    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::geometric::SST::Witness *ompl::geometric::SST::findClosestWitness(ompl::geometric::SST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        if (distanceFunction(closest, node) > pruningRadius_)
        {
            closest = new Witness(si_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(si_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::State *ompl::geometric::SST::monteCarloProp(Motion *m) {
    // Destination state
    base::State *xstate = si_->allocState();

    Vector x_ng(2);
    retrieveStateVector(m->state_, x_ng);

    do {
        // Sample time step
        double dt = maxTimeStep_/2; //rng_.uniformReal(0, maxTimeStep_);

        // Sample control
        Vector u(2);
        u[0] = rng_.uniformReal(0, maxVelocity_);
        u[1] = rng_.uniformReal(-PI, PI);

        // Take a step with dt and control u
        Vector x_new = prop(x_ng, u, dt);
        updateStateVector(xstate,  x_new);
    } while (!si_->satisfiesBounds(xstate)); // Verify that propagation is within the bounds

    return xstate;
}

ompl::geometric::SST::Motion *ompl::geometric::SST::ParticlesProp(Motion *nmotion) {
    // Destination state
    base::State *xstate = si_->allocState();
    base::State *wstate = si_->allocState();

    Vector x_ng(2);
    retrieveStateVector(nmotion->state_, x_ng);

    // Sample control
    Vector u(2);
    u[0] = rng_.uniformReal(0, maxVelocity_);
    u[1] = rng_.uniformReal(-PI, PI);

    Vector MeanState(2, 0); // Should be removed when including clustering
    int numSuccess_particles = 0;
    Matrix Pa;
    for (int i = 0; i < MAXNUMPARTICLES; i++) {
        // randomly pick a particle and propagate
        int l = rng_.uniformInt(0, nmotion->nParticles_-1);

        // Take a step from a particle with DT and control u
        Vector x_new = prop(nmotion->particles[l], u, DT);
        updateStateVector(xstate, x_new);
        updateStateVector(wstate, nmotion->particles[l]);
        if (!si_->satisfiesBounds(xstate) || !si_->checkMotion(wstate, xstate)) // Verify that propagation is within the bounds and collision free
            continue;

        Pa.push_back(x_new);
        numSuccess_particles++;

        MeanState[0] += x_new[0];
        MeanState[1] += x_new[1];
    } 

    si_->freeState(xstate);
    si_->freeState(wstate);

    if (numSuccess_particles > 0){//.1 * MAXNUMPARTICLES) { // If could not propagate enough (less than 10% of MAXNUMPARTICLES particles)

        cluster C = meanshift(Pa, CLEARANCE, 0.001);
        // cout << "------\n";
        // cout << Pa.size() << " " << C.points.size() << endl;
        // cout << C.centroid[0] << " " << C.centroid[1] << endl;
        // cout << MeanState[0] / Pa.size() << " " << MeanState[1] / Pa.size() << endl;

        // if (C.stddev > 100) 
        //     return nullptr;

        // Bias toward more qualitative nodes
        double prob = nmotion->probability_ * double(C.points.size()) / MAXNUMPARTICLES;
        // double qual;
        // if (fabs(min_probability_- 1.) < 1e-3)
        //     qual = prob; //pow(motion->probability_, 1.0/(nmotion->nodesFromRoot_+1))
        // else
        //     qual = (prob - min_probability_) / (1. - min_probability_);
        if (prob < rng_.uniform01()) 
            return nullptr;

        // Create new motion
        auto *motion = new Motion(si_);
        motion->particles = C.points;
        motion->nParticles_ = C.points.size();
        motion->action.push_back(u[0]);
        motion->action.push_back(u[1]);
        motion->probability_ = prob;
        // motion->quality_ = qual;
        updateStateVector(motion->state_, C.centroid);

        return motion;
    }

    return nullptr;
}

ompl::base::PlannerStatus ompl::geometric::SST::solve(const base::PlannerTerminationCondition &ptc)
{
    
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    goal_s->setThreshold(maxDistance_); // Set region for the goal

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state_, st);
        sampleParticles4Motion(motion);
        motion->accCost_ = opt_->combineCosts( opt_->identityCost(), opt_->costToGo(motion->state_, goal) );
        motion->rootToStateCost_ = opt_->identityCost();
        motion->probability_ = 1;
        motion->quality_ = 1;
        motion->nodesFromRoot_ = 0;
        motion->action.push_back(0);
        motion->action.push_back(0);
        nn_->add(motion);
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state_;
    base::State *xstate = si_->allocState();

    unsigned iterations = 0;


    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        bool attemptToReachGoal = (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample());
        if (attemptToReachGoal)
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);

        // dstate = monteCarloProp(nmotion);
        auto *motion = ParticlesProp(nmotion); // Propagation already creates a motion, if enough particles

        if (motion != nullptr)
        {
            base::Cost incCost = opt_->combineCosts( opt_->motionCost(nmotion->state_, motion->state_), opt_->costToGo(motion->state_, goal) ); // c + h
            base::Cost cost = opt_->combineCosts(nmotion->rootToStateCost_, incCost);
            cost = opt_->combineCosts(cost, ob::Cost(10. / motion->probability_));
            Witness *closestWitness = findClosestWitness(motion);

            if (closestWitness->rep_ == motion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
            {
                Motion *oldRep = closestWitness->rep_;
                motion->accCost_ = cost;
                motion->rootToStateCost_ = opt_->combineCosts( nmotion->rootToStateCost_, opt_->motionCost(nmotion->state_, motion->state_) );
                motion->parent_ = nmotion;
                nmotion->numChildren_++;
                motion->nodesFromRoot_ = nmotion->nodesFromRoot_ + 1;
                if (motion->probability_ < min_probability_) // track minimum quality
                    min_probability_ = motion->probability_;
                
                closestWitness->linkRep(motion);

                nn_->add(motion);

                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state_, &dist);
                if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                {
                    approxdif = dist;
                    solution = motion;

                    for (auto &i : prevSolution_)
                        if (i)
                            si_->freeState(i);
                    prevSolution_.clear();
                    Motion *solTrav = solution;
                    while (solTrav != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        solTrav = solTrav->parent_;
                    }
                    prevSolutionCost_ = solution->accCost_;

                    OMPL_INFORM("Found solution with cost %.2f and (probability, quality) <%.2f, %.2f>", solution->accCost_.value(), solution->probability_, solution->quality_);
                    sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                    if (sufficientlyShort)
                    {
                        break;
                    }
                }

                if (solution == nullptr && dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;

                    for (auto &i : prevSolution_)
                    {
                        if (i)
                            si_->freeState(i);
                    }
                    prevSolution_.clear();
                    Motion *solTrav = approxsol;
                    while (solTrav != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        solTrav = solTrav->parent_;
                    }
                }
 
                if (solution != nullptr && oldRep != motion && oldRep != solution) // This change prevents the deletion of the current solution via oldRep
                {
                    /** This is a correction of the OMPL version when it is possible that the oldRep 
                        node is also the estimated solution. Therefore, the new rep replaces it as the approximated solution so far.
                    **/
                    if (oldRep == approxsol) { 
                        approxsol = motion;
                        approxdif = dist;
                    }

                    oldRep->inactive_ = true;
                    nn_->remove(oldRep);
                    while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                    {
                        if (oldRep->state_)
                            si_->freeState(oldRep->state_);
                        oldRep->state_ = nullptr;
                        oldRep->parent_->numChildren_--;
                        Motion *oldRepParent = oldRep->parent_;
                        delete oldRep;
                        oldRep = oldRepParent;
                    }
                }
            }
            else
                delete motion;
        }
        iterations++;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = prevSolution_.size() - 1; i >= 0; --i)
            path->append(prevSolution_[i]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());

        if (approximate)
            OMPL_INFORM("Final approximated solution with cost %.2f and (probability, quality) <%.2f, %.2f>", solution->accCost_.value(), solution->probability_, solution->quality_);
        else
            OMPL_INFORM("Final solution with cost %.2f and (probability, quality) <%.2f, %.2f>", solution->accCost_.value(), solution->probability_, solution->quality_);

        listTree();
        simulate(solution);
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    rmotion->state_ = nullptr;
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

    return base::PlannerStatus(solved, approximate);
}

void ompl::geometric::SST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
        if (motion->numChildren_ == 0)
            allMotions.push_back(motion);
    for (unsigned i = 0; i < allMotions.size(); i++)
        if (allMotions[i]->getParent() != nullptr)
            allMotions.push_back(allMotions[i]->getParent());

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto &allMotion : allMotions)
    {
        if (allMotion->getParent() == nullptr)
            data.addStartVertex(base::PlannerDataVertex(allMotion->getState()));
        else
            data.addEdge(base::PlannerDataVertex(allMotion->getParent()->getState()),
                         base::PlannerDataVertex(allMotion->getState()));
    }
}

void ompl::geometric::SST::sampleParticles4Motion(Motion *motion) {

    // Gen initial particles
    Vector s(2);
    retrieveStateVector(motion->state_, s);
    for (int i = 0; i < MAXNUMPARTICLES; i++)
        motion->particles.push_back({rng_.gaussian(s[0], motion->stddev[0]), rng_.gaussian(s[1], motion->stddev[1])});
}


void ompl::geometric::SST::listTree() {

    std::ofstream TF, TP, TM;
	TF.open("./path/tree.txt");
    TP.open("./path/particles.txt");
    TM.open("./path/motions.txt");

    std::vector<Motion*> motions;
	nn_->list(motions);

    Vector q1(2), q2(2);
    for (int i = 1; i < motions.size(); i++) {
        Motion* tmotion = motions[i];
        if (tmotion->numChildren_ == 0) {
            while (tmotion->parent_ != nullptr) {
                retrieveStateVector(tmotion->state_, q1);
                retrieveStateVector(tmotion->parent_->state_, q2);
                TF << q1[0] << " " << q1[1] << " " << q2[0] << " " << q2[1] << endl;

                for (int j = 0; j < tmotion->particles.size(); j++) {
                    TP << tmotion->particles[j][0] << " " << tmotion->particles[j][1] << endl;
                    // TP << tmotion->parent_->particles[j][0] << " " << tmotion->parent_->particles[j][1] << endl;
                }

                TM << q1[0] << " " << q1[1] << endl;
                TM << q2[0] << " " << q2[1] << endl;

                tmotion = tmotion->parent_;
            }
        } 
    }

    TF.close();
    TP.close();
    TM.close();
}

void ompl::geometric::SST::simulate(Motion *solution) {
    
    cout << "Simulating...\n";
    Matrix Ur, U;
    Motion *solTrav = solution;
    Vector x(2);

    while (solTrav != nullptr)
    {
        Ur.push_back(solTrav->action);
        retrieveStateVector(solTrav->state_, x);
        solTrav = solTrav->parent_;
    }
    Ur.pop_back();
    for (int i = Ur.size() - 1; i >= 0; --i)
        U.push_back(Ur[i]);

    std::ofstream TS;
	TS.open("./path/sim_path.txt");

    TS << x[0] << " " << x[1] << endl;

    for (int i = 0; i < U.size(); i++) {
        x = prop(x, U[i], DT);
        TS << x[0] << " " << x[1] << endl;
    }

    TS.close();
}
 