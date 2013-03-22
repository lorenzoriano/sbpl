#ifndef ENVIRONMENT_CAR_H
#define ENVIRONMENT_CAR_H

#include <sbpl/discrete_space_information/environment.h>
#include <boost/functional/hash.hpp>
#include <sbpl/utils/utils.h>

#include <cmath>
#include <iostream>

inline int continousSpeedToDisc(float v, const int numofspeeds,
                                const float max_v,
                                const float min_v) {

    if ( v >max_v)
        v = max_v;
    else if (v < min_v)
        v = min_v;
    int discv = floor((v - min_v) / (max_v - min_v) * numofspeeds);
    return discv;
}

struct motion_primitive {
    float dx;
    float dy;
    float dth;
    float v;
    float w;
};

class ContinuousCell {

public:
    ContinuousCell(float x, float y, float th, float v, float w,
                   float map_res,
                   int theta_bins,
                   int v_bins,
                   float max_v = 1.0,
                   float min_v = -0.3
                   ) {
        x_ = x;
        y_ = y;
        th_ = th;
        v_ = v;
        w_ = w;

        map_res_ = map_res;
        theta_bins_ = theta_bins;
        v_bins_ = v_bins;
        w_bins_ = theta_bins;
        max_v_ = max_v;
        min_v_ = min_v;
    }

    std::size_t hash() const {

        using boost::hash_combine;
        std::size_t seed = 0;

        hash_combine(seed, int(floor(x_ / map_res_)));
        hash_combine(seed, int(floor(y_ / map_res_)));
        hash_combine(seed, ContTheta2Disc(th_, theta_bins_));
        hash_combine(seed, continousSpeedToDisc(v_, v_bins_,
                                                max_v_, min_v_));
        hash_combine(seed, ContTheta2Disc(w_,  w_bins_));

        return seed;
    }
    friend std::ostream& operator<<(std::ostream& stream, const ContinuousCell& matrix);

private:
    float x_, y_, th_, v_, w_;
    float map_res_;
    int theta_bins_;
    float v_bins_;
    float w_bins_;
    float max_v_;
    float min_v_;
};

std::ostream& operator<<(std::ostream& stream, const ContinuousCell& cell) {
    stream<<cell.x_<<" "<<cell.y_<<" "<<cell.th_<<" "<<cell.v_<<" "<<cell.w_;
    return stream;
}



class EnvironmentCar : public DiscreteSpaceInformation {

    /**
     * \brief initialization environment from file (see .cfg files for examples)
     */
    virtual bool InitializeEnv(const char* sEnvFile);

    /**
     * \brief initialization of MDP data structure
     */
    virtual bool InitializeMDPCfg(MDPConfig *MDPCfg);

    /**
     * \brief heuristic estimate from state FromStateID to state ToStateID
     */
    virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

    /**
     * \brief heuristic estimate from state with stateID to goal state
     */
    virtual int GetGoalHeuristic(int stateID);

    /**
     * \brief heuristic estimate from start state to state with stateID
     */
    virtual int GetStartHeuristic(int stateID);

    /** \brief depending on the search used, it may call GetSuccs function
     *         (for forward search) or GetPreds function (for backward search)
     *         or both (for incremental search). At least one of this functions should
     *         be implemented (otherwise, there will be no search to run) Some searches
     *         may also use SetAllActionsandAllOutcomes or SetAllPreds functions if they
     *         keep the pointers to successors (predecessors) but most searches do not
     *         require this, so it is not necessary to support this
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

    /**
     * \brief see comments for GetSuccs functon
     */
    virtual void SetAllPreds(CMDPSTATE* state);

    /**
     * \brief returns the number of states (hashentries) created
     */
    virtual int SizeofCreatedEnv();

    /**
     * \brief prints the state variables for a state with stateID
     */
    virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

    /**
     * \brief prints environment config file
     */
    virtual void PrintEnv_Config(FILE* fOut);

    /**
     * \brief sets a parameter to a value. The set of supported parameters depends on the particular environment
     */
    virtual bool SetEnvParameter(const char* parameter, int value)
    {
        SBPL_ERROR("ERROR: Environment has no parameters that can be set via SetEnvParameter function\n");
        return false;
    }

    /** \brief returns true if two states meet the same condition, brief this
     *         is used in some planners to figure out if two states are the same in
     *         some lower-dimensional manifold for example, in robotarm planning, two
     *         states could be equivalent if their end effectors are at the same
     *         position unless overwritten in a child class, this function is not
     *         implemented
     */
    virtual bool AreEquivalent(int StateID1, int StateID2)
    {
        SBPL_ERROR("ERROR: environment does not support calls to AreEquivalent function\n");
        throw new SBPL_Exception();
    }

    /** \brief the following two functions generate succs/preds at some
     *         domain-dependent distance. The number of generated succs/preds is up
     *         to the environment. NOTE: they MUST generate goal state as a succ/pred if
     *         it is within the distance from the state CLowV is the corresponding
     *         vector of lower bounds on the costs from the state to the successor
     *         states (or vice versa for preds function) unless overwritten in a child
     *         class, this function is not implemented
     */
    virtual void GetRandomSuccsatDistance(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CLowV)
    {
        SBPL_ERROR("ERROR: environment does not support calls to GetRandomSuccsatDistance function\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief see comments for GetRandomSuccsatDistance
     */
    virtual void GetRandomPredsatDistance(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CLowV)
    {
        SBPL_ERROR("ERROR: environment does not support calls to GetRandomPredsatDistance function\n");
        throw new SBPL_Exception();
    }

    /**
     * \brief checks the heuristics for consistency (some environments do not support this debugging call)
     */
    virtual void EnsureHeuristicsUpdated(bool bGoalHeuristics)
    {
        // by default the heuristics are up-to-date, but in some cases, the
        // heuristics are computed only when really needed. For example,
        // xytheta environment uses 2D gridsearch as heuristics, and then
        // re-computes them only when this function is called. This minimizes
        // the number of times heuristics are re-computed which is an expensive
        // operation if bGoalHeuristics == true, then it updates goal
        // heuristics (for forward search), otherwise it updates start
        // heuristics (for backward search)
    }


};

#endif // ENVIRONMENT_CAR_H
