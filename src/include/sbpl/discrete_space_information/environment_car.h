#ifndef ENVIRONMENT_CAR_H
#define ENVIRONMENT_CAR_H

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/utils/continuouscell.h>
#include <sbpl/utils/car_motion_primitive.h>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <iomanip>
#include <sstream>
#include <exception>
#include <utility>

typedef double scalar;

class CarException : public std::exception {
public:
    CarException(std::string msg) {
        msg_ = msg;
    }

    virtual ~CarException() throw() {}

    virtual const char* what() const throw(){
        return msg_.c_str();
    }

private:
    std::string msg_;
};


class EnvironmentCar : public DiscreteSpaceInformation {

public:
    EnvironmentCar(scalar map_res, scalar car_length,
                   bool fixed_cells,
                   int theta_bins,
                   scalar max_v,
                   scalar min_v,
                   scalar max_steer,
                   scalar min_steer,
                   bool store_graph,
                   scalar flipping_cost=1.0);

    EnvironmentCar(const char *cfg_file, bool store_graph);

    void setGoal(scalar x, scalar y, scalar th);
    void setStart(scalar x, scalar y, scalar th);

    virtual bool isReachableCells(const ContinuousCellPtr& start,
                                  const ContinuousCellPtr& dest,
                                  double initial_steering,
                                  double duration) const;

    friend std::ostream& operator<<(std::ostream& stream, const EnvironmentCar& cell);

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

    bool loadPrimitives(const char* filename);

    /**
     * @brief Finds a cell from its id
     *
     * @param state_id the id of the cell
     * @return the cell
     */
    ContinuousCellPtr findCell(int state_id) const;

    int numStates() const {
        assert(cells_.size() == idCellsMap_.size());
        return cells_.size();
    }

    bool saveSolutionYAML(const std::vector<int>& ids, const char* filename) const;

protected:

    int addHashMapping(ContinuousCellPtr c);
    ContinuousCellPtr addCell(ContinuousCellPtr c);

    std::vector<motion_primitive> primitives_;
    scalar simulation_time_step_;

    boost::unordered_set<ContinuousCellPtr> cells_;
    boost::unordered_map<int, ContinuousCellPtr> idCellsMap_;

    scalar car_length_;
    scalar map_res_;
    scalar theta_bins_;
    scalar  max_v_;
    scalar  min_v_;
    scalar  max_steer_;
    scalar  min_steer_;
    scalar flipping_cost_;
    bool fixed_cells_;
    bool store_graph_;

    std::size_t start_id_;
    std::size_t goal_id_;
    ContinuousCellWeakPtr goal_cell_;
    ContinuousCellWeakPtr start_cell_;
};

inline std::ostream& operator<<(std::ostream& stream, const EnvironmentCar& env) {
    stream<<"Car length: "<<env.car_length_<<" ";
    stream<<"Map resolution: "<<env.map_res_<<" ";
    stream<<"Theta bins: "<<env.theta_bins_<<" ";
    stream<<"Max velocity: "<<env.max_v_<<" ";
    stream<<"Min velocity: "<<env.min_v_<<" ";
    stream<<"Max steering angle: "<<env.max_steer_<<" ";
    stream<<"Min steering angle: "<<env.min_steer_<<" ";
    stream<<"Fixed Cells: "<<env.fixed_cells_<<" ";
}

#endif // ENVIRONMENT_CAR_H
