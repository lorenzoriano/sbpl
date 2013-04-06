#ifndef ENVIRONMENT_CAR_H
#define ENVIRONMENT_CAR_H

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <sbpl/utils/car_simulator.h>

#include <boost/functional/hash.hpp>
#include <boost/bimap.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <map>
#include <iomanip>
#include <sstream>
#include <exception>

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

/**
 * @brief Discretize an element into bins.
 *
 * @param x the number to discretize
 * @param numofbins the number of bins to use
 * @param max the maximum value for x
 * @param min the minimum value for x
 * @return the discretized value
 */
inline int continousToDiscBins(float x, const int numofbins,
                                const float max,
                                const float min) {
    if ( x >max)
        x = max;
    else if (x < min)
        x = min;
    int discv = round((x - min) / (max - min) * (numofbins-1));
    return discv;
}

inline float wrap_angle(float angle) {
    while (angle >= 2*M_PI)
        angle -= 2*M_PI;

    while (angle<0)
        angle += 2*M_PI;

    return angle;
}

/**
 * @brief Bins an angle.  Returns the index of
    the bin (that is, discrete value).
 *
 * @param x the angle to discretize
 * @param numofbins the number of bins
 * @return the discretized value
 */
inline int bin_angle(float x, int numofbins) {
    x = wrap_angle(x);
    return continousToDiscBins(x, numofbins, 2.*M_PI, 0.);
}

/**
 * @brief Converts a binned angle to a continous value.
 *
 * @param the (discrete) angle
 * @param numofbins the numebr of bins
 * @return the continous angle
 */
inline float continous_angle(int x, int numofbins) {
    return x * 2.*M_PI / (numofbins-1);
}

/**
 * @brief Discretizes an angle, by first binning it, then converting it
 * back to a continous value.
 *
 * @param angle the angle to discretize
 * @param numofbins the number of bins
 * @return the (continous) discretized angle.
 */
float discretize_angle(float angle, int numofbins) {
    int binned = bin_angle(angle, numofbins);
    return continous_angle(binned, numofbins);
}

/**
 * @brief Discretizes a coordinate by returing the coordinate of the
 * center of the correspoding cell.
 *
 * @param x the value to discretize
 * @param map_resolution the resolution fo the map
 * @return the center of the corresponding cell
 */
inline float discretize_coordinate(float x, float map_resolution) {
    return round(x/map_resolution) * map_resolution;
}

/**
 * @brief Returns the absolute difference between two angles, taking into
 * consideration wrapping around.
 *
 * @param a1 first angle, in radians
 * @param a2 second angle, in radians
 * @return the difference between a1 and a2, between -pi and pi
 */
inline float diff_angle(float a1, float a2) {
     return M_PI - fabs(M_PI - fabs(a1 -a2));
}

/**
 * @brief This is a basic motion primitive, with velocity, steering angle, duration
 * and cost.
 */
struct motion_primitive {
    float v;
    float steer;
    float duration;
    int cost;

    bool operator==(const motion_primitive& p) {
        return (v== p.v) && (steer == p.steer) && (duration == p.duration) && (cost == p.cost);
    }
};


std::ostream& operator<<(std::ostream& stream, const motion_primitive& p) {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(2);
    ss<<"v: "<<p.v<<" steer: "<<p.steer<<" cost: "<<p.cost<<" duration: "<<p.duration;
    stream<<ss.str();
    return stream;
}

/**
 * @brief This is cell in a lattice. The x,y,th values will be discretized
 * at construction time
 */
class ContinuousCell {

public:
    ContinuousCell(float x, float y, float th,
                   float map_res,
                   int theta_bins
                   ) {
        x_ = discretize_coordinate(x, map_res);
        y_ = discretize_coordinate(y, map_res);
        th_ = discretize_angle(th, theta_bins);
//        x_ = x;
//        y_ = y;
//        th_ = th;

        map_res_ = map_res;
        theta_bins_ = theta_bins;
        cached_hash_ = 0;
        hash_calculated_ = false;
    }

    ContinuousCell(const CarSimulator::state_type& p, float map_res, int theta_bins) {
        x_ = discretize_coordinate(p[0], map_res);
        y_ = discretize_coordinate(p[1], map_res);
        th_ = discretize_angle(p[2], theta_bins);
//        x_ = p[0];
//        y_ = p[1];
//        th_ = p[2];

        map_res_ = map_res;
        theta_bins_ = theta_bins;
        cached_hash_ = 0;
        hash_calculated_ = false;
    }

    float x() const {
        return x_;
    }
    float y() const {
        return y_;
    }
    float th() const {
        return th_;
    }

    std::size_t hash() const {

        if (hash_calculated_)
            return cached_hash_;

        using boost::hash_combine;
        std::size_t seed = 0;

        hash_combine(seed, x_);
        hash_combine(seed, y_);
        hash_combine(seed, th_);
//        hash_combine(seed, discretize_coordinate(x_, map_res_));
//        hash_combine(seed, discretize_coordinate(y_, map_res_));
//        hash_combine(seed, bin_angle(th_, theta_bins_));

        cached_hash_ = seed;
        hash_calculated_ = true;
        return seed;
    }

    std::string repr() const {
        std::stringstream ss;
        ss.unsetf(std::ios::floatfield);
        ss<<std::setprecision(2);
        ss<<x_<<" "<<y_<<" "<<th_;
        return ss.str();
    }

    bool operator==(const ContinuousCell& c) {
        return hash() == c.hash();
    }

    CarSimulator::state_type toCarState() const {
        CarSimulator::state_type s;
        s[0] = x_;
        s[1] = y_;
        s[2] = th_;
        return s;
    }

    friend std::ostream& operator<<(std::ostream& stream, const ContinuousCell& matrix);

private:
    float x_, y_, th_;
    float map_res_;
    float theta_bins_;
    mutable bool hash_calculated_;
    mutable std::size_t cached_hash_;

};

std::ostream& operator<<(std::ostream& stream, const ContinuousCell& cell) {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(2);
    ss<<cell.x_<<" "<<cell.y_<<" "<<cell.th_;
    stream<<ss.str();
    return stream;
}

class EnvironmentCar : public DiscreteSpaceInformation {

public:
    EnvironmentCar(float map_res, float car_length,
                   int theta_bins,
                   float max_v,
                   float min_v,
                   float max_steer,
                   float min_steer);

    EnvironmentCar(const char *cfg_file);

    void setGoal(float x, float y, float th);
    void setStart(float x, float y, float th);

    bool isValidCell(const ContinuousCell& c);

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
    const ContinuousCell &findCell(int state_id) const;
    ContinuousCell &findCell(int state_id);

    int numStates() const {
        assert(cells_map_.size() == hash_int_map_.size());
        return cells_map_.size();
    }

    motion_primitive findPrimitive(const ContinuousCell& source, const ContinuousCell& dest) const;
    motion_primitive findPrimitive(int start_id, int dest_id) const;
    ContinuousCell applyPrimitive(const ContinuousCell& start, const motion_primitive& p) const;
    std::vector<CarSimulator::state_type> trajectoryFromIds(std::vector<int> ids,
                                                            unsigned int number_of_steps = 10) const;

protected:

    int addHashMapping(std::size_t hash_entry);
    int addIfRequired(const ContinuousCell &c);
    int findIdFromHash(std::size_t hash);


    std::vector<motion_primitive> primitives_;
    float simulation_time_step_;
    std::map<std::size_t, ContinuousCell> cells_map_;

    typedef boost::bimap<int, std::size_t> hash_int_map_t;
    hash_int_map_t hash_int_map_;


    float car_length_;
    float map_res_;
    float theta_bins_;
    float max_v_;
    float min_v_;
    float max_steer_;
    float min_steer_;

    std::size_t start_id_;
    std::size_t goal_id_;
    ContinuousCell* goal_cell_;
    ContinuousCell* start_cell_;
};

std::ostream& operator<<(std::ostream& stream, const EnvironmentCar& env) {
    stream<<"Car length: "<<env.car_length_<<" ";
    stream<<"Map resolution: "<<env.map_res_<<" ";
    stream<<"Theta bins: "<<env.theta_bins_<<" ";
    stream<<"Max velocity: "<<env.max_v_<<" ";
    stream<<"Min velocity: "<<env.min_v_<<" ";
    stream<<"Max steering angle: "<<env.max_steer_<<" ";
    stream<<"Min steering angle: "<<env.min_steer_<<" ";
}

#endif // ENVIRONMENT_CAR_H
