#ifndef ENVIRONMENT_CAR_H
#define ENVIRONMENT_CAR_H

#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/utils.h>
#include <sbpl/utils/car_simulator.h>

#include <boost/functional/hash.hpp>
#include <boost/bimap.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

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

/**
 * @brief Discretize an element into bins.
 *
 * @param x the number to discretize
 * @param numofbins the number of bins to use
 * @param max the maximum value for x
 * @param min the minimum value for x
 * @return the discretized value
 */
inline int continousToDiscBins(scalar  x, const int numofbins,
                                const scalar  max,
                                const scalar  min) {
    if ( x >max)
        x = max;
    else if (x < min)
        x = min;
    int discv = round((x - min) / (max - min) * (numofbins-1));
    return discv;
}

inline scalar  wrap_angle(scalar  angle) {
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
inline int bin_angle(scalar  x, int numofbins) {
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
inline scalar  continous_angle(int x, int numofbins) {
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
scalar  discretize_angle(scalar  angle, int numofbins) {
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
inline scalar  discretize_coordinate(scalar  x, scalar  map_resolution) {
    return round(x/map_resolution) * map_resolution;
}

/**
 * @brief Returns the absolute difference between two angles, taking into
 * consideration wrapping around.
 *
 * @param a1 first angle, in radians
 * @param a2 second angle, in radians
 * @return the difference between a1 and a2, in the interval [-pi and pi]
 */
inline scalar  diff_angle(scalar  a1, scalar  a2) {
     return M_PI - fabs(M_PI - fabs(a1 -a2));
}

/**
 * @brief This is a basic motion primitive, with velocity, steering angle, duration
 * and cost.
 */
struct motion_primitive {
    scalar  v;
    scalar  steer;
    scalar  duration;
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
    typedef std::map<int,
                     std::pair<motion_primitive,
                               boost::weak_ptr<const ContinuousCell> > >
            prims_cells_t;

    ContinuousCell(scalar x, scalar y, scalar th,
                   bool is_forward,
                   scalar map_res,
                   int theta_bins,
                   bool fixed_cells
                   ) {

        fixed_cells_ = fixed_cells;

        if (fixed_cells_) {
            x_ = discretize_coordinate(x, map_res);
            y_ = discretize_coordinate(y, map_res);
            th_ = discretize_angle(th, theta_bins);
        }
        else {
            x_ = x;
            y_ = y;
            th_ = th;
        }

        is_forward_ = is_forward;

        map_res_ = map_res;
        theta_bins_ = theta_bins;
        cached_hash_ = 0;
        hash_calculated_ = false;
        id_ = -1;
    }

    ContinuousCell(const CarSimulator::state_type& p,
                   bool is_forward,
                   scalar map_res, int theta_bins,
                   bool fixed_cells) {

        fixed_cells_ = fixed_cells;
        if (fixed_cells_) {
            x_ = discretize_coordinate(p[0], map_res);
            y_ = discretize_coordinate(p[1], map_res);
            th_ = discretize_angle(p[2], theta_bins);
        }
        else {
            x_ = p[0];
            y_ = p[1];
            th_ = p[2];
        }

        is_forward_ = is_forward;
        map_res_ = map_res;
        theta_bins_ = theta_bins;
        cached_hash_ = 0;
        hash_calculated_ = false;
        id_ = -1;
    }

    int id() const {
        return id_;
    }
    void setId(int value) {
        id_ = value;
    }

    scalar x() const {
        return x_;
    }
    scalar y() const {
        return y_;
    }
    scalar th() const {
        return th_;
    }
    bool is_forward() const {
        return is_forward_;
    }

    std::size_t hash() const {

        if (hash_calculated_)
            return cached_hash_;

        using boost::hash_combine;
        std::size_t seed = 0;

        if (fixed_cells_) {
            int val = round(x_ / map_res_);
            hash_combine(seed, val);
            val = round(y_ / map_res_);
            hash_combine(seed, val);
            val = bin_angle(th_, theta_bins_);
            hash_combine(seed, val);
        }
        else {
            hash_combine(seed, int(discretize_coordinate(x_, map_res_)/ map_res_));
            hash_combine(seed, int(discretize_coordinate(y_, map_res_)/ map_res_));
            hash_combine(seed, bin_angle(th_, theta_bins_));
        }
        hash_combine(seed, int(is_forward_));

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


    void addPredecessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c) {
        predecessors_[c->id()] = std::make_pair(p, c);
    }

    void addSuccessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c) {
        successors_[c->id()] = std::make_pair(p, c);
    }

    const prims_cells_t& getPredecessors() const {
        return predecessors_;
    }

    const prims_cells_t& getSuccessors() const {
        return successors_;
    }

    void checkHashCollision(const ContinuousCell& other) {
        if (hash() == other.hash()) {
            if (( fabs(x()  -  other.x()) > map_res_) ||
                ( fabs(y()  -  other.y()) > map_res_) ||
                ( fabs(diff_angle(th(), other.th())) > double(theta_bins_)/(2*M_PI)) ||
                ( is_forward() != other.is_forward())
               ) {

                {
                    this->hash_calculated_ = false;
                    this->hash();
                    other.hash_calculated_ = false;
                    other.hash();
                }

                std::stringstream msg;
                msg<<"Collision!!!"<<std::endl;
                msg<<"C1: "<<x()<<", "<<y()<<", "<<th()<<", "<<is_forward()<<std::endl;
                msg<<"C2: "<<other.x()<<", "<<other.y()<<", "<<other.th()<<", "<<other.is_forward()<<std::endl;
                msg<<"Hashes: "<<hash()<<" -- "<<other.hash();
                throw(std::logic_error(msg.str()));
            }
        }
    }

private:
    scalar x_, y_, th_;
    bool is_forward_;
    scalar map_res_;
    scalar theta_bins_;
    bool fixed_cells_;
    mutable bool hash_calculated_;
    mutable std::size_t cached_hash_;
    int id_;

    prims_cells_t predecessors_;
    prims_cells_t successors_;

};

typedef boost::shared_ptr<ContinuousCell> ContinuousCellPtr;
typedef boost::weak_ptr<const ContinuousCell> ConstContinuousCellWeakPtr;
typedef boost::weak_ptr<ContinuousCell> ContinuousCellWeakPtr;

std::ostream& operator<<(std::ostream& stream, const ContinuousCell& cell) {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(2);
    ss<<cell.x_<<" "<<cell.y_<<" "<<cell.th_<<" "<<-cell.is_forward_;
    stream<<ss.str();
    return stream;
}

class EnvironmentCar : public DiscreteSpaceInformation {

public:
    EnvironmentCar(scalar map_res, scalar car_length,
                   bool fixed_cells,
                   int theta_bins,
                   scalar max_v,
                   scalar min_v,
                   scalar max_steer,
                   scalar min_steer);

    EnvironmentCar(const char *cfg_file);

    void setGoal(scalar x, scalar y, scalar th);
    void setStart(scalar x, scalar y, scalar th);

    bool isValidCell(const ContinuousCellPtr& c);

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
        assert(cells_map_.size() == hash_int_map_.size());
        return cells_map_.size();
    }

//    motion_primitive findPrimitive(const ContinuousCellPtr& source, const ContinuousCellPtr& dest) const;
//    motion_primitive findPrimitive(int start_id, int dest_id) const;
//    ContinuousCellPtr applyPrimitive(const ContinuousCellPtr& start, const motion_primitive& p) const;

    bool saveSolutionYAML(const std::vector<int>& ids, const char* filename) const;

protected:

    int addHashMapping(std::size_t hash_entry);
    ContinuousCellPtr addIfRequired(ContinuousCellPtr c);


    std::vector<motion_primitive> primitives_;
    scalar simulation_time_step_;
    std::map<std::size_t, ContinuousCellPtr> cells_map_;

    typedef boost::bimap<int, std::size_t> hash_int_map_t;
    hash_int_map_t hash_int_map_;


    scalar car_length_;
    scalar map_res_;
    scalar theta_bins_;
    scalar  max_v_;
    scalar  min_v_;
    scalar  max_steer_;
    scalar  min_steer_;
    bool fixed_cells_;

    std::size_t start_id_;
    std::size_t goal_id_;
    ContinuousCellWeakPtr goal_cell_;
    ContinuousCellWeakPtr start_cell_;
};

std::ostream& operator<<(std::ostream& stream, const EnvironmentCar& env) {
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
