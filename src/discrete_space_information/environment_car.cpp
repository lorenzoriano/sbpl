#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/planners/planner.h>
#include <sbpl/config.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

template< class MapType >
void print_map(const MapType & m)
{
    typedef typename MapType::const_iterator const_iterator;
    for( const_iterator iter = m.begin(), iend = m.end(); iter != iend; ++iter )
    {
        SBPL_DEBUG("%d -> %zu\n",iter->first, iter->second);
    }
}

EnvironmentCar::EnvironmentCar(float map_res,
                               float car_length,
                               int theta_bins,
                               int v_bins,
                               float max_v,
                               float min_v,
                               float max_steer,
                               float min_steer) {

    map_res_ = map_res;
    theta_bins_ = theta_bins;
    v_bins_ = v_bins;
    w_bins_ = theta_bins;
    max_v_ = max_v;
    min_v_ = min_v;
    car_length_ = car_length;
    min_steer_ = min_steer;
    max_steer_ = max_steer;
}

EnvironmentCar::EnvironmentCar(const char* cfg_file) {
    InitializeEnv(cfg_file);

}

bool EnvironmentCar::InitializeEnv(const char* sEnvFile) {
    std::ifstream fin(sEnvFile);
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["map_resolution"] >> map_res_;
    doc["car_length"] >> car_length_;
    doc["theta_bins"] >> theta_bins_;
    doc["v_bins"] >> v_bins_;
    doc["w_bins"] >> w_bins_;
    doc["max_v"] >> max_v_;
    doc["min_v"] >> min_v_;
    doc["max_steering_angle"] >> max_steer_;
    doc["min_steering_angle"] >> min_steer_;
}

void EnvironmentCar::setGoal(float x, float y, float th, float v, float steering_angle) {

    ContinuousCell c(x, y, th, v,steering_angle,
            map_res_, theta_bins_, v_bins_, w_bins_, max_v_, min_v_, max_steer_, min_steer_);
    std::size_t c_hash = c.hash();

    //add the new state if it doesn't exist
    goal_id_ = c_hash;
    SBPL_DEBUG("Setting goal %s with hash %zu\n", c.repr().c_str(), c_hash);
    int id = addIfRequired(c);
    goal_cell_ = &findCell(id);

}

void EnvironmentCar::setStart(float x, float y, float th, float v, float steering_angle) {

    ContinuousCell c(x, y, th, v,steering_angle,
            map_res_, theta_bins_, v_bins_,  w_bins_, max_v_, min_v_, max_steer_, min_steer_);
    std::size_t c_hash = c.hash();

    //add the new state if it doesn't exist
    start_id_ = c_hash;
    SBPL_DEBUG("Setting start %s with hash %zu\n", c.repr().c_str(), c_hash);
    int id = addIfRequired(c);
    start_cell_ = &findCell(id);
}

bool EnvironmentCar::isValidCell(const ContinuousCell& c) {

    return (c.v() <= max_v_ &&
            c.v() >= min_v_ &&
            c.steering() <= max_steer_ &&
            c.steering() >= min_steer_);

//    return true;
}

bool EnvironmentCar::InitializeMDPCfg(MDPConfig *MDPCfg) {    
    MDPCfg->startstateid = findIdFromHash(start_id_);
    MDPCfg->goalstateid = findIdFromHash(goal_id_);
}

int EnvironmentCar::GetFromToHeuristic(int FromStateID, int ToStateID) {
    throw(SBPL_Exception());
    return 0;

}

int EnvironmentCar::GetGoalHeuristic(int stateID) {
    const ContinuousCell& start = findCell(stateID);

    float dx = start.x() - goal_cell_->x();
    float dy = start.y() - goal_cell_->y();


    //manhattan distance
//    float dist = fabs(dx) + fabs(dy);

    //euclidean distance
    float dist = sqrt(dx*dx + dy*dy);

    int heuristic = int(floor(dist / map_res_));

    return heuristic;
//    return 0;
}

int EnvironmentCar::GetStartHeuristic(int stateID) {
    throw(SBPL_Exception());
    return 0;
}

const ContinuousCell& EnvironmentCar::findCell(int state_id) const {
    //get hash
    hash_int_map_t::left_map::const_iterator hash_i = hash_int_map_.left.find(state_id);
    if (hash_i == hash_int_map_.left.end()) {
        std::cerr<<"Error: the state with id "<<state_id<<" does not exist!"<<std::endl;        
        throw(SBPL_Exception());
    }

    //get cell
    std::map<std::size_t, ContinuousCell>::const_iterator i = cells_map_.find(hash_i->second);
    if (i == cells_map_.end()) {
        std::cerr<<"Error: the state with hash id "<<hash_i->first<<" does not exist!"<<std::endl;
        throw(SBPL_Exception());
    }
    SBPL_DEBUG("State id %d has been found with hash %zu\n", state_id, i->second.hash());
    return i->second;
}

ContinuousCell& EnvironmentCar::findCell(int state_id){
    //get hash
    hash_int_map_t::left_map::iterator hash_i = hash_int_map_.left.find(state_id);
    if (hash_i == hash_int_map_.left.end()) {
        std::cerr<<"Error: the state with id "<<state_id<<" does not exist!"<<std::endl;
        throw(SBPL_Exception());
    }

    //get cell
    std::map<std::size_t, ContinuousCell>::iterator i = cells_map_.find(hash_i->second);
    if (i == cells_map_.end()) {
        std::cerr<<"Error: the state with hash id "<<hash_i->first<<" does not exist!"<<std::endl;
        throw(SBPL_Exception());
    }
    SBPL_DEBUG("State id %d has been found with hash %zu\n", state_id, i->second.hash());
    return i->second;
}

int EnvironmentCar::findIdFromHash(std::size_t hash) {
    hash_int_map_t::right_map::iterator i = hash_int_map_.right.find(hash);
    if (i == hash_int_map_.right.end()) {//element already exist
        SBPL_ERROR("Cell with hash %zu does not exists\n", hash);
        throw (SBPL_Exception());
    }
    SBPL_DEBUG("Cell with hash %zu has id %d\n", hash, i->second);
    return i->second;
}

int EnvironmentCar::addIfRequired(const ContinuousCell& c) {
    std::size_t hash = c.hash();
    //get starting state
    hash_int_map_t::right_map::iterator i = hash_int_map_.right.find(hash);
    if (i != hash_int_map_.right.end()) {//element already exist
        SBPL_DEBUG("Cell with hash %zu already exists with id %d, not adding\n", hash, i->second);
        return i->second;
    }

    //DEBUG!!!
//    std::cout<<c.repr()<<"\n";

    //a new entry needs to be added
    if (!cells_map_.insert(std::make_pair(hash, c)).second) {
        //this should never have happened
        SBPL_ERROR("The hash value %zu is already in the map!\n", hash);
        throw(SBPL_Exception());
    }
//    SBPL_DEBUG("Adding a new cell with hash %zu\n", hash);
    return addHashMapping(hash);

}

void EnvironmentCar::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) {

    const ContinuousCell& start = findCell(SourceStateID);
    SBPL_DEBUG("\nGetting successors of cell %s\n", start.repr().c_str());

    SuccIDV->reserve(primitives_.size());
    CostV->reserve(primitives_.size());

    Car sim(car_length_);
    sim.setInitialState(start.x(), start.y(), start.th());
    float initial_v = start.v();
    float initial_w = start.steering();

    //now loop over all the primitives
    for (std::vector<motion_primitive>::iterator p = primitives_.begin(); p != primitives_.end(); p++) {
        float new_v = initial_v + (*p).dv;
        float new_th = initial_w + (*p).dth;
        sim.setControl(new_v, new_th);
        Car::return_type new_state = sim.simulate((*p).duration, simulation_time_step_);

        ContinuousCell c(new_state[0],new_state[1],new_state[2],new_state[3],new_state[4],
                map_res_, theta_bins_, v_bins_, w_bins_, max_v_, min_v_, max_steer_, min_steer_);

        SBPL_DEBUG("Motion primitive: dv: %f, dth: %fm cost: %d\n", (*p).dv, (*p).dth, (*p).cost);
        SBPL_DEBUG("Successor cell %s\n", c.repr().c_str());

        //check if cell is reachable
        if (!isValidCell(c))
            continue;

        //add the new state if it doesn't exist
        int c_id = addIfRequired(c);

        //add the successor state
        SuccIDV->push_back(c_id);
        CostV->push_back((*p).cost);

    }
    SBPL_DEBUG("Finished finding the successors\n\n");
}

void EnvironmentCar::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) {

}

void EnvironmentCar::SetAllActionsandAllOutcomes(CMDPSTATE* state) {

}

void EnvironmentCar::SetAllPreds(CMDPSTATE* state) {

}

int EnvironmentCar::SizeofCreatedEnv() {

}

void EnvironmentCar::PrintState(int stateID, bool bVerbose, FILE* fOut) {

}

void EnvironmentCar::PrintEnv_Config(FILE* fOut) {

}

bool EnvironmentCar::loadPrimitives(const char* filename) {
    std::ifstream fin(filename);
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["time_step"] >> simulation_time_step_;

    //primitives
    const YAML::Node& primitives_node = doc["motions"];
    for(YAML::Iterator it=primitives_node.begin(); it!=primitives_node.end();++it) {
        motion_primitive p;
        (*it)["ds"] >> p.dv;
        (*it)["dth"] >> p.dth;
        (*it)["cost"] >> p.cost;
        (*it)["duration"] >> p.duration;
        primitives_.push_back(p);
    }

}

int EnvironmentCar::addHashMapping(std::size_t hash_entry) {

    //get the next entry
    int int_entry = hash_int_map_.size();
    SBPL_DEBUG("Cell with hash id %zu will have int id %d\n", hash_entry, int_entry);
    hash_int_map_.insert(hash_int_map_t::value_type(int_entry, hash_entry));

    //insert and initialize the mappings
    //still obscure code from original SBPL
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (unsigned int i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[int_entry][i] = -1;
    }

    if (int_entry != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

    SBPL_DEBUG("After addHashMapping the map has entries:\n");
    print_map(hash_int_map_.left);
//    std::cout<<std::endl;

    return int_entry;
}

