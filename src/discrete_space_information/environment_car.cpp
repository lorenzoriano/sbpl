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
                               float max_v,
                               float min_v,
                               float max_steer,
                               float min_steer) {

    map_res_ = map_res;
    theta_bins_ = theta_bins;
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
    if (! fin.good()) {
        std::string msg = "Error while opening the file ";
        msg += sEnvFile;
        throw CarException(msg);
    }
    YAML::Parser parser(fin);

    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["map_resolution"] >> map_res_;
    doc["car_length"] >> car_length_;
    doc["theta_bins"] >> theta_bins_;
    doc["max_v"] >> max_v_;
    doc["min_v"] >> min_v_;
    doc["max_steering_angle"] >> max_steer_;
    doc["min_steering_angle"] >> min_steer_;
}

void EnvironmentCar::setGoal(float x, float y, float th) {

    ContinuousCell c(x, y, th,
            map_res_, theta_bins_);
    std::size_t c_hash = c.hash();

    //add the new state if it doesn't exist
    goal_id_ = c_hash;
    SBPL_DEBUG("Setting goal %s with hash %zu\n", c.repr().c_str(), c_hash);
    int id = addIfRequired(c);
    goal_cell_ = &findCell(id);

}

void EnvironmentCar::setStart(float x, float y, float th) {

    ContinuousCell c(x, y, th,
            map_res_, theta_bins_);
    std::size_t c_hash = c.hash();

    //add the new state if it doesn't exist
    start_id_ = c_hash;
    SBPL_DEBUG("Setting start %s with hash %zu\n", c.repr().c_str(), c_hash);
    int id = addIfRequired(c);
    start_cell_ = &findCell(id);
}

bool EnvironmentCar::isValidCell(const ContinuousCell& c) {

    return true;
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

    int heuristic = int(round(dist / map_res_));

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
        std::ostringstream ss;
        ss<<"Error: the state with id "<<state_id<<" does not exist!";
        throw(SBPL_Exception());
    }

    //get cell
    std::map<std::size_t, ContinuousCell>::const_iterator i = cells_map_.find(hash_i->second);
    if (i == cells_map_.end()) {
        std::ostringstream ss;
        ss<<"Error: the state with hash id "<<hash_i->first<<" does not exist!";
        throw(CarException(ss.str()));
    }
    SBPL_DEBUG("State id %d has been found with hash %zu\n", state_id, i->second.hash());
    return i->second;
}

ContinuousCell& EnvironmentCar::findCell(int state_id){
    //get hash
    hash_int_map_t::left_map::iterator hash_i = hash_int_map_.left.find(state_id);
    if (hash_i == hash_int_map_.left.end()) {
        std::ostringstream ss;
        ss<<"Error: the state with id "<<state_id<<" does not exist!"<<std::endl;
        throw(CarException(ss.str()));
    }

    //get cell
    std::map<std::size_t, ContinuousCell>::iterator i = cells_map_.find(hash_i->second);
    if (i == cells_map_.end()) {
        std::ostringstream ss;
        ss<<"Error: the state with hash id "<<hash_i->first<<" does not exist!";
        throw (CarException(ss.str()));
    }
    SBPL_DEBUG("State id %d has been found with hash %zu\n", state_id, i->second.hash());
    return i->second;
}

int EnvironmentCar::findIdFromHash(std::size_t hash) {
    hash_int_map_t::right_map::iterator i = hash_int_map_.right.find(hash);
    if (i == hash_int_map_.right.end()) {//element already exist
        std::ostringstream ss;
        ss<<"Cell with hash "<<hash<<" does not exists";
        throw (CarException(ss.str()));
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

    CarSimulator sim(car_length_);
    sim.setInitialState(start.x(), start.y(), start.th());

    //now loop over all the primitives
    for (std::vector<motion_primitive>::iterator p = primitives_.begin(); p != primitives_.end(); p++) {
        float new_v = (*p).v;
        float new_steer = (*p).steer;
        sim.setControl(new_v, new_steer);
        CarSimulator::state_type new_state = sim.simulate((*p).duration, simulation_time_step_);

        ContinuousCell c(new_state[0],new_state[1],new_state[2],
                map_res_, theta_bins_);

        SBPL_DEBUG("Motion primitive: dv: %f, dth: %fm cost: %d\n", (*p).v, (*p).steer, (*p).cost);
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
    if (! fin.good()) {
        std::string msg = "Error while opening the file ";
        msg += filename;
        throw CarException(msg);
    }
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    doc["time_step"] >> simulation_time_step_;

    //primitives
    const YAML::Node& primitives_node = doc["motions"];
    for(YAML::Iterator it=primitives_node.begin(); it!=primitives_node.end();++it) {
        motion_primitive p;
        (*it)["v"] >> p.v;
        (*it)["steer"] >> p.steer;
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

motion_primitive EnvironmentCar::findPrimitive(const ContinuousCell& source, const ContinuousCell& dest) const{
    for (std::vector<motion_primitive>::const_iterator p = primitives_.begin(); p != primitives_.end(); p++) {
        if (applyPrimitive(source, *p) == dest)
            return *p;
    }
    //when reaching this position, no primitive has been found
    std::ostringstream ss;
    ss<<"No primitive found to go from cell "<<source<<" to cell "<<dest;
    throw CarException(ss.str());
}

motion_primitive EnvironmentCar::findPrimitive(int start_id, int dest_id) const{
    const ContinuousCell& start = findCell(start_id);
    const ContinuousCell& dest = findCell(dest_id);
    return findPrimitive(start, dest);
}

ContinuousCell EnvironmentCar::applyPrimitive(const ContinuousCell& start, const motion_primitive& p) const {
    CarSimulator sim(car_length_);
    sim.setInitialState(start.x(), start.y(), start.th());
    sim.setControl(p.v, p.steer);
    CarSimulator::state_type new_state = sim.simulate(p.duration, simulation_time_step_);

    ContinuousCell c(new_state[0],new_state[1],new_state[2],
            map_res_, theta_bins_);
    return c;
}


std::vector<CarSimulator::state_type> EnvironmentCar::trajectoryFromIds(std::vector<int> ids,
                                                unsigned int number_of_steps) const {
    std::vector<CarSimulator::state_type> res;

    //initializing the simulator with the first position
    const ContinuousCell& start = findCell(ids[0]);
    CarSimulator::state_type prev_position = start.toCarState();
    res.push_back(prev_position);

    CarSimulator sim(car_length_);
    sim.setInitialState(prev_position);

    for (unsigned int i=1; i<ids.size(); i++ ){
        const ContinuousCell& start = findCell(ids[i-1]);
        const ContinuousCell& dest = findCell(ids[i]);
        motion_primitive p = findPrimitive(start, dest);
        std::cout<<"Starting from cell "<<start<<std::endl;
        std::cout<<"Applying primitive "<<p<<std::endl;
        sim.setControl(p.v, p.steer);

        //doing smaller intermediate steps
        float dt = p.duration / number_of_steps;
        for (unsigned int t=0; t<number_of_steps; t++) {

//            CarSimulator::state_type new_state = sim.simulate(dt, dt/10.);
//            //discretizing the state
//            prev_position = ContinuousCell(new_state, map_res_, theta_bins_).toCarState();

            prev_position = sim.simulate(dt, dt/10.);
            res.push_back(prev_position);
            sim.setInitialState(prev_position);
        }
        float dx = fabs(prev_position[0] - dest.x());
        float dy = fabs(prev_position[1] - dest.y());
        float dth = diff_angle(prev_position[2], dest.th());
        std::cout<<"Expected cell "<<dest<<std::endl;
        std::cout<<"Obtained: "<<prev_position<<std::endl;
        std::cout<<"Error: "<<dx<<" "<<dy<<" "<<dth<<std::endl<<std::endl;

    }


    return res;
}
