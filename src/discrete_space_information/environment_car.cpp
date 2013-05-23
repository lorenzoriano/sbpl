#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/planners/planner.h>
#include <sbpl/config.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

#include <boost/make_shared.hpp>

template< class MapType >
void print_map(const MapType & m)
{
    typedef typename MapType::const_iterator const_iterator;
    for( const_iterator iter = m.begin(), iend = m.end(); iter != iend; ++iter )
    {
        SBPL_DEBUG("%d -> %zu\n",iter->first, iter->second);
    }
}

EnvironmentCar::EnvironmentCar(scalar map_res,
                               scalar car_length, bool fixed_cells,
                               int theta_bins,
                               scalar max_v,
                               scalar min_v,
                               scalar max_steer,
                               scalar min_steer) {

    map_res_ = map_res;
    theta_bins_ = theta_bins;
    max_v_ = max_v;
    min_v_ = min_v;
    car_length_ = car_length;
    min_steer_ = min_steer;
    max_steer_ = max_steer;
    fixed_cells_ = fixed_cells;
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
    doc["fixed_cells"] >> fixed_cells_;
}

void EnvironmentCar::setGoal(scalar x, scalar y, scalar th) {

    bool is_forward = true;

    ContinuousCellPtr c(new ContinuousCell(x, y, th, is_forward,
            map_res_, theta_bins_, fixed_cells_));
    std::size_t c_hash = c->hash();

    //add the new state if it doesn't exist
    goal_id_ = c_hash;
    SBPL_DEBUG("Setting goal %s with hash %zu\n", c->repr().c_str(), c_hash);
    goal_cell_ = addIfRequired(c);
}

void EnvironmentCar::setStart(scalar x, scalar y, scalar th) {

    bool is_forward = true;
    ContinuousCellPtr c = boost::make_shared<ContinuousCell>(x, y, th, is_forward,
            map_res_, theta_bins_, fixed_cells_);
    std::size_t c_hash = c->hash();

    //add the new state if it doesn't exist
    start_id_ = c_hash;
    SBPL_DEBUG("Setting start %s with hash %zu\n", c->repr().c_str(), c_hash);
    start_cell_ = addIfRequired(c);
}

bool EnvironmentCar::isValidCell(const ContinuousCellPtr &c) {

    return true;
}

bool EnvironmentCar::InitializeMDPCfg(MDPConfig *MDPCfg) {    
    MDPCfg->startstateid = start_cell_.lock()->id();
    MDPCfg->goalstateid = goal_cell_.lock()->id();
}

int EnvironmentCar::GetFromToHeuristic(int FromStateID, int ToStateID) {
    throw(SBPL_Exception());
    return 0;

}

int EnvironmentCar::GetGoalHeuristic(int stateID) {
    ContinuousCellPtr start = findCell(stateID);

    scalar dx = start->x() - goal_cell_.lock()->x();
    scalar dy = start->y() - goal_cell_.lock()->y();


    //manhattan distance
//    scalar dist = fabs(dx) + fabs(dy);

    //euclidean distance
    scalar dist = sqrt(dx*dx + dy*dy);

    int heuristic = int(round(dist / map_res_));

    return heuristic;
//    return 0;
}

int EnvironmentCar::GetStartHeuristic(int stateID) {
    throw(SBPL_Exception());
    return 0;
}

ContinuousCellPtr EnvironmentCar::findCell(int state_id) const {
    //get hash
    hash_int_map_t::left_map::const_iterator hash_i = hash_int_map_.left.find(state_id);
    if (hash_i == hash_int_map_.left.end()) {
        std::ostringstream ss;
        ss<<"Error: the state with id "<<state_id<<" does not exist!";
        throw(SBPL_Exception());
    }

    //get cell
    std::map<std::size_t, ContinuousCellPtr>::const_iterator i = cells_map_.find(hash_i->second);
    if (i == cells_map_.end()) {
        std::ostringstream ss;
        ss<<"Error: the state with hash id "<<hash_i->first<<" does not exist!";
        throw(CarException(ss.str()));
    }
    SBPL_DEBUG("State id %d has been found with hash %zu\n", state_id, i->second.hash());
    return i->second;
}

ContinuousCellPtr EnvironmentCar::addIfRequired(ContinuousCellPtr c) {
    std::size_t hash = c->hash();
    //get starting state
    hash_int_map_t::right_map::iterator i = hash_int_map_.right.find(hash);
    if (i != hash_int_map_.right.end()) {//element already exist
        SBPL_DEBUG("Cell with hash %zu already exists with id %d, not adding\n", hash, i->second);

        //THIS IS SLOW AND HAS TO BE FIXED!!
        cells_map_[hash]->checkHashCollision(*c.get());

        return cells_map_[hash];
    }

    //a new entry needs to be added
    if (!cells_map_.insert(std::make_pair(hash, c)).second) {
        //this should never have happened
        SBPL_ERROR("The hash value %zu is already in the map!\n", hash);
        throw(SBPL_Exception());
    }

    int new_id = addHashMapping(hash);
    c->setId(new_id);
    return c;
}

void EnvironmentCar::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) {

    ContinuousCellPtr start = findCell(SourceStateID);
    SBPL_DEBUG("\nGetting successors of cell %s\n", start.repr().c_str());

    SuccIDV->reserve(primitives_.size());
    CostV->reserve(primitives_.size());

    CarSimulator sim(car_length_);
    sim.setInitialState(start->x(), start->y(), start->th());

    //now loop over all the primitives
    for (std::vector<motion_primitive>::iterator p = primitives_.begin(); p != primitives_.end(); p++) {
        scalar new_v = (*p).v;
        scalar new_steer = (*p).steer;
        sim.setControl(new_v, new_steer);
        CarSimulator::state_type new_state = sim.simulate((*p).duration, simulation_time_step_);

        bool is_forward = new_v >= 0;
        ContinuousCellPtr c = boost::make_shared<ContinuousCell>(new_state, is_forward,
                map_res_, theta_bins_, fixed_cells_);

        SBPL_DEBUG("Motion primitive: dv: %f, dth: %fm cost: %d\n", (*p).v, (*p).steer, (*p).cost);
        SBPL_DEBUG("Successor cell %s\n", c.repr().c_str());

        //check if cell is reachable
        if (!isValidCell(c))
            continue;

        //add the new state if it doesn't exist
        c = addIfRequired(c);

        //add the successor state
        SuccIDV->push_back(c->id());
        CostV->push_back((*p).cost);

        //creating the links in the cells
        start->addSuccessor(*p, c);
        c->addPredecessor(*p, start);

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
    int new_id = hash_int_map_.size();
    SBPL_DEBUG("Cell with hash id %zu will have int id %d\n", hash_entry, new_id);
    hash_int_map_.insert(hash_int_map_t::value_type(new_id, hash_entry));

    //insert and initialize the mappings
    //still obscure code from original SBPL
    int* entry = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(entry);
    for (unsigned int i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[new_id][i] = -1;
    }

    if (new_id != (int)StateID2IndexMapping.size() - 1) {
        SBPL_ERROR("ERROR in Env... function: last state has incorrect stateID\n");
        throw new SBPL_Exception();
    }

    SBPL_DEBUG("After addHashMapping the map has entries:\n");
    print_map(hash_int_map_.left);
//    std::cout<<std::endl;

    return new_id;
}

//motion_primitive EnvironmentCar::findPrimitive(const ContinuousCellPtr& source, const ContinuousCellPtr& dest) const{
//    for (std::vector<motion_primitive>::const_iterator p = primitives_.begin(); p != primitives_.end(); p++) {
//        if (applyPrimitive(source, *p).get() == dest.get())
//            return *p;
//    }
//    //when reaching this position, no primitive has been found
//    std::ostringstream ss;
//    ss<<"No primitive found to go from cell "<<source<<" to cell "<<dest;
//    throw CarException(ss.str());
//}

//motion_primitive EnvironmentCar::findPrimitive(int start_id, int dest_id) const{
//    const ContinuousCellPtr& start = findCell(start_id);
//    const ContinuousCellPtr& dest = findCell(dest_id);
//    return findPrimitive(start, dest);
//}

//ContinuousCellPtr EnvironmentCar::applyPrimitive(const ContinuousCellPtr& start, const motion_primitive& p) const {
//    CarSimulator sim(car_length_);
//    sim.setInitialState(start->x(), start->y(), start->th());
//    sim.setControl(p.v, p.steer);
//    CarSimulator::state_type new_state = sim.simulate(p.duration, simulation_time_step_);

//    bool is_forward = p.v >= 0;
//    ContinuousCellPtr c( new ContinuousCell(new_state, is_forward,
//            map_res_, theta_bins_, fixed_cells_));
//    return c;
//}

bool EnvironmentCar::saveSolutionYAML(const std::vector<int>& ids, const char* filename) const {
    std::ofstream fout(filename);
    if (! fout.good()) {
        std::string msg = "Error while opening the file ";
        msg += filename;
        throw CarException(msg);
    }

    YAML::Emitter doc;

    //all the cells
    doc<<YAML::BeginMap;
    {
        doc<<YAML::Key<<"nodes";
        doc<<YAML::Value;
        doc<<YAML::BeginMap;
        {
            for (std::vector<int>::const_iterator id = ids.begin(); id != ids.end(); id++) {

                //NODE INFO
                ContinuousCellPtr c = findCell(*id);
                doc<<YAML::Key<<*id;
                doc<<YAML::Value;
                doc<<YAML::BeginMap;
                {
                    doc<<YAML::Key<<"x";
                    doc<<YAML::Value<<c->x();
                    doc<<YAML::Key<<"y";
                    doc<<YAML::Value<<c->y();
                    doc<<YAML::Key<<"th";
                    doc<<YAML::Value<<c->th();
                    doc<<YAML::Key<<"is_forward";
                    doc<<YAML::Value<<c->is_forward();
                    doc<<YAML::Key<<"id";
                    doc<<YAML::Value<<c->id();
                    doc<<YAML::Key<<"hash";
                    doc<<YAML::Value<<c->hash();

                    //OUTGOING LINKS
                    doc<<YAML::Key<<"out_links";
                    doc<<YAML::Value;
                    doc<<YAML::BeginMap;
                    {
                        const ContinuousCell::prims_cells_t& succs = c->getSuccessors();
                        for (ContinuousCell::prims_cells_t::const_iterator i =  succs.begin(); i != succs.end(); i++) {

                            ConstContinuousCellWeakPtr next_cell = i->second.second;
                            const motion_primitive& p = i->second.first;

                            doc<<YAML::Key<<next_cell.lock()->id();
                            doc<<YAML::Value;

                            doc<<YAML::BeginMap;
                            {
                                doc<<YAML::Key<<"from";
                                doc<<YAML::Value<<c->id();
                                doc<<YAML::Key<<"to";
                                doc<<YAML::Value<<next_cell.lock()->id();

                                doc<<YAML::Key<<"primitive";
                                doc<<YAML::Value;
                                doc<<YAML::BeginMap;
                                {
                                    doc<<YAML::Key<<"v";
                                    doc<<YAML::Value<<p.v;
                                    doc<<YAML::Key<<"steer";
                                    doc<<YAML::Value<<p.steer;
                                    doc<<YAML::Key<<"duration";
                                    doc<<YAML::Value<<p.duration;
                                }
                                doc<<YAML::EndMap;
                            }
                            doc<<YAML::EndMap;
                        }
                    }
                    doc<<YAML::EndMap;

                    //INCOMING LINKS
                    doc<<YAML::Key<<"in_links";
                    doc<<YAML::Value;
                    doc<<YAML::BeginMap;
                    {
                        const ContinuousCell::prims_cells_t& preds = c->getPredecessors();
                        for (ContinuousCell::prims_cells_t::const_iterator i =  preds.begin(); i != preds.end(); i++) {

                            ConstContinuousCellWeakPtr next_cell = i->second.second;
                            const motion_primitive& p = i->second.first;

                            doc<<YAML::Key<<next_cell.lock()->id();
                            doc<<YAML::Value;

                            doc<<YAML::BeginMap;
                            {
                                doc<<YAML::Key<<"from";
                                doc<<YAML::Value<<next_cell.lock()->id();
                                doc<<YAML::Key<<"to";
                                doc<<YAML::Value<<c->id();

                                doc<<YAML::Key<<"primitive";
                                doc<<YAML::Value;
                                doc<<YAML::BeginMap;
                                {
                                    doc<<YAML::Key<<"v";
                                    doc<<YAML::Value<<p.v;
                                    doc<<YAML::Key<<"steer";
                                    doc<<YAML::Value<<p.steer;
                                    doc<<YAML::Key<<"duration";
                                    doc<<YAML::Value<<p.duration;
                                }
                                doc<<YAML::EndMap;
                            }
                            doc<<YAML::EndMap;
                        }
                    }
                    doc<<YAML::EndMap;
                }
                doc<<YAML::EndMap;
            }
        }
        doc<<YAML::EndMap;
    }
    doc<<YAML::EndMap;
    doc<<YAML::EndDoc;

    fout<<doc.c_str();
}
