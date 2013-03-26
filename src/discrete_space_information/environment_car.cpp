#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <yaml-cpp/yaml.h>
#include <fstream>


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
    max_steer_ = max_steer_;
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

bool EnvironmentCar::InitializeMDPCfg(MDPConfig *MDPCfg) {

}

int EnvironmentCar::GetFromToHeuristic(int FromStateID, int ToStateID) {

}

int EnvironmentCar::GetGoalHeuristic(int stateID) {

}

int EnvironmentCar::GetStartHeuristic(int stateID) {

}

void EnvironmentCar::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) {

    //get starting state
    std::map<std::size_t, ContinuousCell>::iterator i = cells_map_.find(SourceStateID);
    if (i == cells_map_.end()) {
        std::cerr<<"Error: the state with id "<<SourceStateID<<" does not exist!"<<std::endl;
        throw(SBPL_Exception());
    }

    SuccIDV->reserve(primitives_.size());
    CostV->reserve(primitives_.size());

    const ContinuousCell& start = i->second;
    Car sim(car_length_);
    sim.setInitialState(start.x(), start.y(), start.th());
    float initial_v = start.v();
    float initial_w = start.w();

    //now loop over all the primitives
    for (std::vector<motion_primitive>::iterator p = primitives_.begin(); p != primitives_.end(); p++) {
        float new_v = initial_v + (*p).dv;
        float new_th = initial_w + (*p).dth;
        sim.setControl(new_v, new_th);
        Car::return_type new_state = sim.simulate((*p).duration, simulation_time_step_);

        //TODO: check if the new state is feasible!

        ContinuousCell c(new_state[0],new_state[1],new_state[2],new_state[3],new_state[4],
                map_res_, theta_bins_, v_bins_, max_v_, min_v_);
        std::size_t c_hash = c.hash();

        //add the new state if it doesn't exist
        cells_map_.insert(std::make_pair(c_hash, c));

        //add the successor state
        SuccIDV->push_back(c_hash);
        CostV->push_back((*p).cost);
    }

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

    //DEBUG
    std::cout<<"Time step: "<<simulation_time_step_<<std::endl;
    for (std::vector<motion_primitive>::iterator i = primitives_.begin(); i != primitives_.end(); i++)
        std::cout<<*i<<std::endl;


}

