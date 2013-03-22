#include <sbpl/discrete_space_information/environment_car.h>

#include <fstream>


bool EnvironmentCar::InitializeEnv(const char* sEnvFile) {

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
