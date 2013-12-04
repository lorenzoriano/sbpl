#include "sbpl/utils/continuouscell.h"

ContinuousCell::ContinuousCell(scalar x, scalar y, scalar th,
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

ContinuousCell::ContinuousCell(const state_type& p,
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

int ContinuousCell::id() const {
    return id_;
}
void ContinuousCell::setId(int value) {
    id_ = value;
}

scalar ContinuousCell::x() const {
    return x_;
}
scalar ContinuousCell::y() const {
    return y_;
}
scalar ContinuousCell::th() const {
    return th_;
}
bool ContinuousCell::is_forward() const {
    return is_forward_;
}

void ContinuousCell::set_isForward(bool value) {
    is_forward_ = value;
}

std::size_t ContinuousCell::hash() const {

    if (hash_calculated_)
        return cached_hash_;

    using boost::hash_combine;
    std::size_t seed = 0;

    if (fixed_cells_) {
        hash_combine(seed, x_);
        hash_combine(seed, y_);
        hash_combine(seed, th_);
    }
    else {
        hash_combine(seed, discretize_coordinate(x_, map_res_));
        hash_combine(seed, discretize_coordinate(y_, map_res_));
        hash_combine(seed, discretize_angle(th_, theta_bins_));
    }
    hash_combine(seed, is_forward_);

    cached_hash_ = seed;
    hash_calculated_ = true;
    return seed;
}

std::string ContinuousCell::repr() const {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(2);
    ss<<x_<<" "<<y_<<" "<<th_;
    return ss.str();
}

ContinuousCell::state_type ContinuousCell::toCarState() const {
    state_type s;
    s[0] = x_;
    s[1] = y_;
    s[2] = th_;
    return s;
}

void ContinuousCell::addPredecessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c) {
    predecessors_[c->id()] = std::make_pair(p, c);
}

void ContinuousCell::addSuccessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c) {
    successors_[c->id()] = std::make_pair(p, c);
}

const ContinuousCell::prims_cells_t& ContinuousCell::getPredecessors() const {
    return predecessors_;
}

const ContinuousCell::prims_cells_t& ContinuousCell::getSuccessors() const {
    return successors_;
}

void ContinuousCell::checkHashCollision(const ContinuousCell& other) {
    if (hash() == other.hash()) {
        if (! (*this == other)) {

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

bool ContinuousCell::operator==(const ContinuousCell& other) const {
    return (( is_forward() == other.is_forward()) &&
            ( fabs(x()  -  other.x()) < map_res_) &&
            ( fabs(y()  -  other.y()) < map_res_) &&
            ( fabs(diff_angle(th(), other.th())) < (2*M_PI)/double(theta_bins_))
           );
}

bool ContinuousCell::equalButForward(const ContinuousCell& other) const {
    return (( fabs(x()  -  other.x()) < map_res_) &&
            ( fabs(y()  -  other.y()) < map_res_) &&
            ( fabs(diff_angle(th(), other.th())) < (2*M_PI)/double(theta_bins_))
           );
}
