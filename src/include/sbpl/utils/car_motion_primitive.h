#ifndef CAR_MOTION_PRIMITIVE
#define CAR_MOTION_PRIMITIVE

#include <sstream>
#include <iomanip>

typedef double scalar;

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


inline std::ostream& operator<<(std::ostream& stream, const motion_primitive& p) {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(2);
    ss<<"v: "<<p.v<<" steer: "<<p.steer<<" cost: "<<p.cost<<" duration: "<<p.duration;
    stream<<ss.str();
    return stream;
}

#endif
