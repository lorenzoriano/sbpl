#ifndef CONTINUOUSCELL_H
#define CONTINUOUSCELL_H

#include <sbpl/utils/car_motion_primitive.h>
#include <sbpl/utils/car_simulator.h>

#include <boost/functional/hash.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <utility>
#include <map>

typedef double scalar;

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
inline scalar  discretize_angle(scalar  angle, int numofbins) {
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
 * @brief This is cell in a lattice. The x,y,th values will be discretized
 * at construction time
 */
class ContinuousCell {

public:
    typedef std::map<int,
                     std::pair<motion_primitive,
                               boost::weak_ptr<const ContinuousCell>
                              >
                    >
            prims_cells_t;

    ContinuousCell(scalar x, scalar y, scalar th,
                   bool is_forward,
                   scalar map_res,
                   int theta_bins,
                   bool fixed_cells
                   );

    ContinuousCell(const CarSimulator::state_type& p,
                   bool is_forward,
                   scalar map_res, int theta_bins,
                   bool fixed_cells);

    int id() const;
    void setId(int value);
    scalar x() const;
    scalar y() const;
    scalar th() const;
    bool is_forward() const;

    std::size_t hash() const;
    std::string repr() const;

    CarSimulator::state_type toCarState() const;

    void addPredecessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c);

    void addSuccessor(const motion_primitive& p, const boost::shared_ptr<ContinuousCell>& c);

    const prims_cells_t& getPredecessors() const;

    const prims_cells_t& getSuccessors() const;

    void checkHashCollision(const ContinuousCell& other);

    bool operator==(const ContinuousCell& other) const;

    friend std::ostream& operator<<(std::ostream& stream, const ContinuousCell& matrix);

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

inline std::ostream& operator<<(std::ostream& stream, const ContinuousCell& cell) {
    std::stringstream ss;
    ss.unsetf(std::ios::floatfield);
    ss<<std::setprecision(3);
    ss<<cell.x_<<" "<<cell.y_<<" "<<cell.th_<<" "<<-cell.is_forward_;
    stream<<ss.str();
    return stream;
}

inline std::size_t hash_value(ContinuousCell const& c) {
    return c.hash();
}


typedef boost::shared_ptr<ContinuousCell> ContinuousCellPtr;
typedef boost::weak_ptr<const ContinuousCell> ConstContinuousCellWeakPtr;
typedef boost::weak_ptr<ContinuousCell> ContinuousCellWeakPtr;


inline bool operator==(ContinuousCellPtr const& p1, ContinuousCellPtr const& p2)
{
    std::cout<<"Operator == called!\n";
    return (*p1.get()) == (*p2.get());
}

inline std::size_t hash_value(ContinuousCellPtr const& p) {

    std::cout<<"Hash value called!\n";
    boost::hash<ContinuousCell> hasher;
    return hasher(*p.get());

}


#endif // CONTINUOUSCELL_H
