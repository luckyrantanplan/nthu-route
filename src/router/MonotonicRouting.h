/*
 * MonotonicRouting.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_MONOTONICROUTING_H_
#define SRC_ROUTER_MONOTONICROUTING_H_

#include <boost/multi_array.hpp>

#include "../misc/geometry.h"

namespace NTHUR {


class Congestion;
class Two_pin_element_2d;

struct Bound {
    double cost;
    int distance;
    int via_num;
    bool flag;
};

class Monotonic_element {
public:
    double max_cost;
    double total_cost;                         //record the accumulated congestion of the monotonic path
    int net_cost;
    int distance;
    int via_num;

    const bool operator <(Monotonic_element& m2) const;
};

class MonotonicRouting {
public:

    MonotonicRouting(Congestion& congestion, bool enable);
    virtual ~MonotonicRouting();
    bool monotonic_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d& two_pin_monotonic_path, int net_id, Bound& bound);

    void traverse_parent_monotonic(int x1, int y1, int x2, int y2, Two_pin_element_2d& two_pin_monotonic_path);
    void compare_two_direction_congestion(int i, int j, OrientationType dir1, int pre_i, OrientationType dir2, int pre_j, int net_id, Bound& bound);
    void compute_path_total_cost_and_distance(Two_pin_element_2d&element, Monotonic_element&);
    bool monotonicRoute(Two_pin_element_2d& two_pin, Bound& bound, std::vector<Coordinate_2d>& bound_path);

private:
    Congestion& congestion;

    boost::multi_array<Monotonic_element, 2> cong_monotonic; //store max congestion during monotonic path
    boost::multi_array<Coordinate_2d, 2> parent_monotonic;             //record parent (x,y) during finding monotonic path

    bool monotonic_enable;
    void monotonic_routing_algorithm(int x1, int y1, int x2, int y2, OrientationType dir, int net_id, Bound& bound);
    bool smaller_than_lower_bound(const Monotonic_element& m, Bound& bound);
    bool direction_congestion(Coordinate_2d pre, int net_id, int distance, Coordinate_2d c, Monotonic_element& vertical_element, Bound& bound);
};

} // namespace NTHUR

#endif /* SRC_ROUTER_MONOTONICROUTING_H_ */
