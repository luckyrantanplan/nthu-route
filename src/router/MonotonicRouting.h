/*
 * MonotonicRouting.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_MONOTONICROUTING_H_
#define SRC_ROUTER_MONOTONICROUTING_H_

#include <boost/multi_array.hpp>
#include "DataDef.h"

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
    boost::multi_array<Monotonic_element, 2> cong_monotonic; //store max congestion during monotonic path
    boost::multi_array<int, 2> parent_monotonic;             //record parent (x,y) during finding monotonic path
    Monotonic_element* compare_cost(Monotonic_element* m1, Monotonic_element* m2);
    Monotonic_element L_pattern_max_cong(int x1, int y1, int x2, int y2, int dir1, int dir2, Two_pin_element_2d& two_pin_L_path, int net_id);

    MonotonicRouting(int x, int y);
    virtual ~MonotonicRouting();
    void allocate_monotonic();
    bool monotonic_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d& two_pin_monotonic_path, int net_id, Bound& bound);

    void monotonic_routing_algorithm(int x1, int y1, int x2, int y2, int dir, int net_id, Bound& bound);
    void traverse_parent_monotonic(int x1, int y1, int x2, int y2, Two_pin_element_2d& two_pin_monotonic_path);
    void compare_two_direction_congestion(int i, int j, int dir1, int pre_i, int dir2, int pre_j, int net_id, Bound& bound);
    bool smaller_than_lower_bound(const Monotonic_element& m, Bound& bound);

};

#endif /* SRC_ROUTER_MONOTONICROUTING_H_ */
