/*
 * Bom.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_CONGESTION_H_
#define SRC_ROUTER_CONGESTION_H_

#include <functional>
#include <vector>

#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "DataDef.h"

class RoutingRegion;

class Congestion {
public:

    struct Statistic {

        double min;
        double max;
        double avg;
    };

    std::function<void(Edge_2d& edge)> pre_evaluate_congestion_cost_fp;

    int via_cost;
    int used_cost_flag;
    double exponent;
    double WL_Cost;
    double factor;

    EdgePlane<Edge_2d> congestionMap2d;

    Congestion(int x, int y);
    ~Congestion();
    double get_cost_2d(int i, int j, OrientationType dir, int net_id, int *distance);
    int cal_max_overflow();
    void pre_evaluate_congestion_cost_all(Edge_2d& edge);
    void pre_evaluate_congestion_cost();
    bool check_path_no_overflow(std::vector<Coordinate_2d>&path, int net_id, int inc_flag);
    int find_overflow_max();
    void init_2d_map(RoutingRegion& rr_map);
    int cal_total_wirelength();
    Statistic stat_congestion();
    void update_congestion_map_insert_two_pin_net(Two_pin_element_2d& element);
    void update_congestion_map_remove_two_pin_net(Two_pin_element_2d& element);
};

#endif /* SRC_ROUTER_CONGESTION_H_ */
