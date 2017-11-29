/*
 * Bom.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_ROUTER_CONGESTION_H_
#define SRC_ROUTER_CONGESTION_H_

#include <boost/multi_array.hpp>

#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"
#include "DataDef.h"

struct CacheEdge {
    double cost;               //Used as cache of cost in whole program
    int MMVisitFlag;        //Used as cache of hash table lookup result in MM_mazeroute

    CacheEdge() :
            cost(0.0), MMVisitFlag(-1) {
    }
};

class Congestion {
public:

    std::function<void(int i, int j, int dir)> pre_evaluate_congestion_cost_fp;

    int via_cost;
    int used_cost_flag;
    double exponent;
    double WL_Cost;
    double factor;

    EdgePlane<Edge_2d> congestionMap2d;
    EdgePlane<CacheEdge> cache;
    Congestion();
    virtual ~Congestion();
    double get_cost_2d(int i, int j, OrientationType dir, int net_id, int *distance);
    int cal_max_overflow();
    void pre_evaluate_congestion_cost_all(int i, int j, OrientationType dir);
    void pre_evaluate_congestion_cost();

};

#endif /* SRC_ROUTER_CONGESTION_H_ */
