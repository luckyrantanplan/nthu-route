/*
 * Bom.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#include "Congestion.h"

#include <boost/range/combine.hpp>
#include <boost/range/detail/combine_cxx11.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/tuple/detail/tuple_basic.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <utility>

#include "../grdb/plane.h"
#include "../grdb/RoutingRegion.h"
#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "../spdlog/spdlog.h"

namespace NTHUR {

constexpr double parameter_h = 0.8;         // used in the edge cost function 1/0.5 0.8/2
constexpr double parameter_k = 2;           // used in the edge cost function
}
NTHUR::Congestion::Congestion(int x, int y) :
        congestionMap2d { x, y }  //
{
    exponent = 5.0;
    WL_Cost = 1.0;
    via_cost = 3;
    factor = 1.0;
    cur_iter = -1;                  // current iteration ID.
    used_cost_flag = FASTROUTE_COST;    // cost function type, i.e., HISTORY_COST, HISTORY_MADEOF_COST, MADEOF_COST, FASTROUTE_COST
    pre_evaluate_congestion_cost_fp = [&]( Edge_2d& edge) {pre_evaluate_congestion_cost_all( edge);};
    log_sp = spdlog::get("NTHUR");
}
namespace NTHUR {

//get edge cost on a 2D layer
double Congestion::get_cost_2d(const Coordinate_2d& c1, const Coordinate_2d& c2, int net_id, int& distance) {

//Check if the specified net pass the edge.
//If it have passed the edge before, then the cost is 0.
    Edge_2d& edge = congestionMap2d.edge(c1, c2);

    if (edge.lookupNet(net_id) == false) {
        distance = 1;

        switch (used_cost_flag) {

        case HISTORY_COST: {    //Used in part II
            return edge.cost;
        }

        case MADEOF_COST: {    //Used in part III: Post processing
            return edge.isFull();
        }

        case FASTROUTE_COST: {    //Used in part I: Initial routing
            return 1 + parameter_h / (1 + exp((-1) * parameter_k * (edge.cur_cap + 1 - edge.max_cap)));
        }
        }
        return 0;
    } else {
        distance = 0;
        return 0;
    }
}

/*==================DEBUG FUNCTION================================*/
//Obtain the max. overflow and total overflowed value of edges of every gCell
int Congestion::cal_max_overflow() {
    int max_2d_of = 0;       //max. overflow (2D)
    int dif_curmax = 0;

    for (Edge_2d& edge : congestionMap2d.all()) {
        pre_evaluate_congestion_cost_fp(edge);
        if (edge.isOverflow()) {
            max_2d_of = std::max(max_2d_of, edge.overUsage());
            dif_curmax += edge.overUsage();
        }
    }

    //obtain the max. overflow and total overflowed value of RIGHT edge of every gCell

    log_sp->info("cal max overflow= {} cur_cap-max_cap= {}", max_2d_of, dif_curmax);

    SPDLOG_TRACE(log_sp, "gridEdge \n{}", congestionMap2d.toString());

    return dif_curmax;
}
/* *NOTICE*
 * You can create many different cost function for difference case easily,
 * just reassign function pointer pre_evaluate_congestion_cost_fp to your
 * function in *route/route.cpp* .                                        */

void Congestion::pre_evaluate_congestion_cost_all(Edge_2d& edge) const {
    static const int inc = 1;
    if (used_cost_flag == HISTORY_COST) {
        double cong = (edge.cur_cap + inc) / (edge.max_cap * (1.0 - ((edge.history - 1) / (cur_iter * (1.5 + 3 * factor)))));
        edge.cost = WL_Cost + (edge.history) * pow(cong, exponent);
    } else {
        if (edge.isFull())
            edge.cost = 1.0;
        else
            edge.cost = 0.0;
    }
}

void Congestion::pre_evaluate_congestion_cost() {

    for (Edge_2d& edge : congestionMap2d.all()) {
        pre_evaluate_congestion_cost_fp(edge);

        if (edge.isOverflow()) {
            ++edge.history;
        }

    }SPDLOG_TRACE(log_sp, "pre_evaluate_congestion_cost gridEdge \n{}", congestionMap2d.toString());
}

//Check if the specified edge is not overflowed
//Return false if the edge is overflowed
bool Congestion::check_path_no_overflow(const std::vector<Coordinate_2d>& path, const int net_id, const int inc_flag) const {
    for (int i = path.size() - 2; i >= 0; --i) {

        const Edge_2d& edge = congestionMap2d.edge(path[i], path[i + 1]);
        //There are two modes:
        // 1. inc_flag = 0: Just report if the specified edge is overflowd
        // 2. inc_flag = 1: Check if the specified edge will be overflowed if wd add a demond on it.
        if (inc_flag == 0) {
            if (edge.isOverflow()) {
                return false;
            }
        } else {
            int inc = 1;
            //If there is another 2-pin net from the same net is using the specified edge,
            //then we don't need to increase a demand on it. We can use the edge directly
            if (edge.lookupNet(net_id))
                inc = 0;

            if (edge.cur_cap + inc > edge.max_cap)
                return false;
        }
    }
    return true;
}

int Congestion::find_overflow_max(int max_zz) const {
    int overflow_max = 0;
    for (const Edge_2d& edge : congestionMap2d.all()) {
        if (edge.overUsage() > overflow_max) {
            overflow_max = edge.overUsage();
        }
    }SPDLOG_TRACE(log_sp, "2D maximum overflow = {}", overflow_max);

    if (overflow_max % max_zz) {
        overflow_max = ((overflow_max / max_zz) * 2) + 2;
    } else {
        overflow_max = ((overflow_max / max_zz) * 2);
    }SPDLOG_TRACE(log_sp, "overflow max = = {}", overflow_max);

    return overflow_max;
}

/*assign the estimated track# to each edge*/
void Congestion::init_2d_map(const RoutingRegion& rr_map) {
#define IBM_CASE
#ifdef IBM_CASE
    int divisor = 2;
#else
    int divisor = 1;
#endif
    const EdgePlane3d<int>& routingSpace = rr_map.getMaxCapacity();

    for (int x = 0; x < routingSpace.getXSize(); ++x) {
        for (int y = 0; y < routingSpace.getYSize(); ++y) {
            NTHUR::Coordinate_2d c2 = Coordinate_2d { x, y };
            for (int z = 0; z < routingSpace.getZSize(); ++z) {
                congestionMap2d.east(c2).max_cap += routingSpace.east(Coordinate_3d { x, y, z });
                congestionMap2d.south(c2).max_cap += routingSpace.south(Coordinate_3d { x, y, z });
            }
        }
    }
    for (Edge_2d& edge : congestionMap2d.all()) {
        edge.max_cap /= divisor;

    }

}

//Sum all demand value on every edge
//So if demand value = wire length, this function can be used
int Congestion::cal_total_wirelength() const {
    int total_wl = 0;
    for (const Edge_2d& edge : congestionMap2d.all()) {
        total_wl += (int) edge.cur_cap;

    }

    log_sp->info("total wire length: {}", total_wl);
    return total_wl;
}

Congestion::Statistic Congestion::stat_congestion() {
    Statistic s;
    s.max = std::numeric_limits<double>::max();
    s.min = std::numeric_limits<double>::min();
    s.avg = 0;

    for (Edge_2d& edge : congestionMap2d.all()) {
        double edgeCongestion = edge.congestion();
        if (edgeCongestion > 1.0) {
            s.min = std::min(edgeCongestion, s.min);
            s.max = std::max(edgeCongestion, s.max);
            s.avg += edgeCongestion;
        }
    }

    s.avg /= congestionMap2d.num_elements();
    return s;
}

//Add the path of two pin element on to congestion map
//The congestion map record not only which net pass which edge,
//but also the number of a net pass through
void Congestion::update_congestion_map_insert_two_pin_net(Two_pin_element_2d& element) {

    for (int i = element.path.size() - 2; i >= 0; --i) {
//get an edge from congestion map - c_map_2d

        Edge_2d& edge = congestionMap2d.edge(element.path[i], element.path[i + 1]);
        std::pair<RoutedNetTable::iterator, bool> insert_result = edge.used_net.insert(std::pair<const int, int>(element.net_id, 1));

        if (!insert_result.second)
            ++((insert_result.first)->second);
        else {
            ++edge.cur_cap;

            if (used_cost_flag != FASTROUTE_COST) {
                pre_evaluate_congestion_cost_fp(edge);
            }
        }
    }
}

//Remove a net from an edge.
//If the net pass that edge more than once, this function will only decrease the counter.
void Congestion::update_congestion_map_remove_two_pin_net(const std::vector<Coordinate_2d>& path, const int net_id) {

    for (int i = path.size() - 2; i >= 0; --i) {
        Edge_2d& edge = congestionMap2d.edge(path[i], path[i + 1]);
        RoutedNetTable::iterator find_result = edge.used_net.find(net_id);

        --(find_result->second);
        if (find_result->second == 0) {
            edge.used_net.erase(net_id);
            --(edge.cur_cap);
            if (used_cost_flag != FASTROUTE_COST) {
                pre_evaluate_congestion_cost_fp(edge);
            }
        }
    }
}

std::string Congestion::plotCongestionNet(int net_id) const {
    std::string s;
    for (int x = 0; x < congestionMap2d.getXSize(); ++x) {
        for (int y = 1; y < congestionMap2d.getYSize(); ++y) {
            Coordinate_2d c1 { x, y - 1 };
            Coordinate_2d c2 { x, y };
            if (congestionMap2d.edge(c1, c2).lookupNet(net_id)) {
                s += std::to_string(c1.x) + " " + std::to_string(c1.y) + "\n";
                s += std::to_string(c2.x) + " " + std::to_string(c2.y) + "\n\n";
            }
        }
    }
    for (int y = 0; y < congestionMap2d.getYSize(); ++y) {
        for (int x = 1; x < congestionMap2d.getXSize(); ++x) {
            Coordinate_2d c1 { x - 1, y };
            Coordinate_2d c2 { x, y };
            if (congestionMap2d.edge(c1, c2).lookupNet(net_id)) {
                s += std::to_string(c1.x) + " " + std::to_string(c1.y) + "\n";
                s += std::to_string(c2.x) + " " + std::to_string(c2.y) + "\n\n";
            }
        }
    } //
    s += "end congestion for net_id" + std::to_string(net_id) + "true\n";
    return s;
}

void Congestion::calculate_cap() const {

    int overflow = 0;
    int max = 0;
    for (const Edge_2d& edge : congestionMap2d.all()) {
        if (edge.isOverflow()) {
            overflow += (edge.overUsage() * 2);
            if (max < edge.overUsage() * 2)
                max = edge.overUsage() * 2;
        }
    }
    log_sp->info("2D sum overflow = {}", overflow); //
    log_sp->info("2D max overflow = {}", max);
}

} // namespace NTHUR
