/*
 * Bom.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#include "Congestion.h"

#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/range/combine.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <vector>

#include "../grdb/RoutingRegion.h"

Congestion::Congestion(int x, int y) :
        congestionMap2d { x, y }  //
{
    exponent = 5.0;
    WL_Cost = 1.0;
    via_cost = 3;
    factor = 1.0;
    used_cost_flag = FASTROUTE_COST;    // cost function type, i.e., HISTORY_COST, HISTORY_MADEOF_COST, MADEOF_COST, FASTROUTE_COST
    pre_evaluate_congestion_cost_fp = [&]( Edge_2d& edge) {pre_evaluate_congestion_cost_all( edge);};

}

Congestion::~Congestion() {
    // TODO Auto-generated destructor stub
}

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

    printf("\033[32mcal max overflow=%d   cur_cap-max_cap=%d\033[m\n", max_2d_of, dif_curmax);
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
    }
}

//Check if the specified edge is not overflowed
//Return false if the edge is overflowed
bool Congestion::check_path_no_overflow(std::vector<Coordinate_2d>& path, int net_id, int inc_flag) {
    for (int i = path.size() - 2; i >= 0; --i) {

        Edge_2d& edge = congestionMap2d.edge(path[i], path[i + 1]);
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
    }

    printf("2D maximum overflow = %d\n", overflow_max);

    if (overflow_max % max_zz) {
        overflow_max = ((overflow_max / max_zz) * 2) + 2;
    } else {
        overflow_max = ((overflow_max / max_zz) * 2);
    }

    printf("overflow max = %d\n", overflow_max);
    return overflow_max;
}

/*assign the estimated track# to each edge*/
void Congestion::init_2d_map(RoutingRegion& rr_map) {
#ifdef IBM_CASE
    int divisor=2;
#else
    int divisor = 1;
#endif
    for (int layer = 0; layer < rr_map.get_layerNumber(); ++layer) {
        for (auto pair : boost::combine(congestionMap2d.all(), rr_map.getLayer(layer).edges().all())) {
            pair.get<0>().max_cap += pair.get<1>() / divisor;
        }
    }
}

//Sum all demand value on every edge
//So if demand value = wire length, this function can be used
int Congestion::cal_total_wirelength() {
    int total_wl = 0;
    for (Edge_2d& edge : congestionMap2d.all()) {
        total_wl += (int) edge.cur_cap;

    }

    printf("total wirelength:%d\n", total_wl);
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
void Congestion::update_congestion_map_remove_two_pin_net(Two_pin_element_2d& element) {

    for (int i = element.path.size() - 2; i >= 0; --i) {
        Edge_2d& edge = congestionMap2d.edge(element.path[i], element.path[i + 1]);
        RoutedNetTable::iterator find_result = edge.used_net.find(element.net_id);

        --(find_result->second);
        if (find_result->second == 0) {
            edge.used_net.erase(element.net_id);
            --(edge.cur_cap);
            if (used_cost_flag != FASTROUTE_COST) {
                pre_evaluate_congestion_cost_fp(edge);
            }
        }
    }
}
