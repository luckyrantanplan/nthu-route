/*
 * Bom.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#include "Congestion.h"
#include "../misc/geometry.h"

Congestion::Congestion(int x, int y) :
        congestionMap2d { x, y }, //
        cache { x, y } {
    exponent = 5.0;
    WL_Cost = 1.0;
    via_cost = 3;
    factor = 1.0;

    used_cost_flag = FASTROUTE_COST;    // cost function type, i.e., HISTORY_COST, HISTORY_MADEOF_COST, MADEOF_COST, FASTROUTE_COST
    pre_evaluate_congestion_cost_fp = [&](int i, int j, OrientationType dir) {pre_evaluate_congestion_cost_all( i, j, dir);};

}

Congestion::~Congestion() {
    // TODO Auto-generated destructor stub
}

//get edge cost on a 2D layer
double Congestion::get_cost_2d(int i, int j, OrientationType dir, int net_id, int *distance) {

//Check if the specified net pass the edge.
//If it have passed the edge before, then the cost is 0.
    if (congestionMap2d.edge(i, j, dir).lookupNet(net_id) == false) {
        (*distance) = 1;

//Used in part II
        switch (used_cost_flag) {

        case HISTORY_COST: {
            return cache.edge(i, j, dir).cost;
        }

//Used in part III: Post processing
        case MADEOF_COST: {
            return congestionMap2d.edge(i, j, dir).isFull();
        }

//Used in part I: Initial routing
        case FASTROUTE_COST: {
            return 1 + parameter_h / (1 + exp((-1) * parameter_k * (congestionMap2d.edge(i, j, dir).cur_cap + 1 - congestionMap2d.edge(i, j, dir).max_cap)));
        }
        }
        return 0;
    } else {
        (*distance) = 0;
        return 0;
    }
}

/*==================DEBUG FUNCTION================================*/
//Obtain the max. overflow and total overflowed value of edges of every gCell
int Congestion::cal_max_overflow() {
    int max_2d_of = 0;       //max. overflow (2D)
    int dif_curmax = 0;

    //obtain the max. overflow and total overflowed value of RIGHT edge of every gCell
    for (int i = congestionMap2d.getXSize() - 2; i >= 0; --i) {
        for (int j = congestionMap2d.getYSize() - 1; j >= 0; --j) {
            if (congestionMap2d.edge(i, j, DIR_EAST).isOverflow())  //overflow occur
            {
                max_2d_of = std::max(max_2d_of, congestionMap2d.edge(i, j, DIR_EAST).overUsage());
                dif_curmax += congestionMap2d.edge(i, j, DIR_EAST).overUsage();
            }
        }
    }

    //obtain the max. overflow and total overflowed value of FRONT edge of every gCell
    for (int i = congestionMap2d.getXSize() - 1; i >= 0; --i) {
        for (int j = congestionMap2d.getYSize() - 2; j >= 0; --j) {
            if (congestionMap2d.edge(i, j, DIR_NORTH).isOverflow()) //overflow occur
            {
                max_2d_of = std::max(max_2d_of, congestionMap2d.edge(i, j, DIR_NORTH).overUsage());
                dif_curmax += congestionMap2d.edge(i, j, DIR_NORTH).overUsage();
            }
        }
    }

    printf("\033[32mcal max overflow=%d   cur_cap-max_cap=%d\033[m\n", max_2d_of, dif_curmax);
    return dif_curmax;
}
/* *NOTICE*
 * You can create many different cost function for difference case easily,
 * just reassign function ponter pre_evaluate_congestion_cost_fp to your
 * function in *route/route.cpp* .                                        */

void Congestion::pre_evaluate_congestion_cost_all(int i, int j, OrientationType dir) {
    static const int inc = 1;
    double cong;

    if (used_cost_flag == HISTORY_COST) {
        cong = (congestionMap2d.edge(i, j, dir).cur_cap + inc) / (congestionMap2d.edge(i, j, dir).max_cap * (1.0 - ((congestionMap2d.edge(i, j, dir).history - 1) / (cur_iter * (1.5 + 3 * factor)))));
        cache.edge(i, j, dir).cost = WL_Cost + (congestionMap2d.edge(i, j, dir).history) * pow(cong, exponent);
    } else {
        if (congestionMap2d.edge(i, j, dir).isFull())
            cache.edge(i, j, dir).cost = 1.0;
        else
            cache.edge(i, j, dir).cost = 0.0;
    }
}

void Congestion::pre_evaluate_congestion_cost() {

    for (int i = congestionMap2d.getXSize() - 1; i >= 0; --i) {
        for (int j = congestionMap2d.getYSize() - 2; j >= 0; --j) {
            pre_evaluate_congestion_cost_fp(i, j, FRONT); // Function Pointer to Cost function
            if (congestionMap2d.edge(i, j, DIR_NORTH).isOverflow()) {
                ++congestionMap2d.edge(i, j, DIR_NORTH).history;
            }
        }
    }
    for (int i = congestionMap2d.getXSize() - 2; i >= 0; --i) {
        for (int j = congestionMap2d.getYSize() - 1; j >= 0; --j) {
            pre_evaluate_congestion_cost_fp(i, j, RIGHT);
            if (congestionMap2d.edge(i, j, DIR_EAST).isOverflow()) {
                ++congestionMap2d.edge(i, j, DIR_EAST).history;
            }
        }
    }
}

//Check if the specified edge is not overflowed
//Return false if the edge is overflowed
bool Congestion::check_path_no_overflow(std::vector<Coordinate_2d*>& path, int net_id, int inc_flag) {
    for (int i = path.size() - 2; i >= 0; --i) {
        Coordinate_2d& coord = *path[i];

        Edge_2d& edge = congestionMap2d.edge(coord.x, coord.y, Coordinate_2d::get_direction(coord, *path[i + 1]));
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

int Congestion::find_overflow_max() {

    int overflow_max = 0;
    for (int i = 0; i < congestionMap2d.edgePlane_.num_elements(); ++i) {
        Edge_2d& edge = congestionMap2d.edgePlane_.data()[i];
        if (edge.overUsage() > overflow_max) {
            overflow_max = edge.overUsage();
        }
    }

    printf("2D maximum overflow = %d\n", overflow_max);
#ifdef MAX_OVERFLOW_CONSTRAINT
#ifdef FOLLOW_PREFER
    if (follow_prefer_direction == 1)
    {
        if (overflow_max % (max_zz / 2))
        overflow_max = ((overflow_max / (max_zz / 2)) << 1) + 2;
        else
        overflow_max = ((overflow_max / (max_zz / 2)) << 1);
    }
    else
    {
        if (overflow_max % max_zz)
        overflow_max = ((overflow_max / max_zz) << 1) + 2;
        else
        overflow_max = ((overflow_max / max_zz) << 1);
    }
#else
    if (overflow_max % max_zz)
    overflow_max = ((overflow_max / max_zz) << 1) + 2;
    else
    overflow_max = ((overflow_max / max_zz) << 1);
#endif
#else
    overflow_max <<= 1;
#endif
    printf("overflow max = %d\n", overflow_max);
    return overflow_max;
}
