/*
 * DataDef.h
 *
 *  Created on: Nov 29, 2017
 *      Author: florian
 */

#ifndef SRC_DATADEF_H_
#define SRC_DATADEF_H_

#include <cstdlib>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "../misc/geometry.h"

//If wanna run IBM testcase, please enable this define
//#define IBM_CASE
#define FREE
//#define MESSAGE

#define parameter_h 0.8         // used in the edge cost function 1/0.5 0.8/2
#define parameter_k 2           // used in the edge cost function

#define error_bound 0.00000000001
#define neg_error_bound -0.00000000001

enum {
    DIR_X, DIR_Y
};
enum {
    PIN, STEINER, DELETED
};
enum {
    HOR, VER
};
enum {
    FASTROUTE_COST, OVERFLOW_COST, CONGESTION_COST, MADEOF_COST, HISTORY_COST, HISTORY_MADEOF_COST
};
/* Cost function Prototype */

//store the information of a 2-pin net
class Two_pin_element_2d {
public:
    Two_pin_element_2d() :
            net_id(0), done(-1) {
    }

public:
    Coordinate_2d pin1;
    Coordinate_2d pin2;
    std::vector<Coordinate_2d> path;
    int net_id;
    int done;

    int boxSize() const {
        return abs(pin1.x - pin2.x) + abs(pin1.y - pin2.y);
    }
    static bool comp_2pin_net_from_path(Two_pin_element_2d &a, Two_pin_element_2d &b);

    static bool comp_stn_2pin(const Two_pin_element_2d&a, const Two_pin_element_2d&b) {
        return (a.boxSize() > b.boxSize());
    }

};

class Two_pin_element {
public:
    Coordinate_3d pin1;
    Coordinate_3d pin2;
    std::vector<Coordinate_3d> path;
    int net_id;
    double max_cong;

    static bool comp_2pin_net(Two_pin_element &a, Two_pin_element &b);
};

class Vertex_flute {
public:
    int x, y;
    int type;   //PIN, STEINER, DELETED
    int index;
    int visit;
    std::vector<Vertex_flute *> neighbor;

    Vertex_flute(int x = 0, int y = 0) :
            x(x), y(y), index { }, visit(0)/*, copy_ind(-1)*/{
    }

    //sort by x,y,pin,steiner
    static bool comp_vertex_fl(const Vertex_flute& a, const Vertex_flute& b);

};

typedef std::unordered_map<int, int> RoutedNetTable;

class Edge_2d {
public:
    Edge_2d();

public:

    double cost;               //Used as cache of cost in whole program
    int MMVisitFlag;        //Used as cache of hash table lookup result in MM_mazeroute

    double cur_cap;
    double max_cap;
    int history;
    RoutedNetTable used_net;

    bool isOverflow() {
        return (cur_cap > max_cap);
    }
    bool isFull() {
        return (cur_cap >= max_cap);
    }
    int overUsage() {
        return static_cast<int>(cur_cap - max_cap);
    }
    bool lookupNet(int netId) {
        return (used_net.find(netId) != used_net.end());
    }
    double congestion() {
        return (cur_cap / max_cap);
    }
};

typedef std::unordered_map<int, int> LRoutedNetTable;
class Edge_3d {
public:
    Edge_3d();

public:
    int max_cap;
    int cur_cap;
    std::set<Two_pin_element *> used_two_pin;
    LRoutedNetTable used_net;
};

typedef Edge_2d* Edge_2d_ptr;

typedef std::vector<Two_pin_element_2d> Two_pin_list_2d;
typedef Vertex_flute *Vertex_flute_ptr;

class Vertex_3d {
public:
    std::shared_ptr<Edge_3d> edge_list[6];  //FRONT,BACK,LEFT,RIGHT,UP,DOWN
};

#endif /* SRC_DATADEF_H_ */
