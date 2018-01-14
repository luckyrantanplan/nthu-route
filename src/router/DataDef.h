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

namespace NTHUR {

constexpr double error_bound = 0.00000000001;
constexpr double neg_error_bound = -0.00000000001;

constexpr int debug_net_id = 661;
constexpr int debug_serial_id = 1007;

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

    std::string toString() const {
        std::string s = "pin1: " + pin1.toString();
        s += " pin2: " + pin2.toString();
        s += " path: [";
        for (const Coordinate_2d& c : path) {
            s += c.toString() + " ";
        }
        s += "] net_id: " + std::to_string(net_id);
        s += " done: " + std::to_string(done);
        return s;

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

    bool isOverflow() const {
        return (cur_cap > max_cap);
    }
    bool isFull() const {
        return (cur_cap >= max_cap);
    }
    int overUsage() const {
        return static_cast<int>(cur_cap - max_cap);
    }
    bool lookupNet(int netId) const {
        return (used_net.find(netId) != used_net.end());
    }
    double congestion() const {
        return (cur_cap / max_cap);
    }

    std::string toString() const {
        std::string s = "cur" + std::to_string(static_cast<int>(cur_cap));
        s += " max:" + std::to_string(static_cast<int>(max_cap));
        s += " use:" + std::to_string(used_net.size());
        s += " cost:" + std::to_string(cost);
        return s;
    }
};

typedef std::vector<Two_pin_element_2d> Two_pin_list_2d;

} // namespace NTHUR

#endif /* SRC_DATADEF_H_ */
