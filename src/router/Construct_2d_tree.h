#ifndef _CONSTRUCT_2D_TREE_H_
#define _CONSTRUCT_2D_TREE_H_

#include "parameter.h"

#include "../grdb/RoutingRegion.h"
#include "../grdb/plane.h"
#include "../misc/geometry.h"
#include "../flute/flute-ds.h"
#include "../util/traversemap.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <string>

using std::vector;
using std::set;
using std::map;

//If wanna run IBM testcase, please enable this define
//#define IBM_CASE
#define FREE
//#define MESSAGE

#define parameter_h 0.8         // used in the edge cost function 1/0.5 0.8/2
#define parameter_k	2			// used in the edge cost function

enum {
    FRONT, BACK, LEFT, RIGHT, UP, DOWN
};
enum {
    DIR_X, DIR_Y
};
enum {
    PIN, STEINER, DELETED
};
enum {
    HOR, VER
};

/* Cost function Prototype */
extern void (*pre_evaluate_congestion_cost_fp)(int i, int j, int dir);
void pre_evaluate_congestion_cost_all(int i, int j, int dir);

//store the information of a 2-pin net
class Two_pin_element_2d {
public:
    Two_pin_element_2d() :
            net_id(0), done(-1) {
    }

public:
    Jm::Coordinate_2d pin1;
    Jm::Coordinate_2d pin2;
    vector<Jm::Coordinate_2d*> path;
    int net_id;
    int done;

    int boxSize() const {
        return abs(pin1.x - pin2.x) + abs(pin1.y - pin2.y);
    }
};

class Monotonic_element {
public:
    double max_cost;
    double total_cost; 			//record the acculmated congestion of the monotonic path
    int net_cost;
    int distance;
    int via_num;
};

class Two_pin_element {
public:
    Jm::Coordinate_3d pin1;
    Jm::Coordinate_3d pin2;
    vector<Jm::Coordinate_3d*> path;
    int net_id;
    double max_cong;

    Two_pin_element() {
    }
    ;
};

class Vertex_flute {
public:
    int x, y;
    int type;	//PIN, SETINER, DELETED
    int index;
    int visit;
    vector<Vertex_flute *> neighbor;

    Vertex_flute(int x = 0, int y = 0) :
            x(x), y(y), visit(0)/*, copy_ind(-1)*/{
    }
};

typedef std::unordered_map<int, int> RoutedNetTable;
class Edge_2d {
public:
    Edge_2d();

public:
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
    set<Two_pin_element *> used_two_pin;
    LRoutedNetTable used_net;
};

typedef Edge_2d* Edge_2d_ptr;
typedef Edge_3d* Edge_3d_ptr;
typedef vector<Two_pin_element_2d*> Two_pin_list_2d;
typedef Vertex_flute *Vertex_flute_ptr;

class Vertex_3d {
public:
    Edge_3d_ptr edge_list[6];	//FRONT,BACK,LEFT,RIGHT,UP,DOWN
};

struct CacheEdge {
    double cost;               //Used as cache of cost in whole program
    int MMVisitFlag;        //Used as cache of hash table lookup result in MM_mazeroute

    CacheEdge() :
            cost(0.0), MMVisitFlag(-1) {
    }
};

extern int par_ind;
extern ParameterSet* parameter_set;
extern RoutingParameters* routing_parameter;

extern const int dir_array[4][2]; //FRONT,BACK,LEFT,RIGHT
extern const int Jr2JmDirArray[4];

extern EdgePlane<Edge_2d>* congestionMap2d;
extern vector<Two_pin_element_2d*> two_pin_list;
extern int two_pin_list_size;
extern int flute_mode;
extern Monotonic_element **cong_monotonic; //store max congestion during monotonic path
extern int **parent_monotonic;             //record parent (x,y) during finding monotonic path
extern Jm::Coordinate_2d **coor_array;
extern int via_cost;
extern double max_congestion_factor;
extern int fail_find_path_count;

extern Vertex_3d ***cur_map_3d;
extern vector<Two_pin_element*> all_two_pin_list;

extern RoutingRegion *rr_map;
extern int cur_iter;
extern int done_iter;
extern double alpha;
extern int total_overflow;
extern int used_cost_flag;
extern int BOXSIZE_INC;
extern Tree* net_flutetree;
extern EdgePlane<CacheEdge>* cache;
//#include "MM_mazeroute.h"

class Multisource_multisink_mazeroute;

extern Multisource_multisink_mazeroute* mazeroute_in_range;

extern void update_congestion_map_insert_two_pin_net(Two_pin_element_2d *element);
extern void update_congestion_map_remove_two_pin_net(Two_pin_element_2d *element);
extern bool monotonic_pattern_route(int x1, int y1, int x2, int y2, Two_pin_element_2d* two_pin_monotonic_path, int net_id, double bound_cost, int bound_distance, int bound_via_num, bool bound_flag);
extern void insert_all_two_pin_list(Two_pin_element_2d *mn_path_2d);
extern double get_cost_2d(int i, int j, int dir, int net_id, int *distance);

extern bool smaller_than_lower_bound(double total_cost, int distance, int via_num, double bound_cost, int bound_distance, int bound_via_num);

inline bool comp_stn_2pin(const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
    return (a->boxSize() > b->boxSize());
}

inline int get_direction_2d_simple(const Jm::Coordinate_2d* a, const Jm::Coordinate_2d* b) {
    assert(!(a->x != b->x && a->y != b->y));

    if (a->x != b->x)
        return LEFT;
    else
        return FRONT;
}

inline int get_direction_2d(const Jm::Coordinate_2d* a, const Jm::Coordinate_2d* b) {
    assert(!(a->x != b->x && a->y != b->y));

    if (a->x != b->x) {
        if (a->x > b->x)
            return LEFT;
        else
            return RIGHT;
    } else {
        if (a->y > b->y)
            return BACK;
        else
            return FRONT;
    }
}

inline
void printMemoryUsage(const char* msg) {
    std::cout << msg << std::endl;
    //for print out memory usage
    std::ifstream mem("/proc/self/status");
    std::string memory;
    for (unsigned i = 0; i < 13; i++) {
        getline(mem, memory);
        if (i > 10) {
            std::cout << memory << std::endl;
        }
    }
}

int cal_max_overflow();

void pre_evaluate_congestion_cost();

int cal_total_wirelength();
void init_3d_map();
void output_3d_map();

extern vector<bool> NetDirtyBit;
#endif /* _CONSTRUCT_2D_TREE_H_ */
