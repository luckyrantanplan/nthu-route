#ifndef INC_LAYER_ASSIGNMENT_H
#define INC_LAYER_ASSIGNMENT_H

#include <boost/multi_array.hpp>
#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "../grdb/EdgePlane3d.h"
#include "../grdb/plane.h"
#include "../misc/geometry.h"

class RoutingRegion;

class Congestion;

typedef std::unordered_map<int, int> LRoutedNetTable;

struct KLAT_NODE {
    int val;
    int via_cost;
    int via_overflow;
    int pi_z;
};

class Edge_3d {
public:
    Edge_3d();

public:

    bool isOverflow() const {
        return (cur_cap > max_cap);
    }
    int overUsage() const {
        return (cur_cap - max_cap);
    }
    int max_cap;
    int cur_cap;
    LRoutedNetTable used_net;

};

struct AVERAGE_NODE {
    double average;
    int id;
    int times;
    int val;
    int vo_times;
    int bends;
};

struct LayerInfo {
    std::vector<KLAT_NODE> klat;
    char path;
};

struct EdgeInfo {
    int overflow;
    char path;
};

struct Layer_assignment {

    struct ElementQueue {
        Coordinate_2d coor;
        Coordinate_2d parent;

        ElementQueue(Coordinate_2d coor, Coordinate_2d parent) :
                coor { coor }, parent { parent } {
        }
    };

    struct ElementStack {

        std::vector<Coordinate_3d> choice;
        int others;
        int cost;

        ElementStack(int parent, int val) :
                choice { }, others { parent }, cost { val } {
        }
    };

    struct Segment3d {
        Coordinate_3d first;
        Coordinate_3d last;
    };

    struct Interval {
        int min;
        int max;

        Interval(int min, int max) :
                min { min }, max { max } {
        }

        void add(int i) {
            min = std::min(i, min);
            max = std::max(i, max);
        }
        int length() {
            return max - min;
        }
        bool operator<(const Interval& i) const {
            return (max > i.max || min > i.max);
        }
    };

    struct VertexCost {
        int cost;
        int via_cost;
        Interval interval;
        std::vector<Coordinate_3d> vertices;

        VertexCost(const Interval& interval) :
                cost { std::numeric_limits<int>::max() }, //
                via_cost { std::numeric_limits<int>::max() }, //
                interval { interval }, //
                vertices { } {
        }

        void addCost(const Coordinate_2d& o, const ElementStack& e, const Layer_assignment& l);

        bool operator<(const VertexCost& v) const {
            return std::tie(cost, via_cost, interval) < std::tie(v.cost, v.via_cost, v.interval);
        }
    };

    int max_xx;
    int max_yy;
    int overflow_max;

    std::vector<AVERAGE_NODE> average_order;

    Plane<LayerInfo, EdgeInfo> layerInfo_map; //edge are overflow
    RoutingRegion& rr_map;
    EdgePlane3d<Edge_3d> cur_map_3d;

    int global_net_id;
    int global_pin_num;

    Congestion& congestion;

    void print_max_overflow();

    void initial_overflow_map();
    void malloc_space();
    void update_cur_map_for_klat_xy(int cur_idx, const Coordinate_2d& start, const Coordinate_2d& end, int net_id);
    void update_cur_map_for_klat_z(int min, int max, const Coordinate_2d& start, int net_id);
    void update_path_for_klat(const Coordinate_2d& start);
    void cycle_reduction(const Coordinate_2d& c, const Coordinate_2d& parent);
    void preprocess(int net_id);
    std::vector<Coordinate_3d> rec_count(const Coordinate_3d& o, KLAT_NODE& klatNode);
    void DP(const Coordinate_3d& c, const Coordinate_3d& parent);

    void generate_output(int net_id, const std::vector<Segment3d>& v);
    int klat(int net_id);
    bool comp_temp_net_order(int p, int q);

    void find_group(int max);

    void calculate_wirelength();
    void calculate_cap();
    void sort_net_order();
    void generate_all_output();
    Layer_assignment(const std::string& outputFileNamePtr, RoutingRegion& rr_map, Congestion& congestion);
    void init_3d_map();
private:
    bool test(const Coordinate_2d& c1, const Coordinate_2d& c2);
    void init_union(const Coordinate_2d& c1, const Coordinate_2d& c2, int& max_layer);
    void collectComb(Coordinate_3d c2, Coordinate_3d& c, std::vector<std::vector<Segment3d> >& comb);
};

#endif //INC_LAYER_ASSIGNMENT_H
