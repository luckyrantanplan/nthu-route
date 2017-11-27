#ifndef INC_LAYER_ASSIGNMENT_H
#define INC_LAYER_ASSIGNMENT_H

#include <array>
#include <memory>
#include <vector>
#include "../misc/geometry.h"
#include "Construct_2d_tree.h"

struct ans {
    int idx;
    int val;
    void update(int itar, int ival) {
        idx = itar;
        val = ival;
    }

};
struct DP_NODE {
    int val;
    Jm::Coordinate_3d *pi;
};
struct NET_NODE {
    int id;
    int val;
};
struct AVERAGE_NODE {
    int id;
    int times;
    int val;
    int vo_times;
    double average;
    int bends;
};
struct NET_INFO_NODE {
    int xy;
    int z;
    double val;
};

struct MULTIPIN_NET_NODE {
    Two_pin_list_2d two_pin_net_list;
};

struct PATH_NODE {
    char val;
    char edge[4];
};
struct KLAT_NODE {
    int val;
    int via_cost;
    int via_overflow;
    int pi_z;
};
struct OVERFLOW_NODE {
    std::array<std::shared_ptr<int>, 4> edge;
};
struct UNION_NODE {
    int pi;
    int sx, sy, bx, by;
    int num;
};
struct LENGTH_NODE {
    int xy;
    int z;
};
struct VIADENSITY_NODE {
    int cur;
    int max;
};
struct PATH_EDGE_3D {
    set<int> used_net;
};

struct PATH_VERTEX_3D {
    std::array<std::shared_ptr<PATH_EDGE_3D>, 6> edge_list;
};

struct ZLayerInfo {
    KLAT_NODE klat;
    VIADENSITY_NODE viadensity;
    Jm::Coordinate_3d coord_3d;
};

struct LayerInfo {
    PATH_NODE path;
    OVERFLOW_NODE overflow;
    std::vector<ZLayerInfo> zLayerInfo;

};

extern void Layer_assignment();

#endif //INC_LAYER_ASSIGNMENT_H
