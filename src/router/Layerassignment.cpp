#include "Layerassignment.h"

#include <boost/multi_array/multi_array_ref.hpp>
#include <boost/range/combine.hpp>
#include <boost/range/detail/combine_cxx11.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <deque>
#include <iostream>
#include <queue>
#include <stack>

#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"
#include "Congestion.h"

using namespace std;

char prefer_direction[6][2] = { 0 };

Edge_3d::Edge_3d() :
        max_cap(0), cur_cap(0), used_net(5) {

}

void Layer_assignment::print_max_overflow() {
    int lines = 0;
    int max = 0;
    int sum = 0;
    for (Edge_3d& edgeLeft : cur_map_3d.all()) {

        if (edgeLeft.isOverflow()) {
            max = std::max(edgeLeft.overUsage(), max);
            sum += edgeLeft.overUsage();
            ++lines;
        }
    }

    printf("3D # of overflow = %d\n", sum);
    printf("3D max overflow = %d\n", max);
    printf("3D overflow edge number = %d\n", lines);
}

void Layer_assignment::initial_overflow_map() {

    for (auto pair : boost::combine(congestion.congestionMap2d.all(), layerInfo_map.edges().all())) {
        pair.get<1>().overflow = pair.get<0>().overUsage() * 2;
    }
}

void Layer_assignment::malloc_space() {

    for (LayerInfo& layerInfo : layerInfo_map.allVertex()) {
        layerInfo.path = 0;  // non-visited
    }
    for (EdgeInfo& edge : layerInfo_map.edges().all()) {
        edge.overflow = 0;
        edge.path = 0;
    }

    for (Edge_3d& edge : cur_map_3d.all()) {
        edge.cur_cap = 0;
        edge.max_cap = 0;
    }
}

void Layer_assignment::update_cur_map_for_klat_xy(int cur_idx, const Coordinate_2d& start, const Coordinate_2d& end, int net_id) {

    Edge_3d& edge = cur_map_3d.edge(Coordinate_3d { end, cur_idx }, Coordinate_3d { start, cur_idx });
    ++edge.used_net[net_id];
    edge.cur_cap = (edge.used_net.size() * 2);	// need check
    if (edge.isOverflow()) {	// need check
        layerInfo_map.edges().edge(start, end).overflow -= 2;
    }
}

void Layer_assignment::update_cur_map_for_klat_z(int min, int max, const Coordinate_2d& start, int net_id) {

    Coordinate_3d previous { start, min };
    for (Coordinate_3d k { start, min + 1 }; k.z < max; ++k.z) {
        Edge_3d& edge = cur_map_3d.edge(k, previous);
        ++edge.used_net[net_id];
        ++edge.cur_cap;
        previous = k;
    }

}

void Layer_assignment::update_path_for_klat(const Coordinate_2d& start) {
    int pin_num = 0;
    std::queue<Coordinate_3d> q;

// BFS
    q.push(Coordinate_3d { start, 0 });	// enqueue
    while (!q.empty()) {
        Coordinate_3d& head3d = q.front();
        Coordinate_2d h = head3d.xy();
        int z_min = 0;
        if (layerInfo_map.vertex(h).path == 2) {	// a pin
            ++pin_num;
        } else {
            z_min = head3d.z;
        }
        int z_max = head3d.z;
        for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(h)) {
            if (handle.edge().path == 1) {	// check legal
                Coordinate_2d& c = handle.vertex();
                if (layerInfo_map.vertex(c).path == 0) {
                    puts("ERROR");
                }
                int pi_z = layerInfo_map.vertex(c).klat[head3d.z].pi_z;
                z_max = std::max(z_max, pi_z);
                z_min = std::min(z_min, pi_z);
                update_cur_map_for_klat_xy(pi_z, h, c, global_net_id);
                q.push(Coordinate_3d { c, pi_z });	// enqueue
            }
        }
        update_cur_map_for_klat_z(z_min, z_max, h, global_net_id);
        layerInfo_map.vertex(h).path = 0;	// visited
        q.pop();	// dequeue
    }
    if (pin_num != global_pin_num)
        printf("net : %d, pin number error %d vs %d\n", global_net_id, pin_num, global_pin_num);
}

void Layer_assignment::cycle_reduction(const Coordinate_2d& c, const Coordinate_2d& parent) {

    for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(c)) {
        if (parent != handle.vertex() && handle.edge().path == 1) {   // check legal
            cycle_reduction(handle.vertex(), c);
        }
    }
    // border effect, we need to loop again
    bool isLeaf = true;
    for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(c)) {
        if (parent != handle.vertex() && handle.edge().path == 1) {   // check legal
            isLeaf = false;
        }
    }
    if (isLeaf && layerInfo_map.vertex(c).path == 1) {	// a leaf && it is not a pin
        layerInfo_map.vertex(c).path = 0;
        layerInfo_map.edges().edge(c, parent).path = 0;
    }
}

void Layer_assignment::preprocess(int net_id) {

    const PinptrList& pin_list = rr_map.get_nPin(net_id);

    for (const Pin* pin : pin_list) {
        layerInfo_map.vertex(pin->get_tileXY()).path = -2;	// pin
    }
// BFS speed up
    Coordinate_2d c = pin_list[0]->get_tileXY();
    layerInfo_map.vertex(c).path = 2;	// visited

    for (KLAT_NODE& k : layerInfo_map.vertex(c).klat) {
        k.val = -1;
    }

    std::queue<ElementQueue> q;
    q.emplace(c, c);	// enqueue
    while (!q.empty()) {
        ElementQueue& front = q.front();
        for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(front.coor)) {
            if (handle.vertex() != front.parent) {
                if (congestion.congestionMap2d.edge(front.coor, handle.vertex()).lookupNet(net_id)) {	//
                    char& pathV = layerInfo_map.vertex(handle.vertex()).path;
                    if (pathV == 0 || pathV == -2) {

                        handle.edge().path = 1;
                        if (pathV == 0) {
                            pathV = 1;	// visited node
                        } else {	// path_map[x][y].val == 2
                            pathV = 2;	// visited pin

                        }
                        for (KLAT_NODE& k : layerInfo_map.vertex(handle.vertex()).klat) {
                            k.val = -1;
                        }
                        q.emplace(handle.vertex(), front.coor);	// enqueue coor + parent
                    } else {
                        // already visited : we cut to avoid loop
                        handle.edge().path = 0;
                    }
                } else {
                    //nothing to see here
                    handle.edge().path = 0;
                }
            }
        }
        q.pop();	// dequeue
    }
    cycle_reduction(c, c);
}

void Layer_assignment::VertexCost::addCost(const Coordinate_2d& o, const ElementStack& e, const Layer_assignment& l) {
    for (const Coordinate_3d& c : e.choice) {
        interval.add(c.z);
    }

    cost = e.cost;
    for (Coordinate_3d c2 { o, interval.min }; c2.z < interval.max; ++c2.z) {
        if (l.cur_map_3d.edge(c2, Coordinate_3d { o, c2.z + 1 }).isOverflow()) {
            ++cost;
        }
    }
    via_cost = 0;
    for (const Coordinate_3d& c : e.choice) {
        via_cost += l.layerInfo_map.vertex(c.xy()).klat[c.z].via_cost; // does not understand this
    }
    via_cost += (interval.length() * l.global_via_cost);

}

std::vector<Coordinate_3d> Layer_assignment::rec_count(const Coordinate_3d& o, KLAT_NODE& klatNode) {
    Interval int_DP_k { 0, o.z };
    if (layerInfo_map.vertex(o.xy()).path == 1) { // not a pin
        int_DP_k.min = o.z;
    }     // pin
    VertexCost vertexCost(int_DP_k);

    std::vector<Coordinate_2d> neighbors;
    std::stack<ElementStack, std::vector<ElementStack>> stack;

    for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(o.xy())) {
        if (handle.edge().path == 1) {
            neighbors.push_back(handle.vertex());
        }
    }
    stack.emplace(neighbors.size() - 1, 0);
    while (!stack.empty()) {
        ElementStack e = stack.top();
        stack.pop();
        if (e.cost <= vertexCost.cost) {
            if (e.others >= 0) {
                Coordinate_2d child = neighbors[e.others];
                --e.others;

                for (std::size_t k = 0; k < layerInfo_map.vertex(child).klat.size(); ++k) {
                    int cost = layerInfo_map.vertex(child).klat[k].val;
                    if (cost >= 0) {
                        ElementStack e1 = e;
                        e1.choice.push_back(Coordinate_3d { child, static_cast<int32_t>(k) });
                        e1.cost += cost;
                        stack.push(std::move(e1));
                    }
                }
            } else {
                VertexCost vcost(int_DP_k);
                vcost.addCost(o.xy(), e, *this);
                if (vcost < vertexCost) {
                    vertexCost = std::move(vcost);
                }
            }
        }     // current e.val is >   min : we do nothing
    }
    klatNode.via_cost = vertexCost.via_cost;
    klatNode.via_overflow = vertexCost.cost;
    klatNode.val = vertexCost.cost;
    return vertexCost.vertices;
}

void Layer_assignment::DP(const Coordinate_3d& c, const Coordinate_3d& parent) {

    Coordinate_2d c1(c.xy());
    KLAT_NODE& klatNode = layerInfo_map.vertex(c1).klat[c.z];
    if (klatNode.val == -1) {	// non-traversed
        bool is_end = true;
        for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(c1)) {	// direction
            if (handle.edge().path == 1 && handle.vertex() != parent.xy()) {	// check legal
                is_end = false;
                for (Coordinate_3d c2 { handle.vertex(), 0 }; c2.z < cur_map_3d.getZSize(); ++c2.z) {	// use global_max_layer to substitute max_zz
                    Edge_3d& edge = cur_map_3d.edge(Coordinate_3d { c1, c2.z }, c2);
                    if (((handle.edge().overflow <= 0) && !edge.isOverflow()) ||	//
                            ((edge.overUsage()) < overflow_max)) {	// pass without overflow
                        DP(c2, c);
                    }
                }
            }
        }
        if (is_end) {
            klatNode.via_cost = global_via_cost * c.z;
            klatNode.via_overflow = 0;
            for (Coordinate_3d c2 { c1, 0 }; c2.z < c.z; ++c2.z) {
                if (cur_map_3d.edge(c2, Coordinate_3d { c1, c.z + 1 }).isOverflow()) {
                    ++klatNode.via_overflow;
                }
            }
            klatNode.val = klatNode.via_overflow;

        } else {	// is_end == false
            for (const Coordinate_3d& n : rec_count(c, klatNode)) {
                layerInfo_map.vertex(n.xy()).klat[c.z].pi_z = n.z;
            }
        }
    }
}

void Layer_assignment::collectComb(Coordinate_3d c2, Coordinate_3d& c, std::vector<std::vector<Segment3d> >& comb) {
    for (pair<const int, int>& net : cur_map_3d.edge(c, c2).used_net) {
        std::vector<Segment3d>& v = comb[net.first];
        if (v.empty()) {
            v.push_back(Segment3d { c2, c });
        } else {
            Segment3d& interval = v.back();
            if (interval.last == c2 && interval.first.isAligned(c)) {
                interval.last = c;
            } else {
                v.push_back(Segment3d { c2, c });
            }
        }
    }
}

void Layer_assignment::generate_output(int net_id, const std::vector<Segment3d>& v, std::ostream & output) {

    int xDetailShift = rr_map.get_llx() + (rr_map.get_tileWidth() / 2);
    int yDetailShift = rr_map.get_lly() + (rr_map.get_tileHeight() / 2);

// the beginning of a net of output file

    output << " p " << rr_map.get_netName(net_id) << "\n";

    // have edge
    for (const Segment3d& seg : v) {
        Coordinate_3d o = seg.first;
        Coordinate_3d d = seg.last;
        o.x = o.x * rr_map.get_tileWidth() + xDetailShift;
        o.y = o.y * rr_map.get_tileHeight() + yDetailShift;
        ++o.z;
        d.x = d.x * rr_map.get_tileWidth() + xDetailShift;
        d.y = d.y * rr_map.get_tileHeight() + yDetailShift;
        ++d.z;
        output << "(" << o.x << "," << o.y << "," << o.z << ")-(" << d.x << "," << d.y << "," << d.z << ")\n";

    }

// the end of a net of output file
    output << "!\n";
}

int Layer_assignment::klat(int net_id) { //SOLAC + APEC
    const PinptrList& pin_list = rr_map.get_nPin(net_id);

    Coordinate_2d start = pin_list[0]->get_tileXY();
    global_net_id = net_id; // LAZY global variable
    global_pin_num = rr_map.get_netPinNumber(net_id);
    preprocess(net_id);
// Find a pin as starting point
// klat start with a pin
    DP(start, start);

    update_path_for_klat(start);

    return layerInfo_map.vertex(start).klat[0].val;
}

bool Layer_assignment::comp_temp_net_order(int p, int q) {

    return average_order[q].average < average_order[p].average || //
            (!(average_order[p].average < average_order[q].average) && //
                    rr_map.get_netPinNumber(p) < rr_map.get_netPinNumber(q));
}

void Layer_assignment::init_union(const Coordinate_2d& c1, const Coordinate_2d& c2) {
    const Edge_2d& edgeWest = congestion.congestionMap2d.edge(c1, c2);
    for (const std::pair<int, int>& iter : edgeWest.used_net) {
        ++average_order[iter.first].times;
    }
}

void Layer_assignment::find_group(int max) {

// initial for average_order
    average_order.resize(max);
    for (AVERAGE_NODE& a : average_order) {
        a.times = 0;
    }

    for (int i = 1; i < max_xx; ++i) {
        for (int j = 0; j < max_yy; ++j) {
            Coordinate_2d c1 { i, j };
            Coordinate_2d c2 { i - 1, j };
            init_union(c1, c2);
        }
    }
    for (int i = 0; i < max_xx; ++i) {
        for (int j = 1; j < max_yy; ++j) {
            Coordinate_2d c1 { i, j };
            Coordinate_2d c2 { i, j - 1 };
            init_union(c1, c2);
        }
    }

    for (int i = 0; i < max; ++i) {
        AVERAGE_NODE& a = average_order[i];
        a.average = static_cast<double>(rr_map.get_netPinNumber(i)) / static_cast<double>((a.times));
    }
}

void Layer_assignment::calculate_wirelength() {

    int xy = 0;
    int z = 0;

    for (Coordinate_3d c { 0, 0, 0 }; c.x < cur_map_3d.getXSize(); ++c.x) {
        for (c.y = 0; c.y < cur_map_3d.getYSize(); ++c.y) {
            for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
                xy += cur_map_3d.east(c).used_net.size();
                xy += cur_map_3d.south(c).used_net.size();
                z += cur_map_3d.front(c).used_net.size();
            }
        }
    }
    z *= global_via_cost;
    std::cout << "total wirelength = " << xy << " + " << z << " = " << (xy + z) << std::endl;

}

void Layer_assignment::sort_net_order() {

    int max = rr_map.get_netNumber();
// re-distribute net

    find_group(max);
    initial_overflow_map();
    std::vector<int> temp_net_order(max);
    for (int i = 0; i < max; ++i) {
        temp_net_order[i] = i;
    }
    std::sort(temp_net_order.begin(), temp_net_order.end(), [&](int a,int b) {return comp_temp_net_order(a,b);});
    int global_pin_cost = 0;
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < max; ++i) {
        global_pin_cost += klat(temp_net_order[i]);	// others
    }

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;

    std::cout << "cost = " << global_pin_cost << "\n" << std::endl;

    std::cout << "time = " << elapsed_seconds.count() << "\n" << std::endl;

}

void Layer_assignment::calculate_cap() {

    int overflow = 0;
    int max = 0;
    for (const Edge_2d& edge : congestion.congestionMap2d.all()) {
        if (edge.isOverflow()) {
            overflow += (edge.overUsage() * 2);
            if (max < edge.overUsage())
                max = edge.overUsage() * 2;
        }
    }
    printf("2D overflow = %d\n", overflow);
    printf("2D max overflow = %d\n", max);
}

void Layer_assignment::generate_all_output(std::ostream & output) {

    std::vector<std::vector<Segment3d> > comb { static_cast<std::size_t>(rr_map.get_netNumber()) };
    Coordinate_3d c2;
    for (Coordinate_3d c { 0, 0, 0 }; c.x < cur_map_3d.getXSize(); ++c.x) {
        c2.x = c.x;
        for (c.y = 0; c.y < cur_map_3d.getYSize(); ++c.y) {
            c2.y = c.y;
            for (c.z = 1; c.z < cur_map_3d.getZSize(); ++c.z) {
                c2.z = c.z - 1;
                collectComb(c2, c, comb);
            }
        }
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.y = 1; c.y < cur_map_3d.getYSize(); ++c.y) {
                c2.y = c.y - 1;
                collectComb(c2, c, comb);
            }
        }
    }
    for (Coordinate_3d c { 0, 0, 0 }; c.y < cur_map_3d.getYSize(); ++c.y) {
        c2.y = c.y;
        for (c.z = 0; c.z < cur_map_3d.getZSize(); ++c.z) {
            c2.z = c.z;
            for (c.x = 1; c.x < cur_map_3d.getXSize(); ++c.x) {
                c2.x = c.x - 1;
                collectComb(c2, c, comb);
            }
        }
    }

    for (int i = 0; i < rr_map.get_netNumber(); ++i) {
        generate_output(i, comb[i], output);
    }
}

Layer_assignment::Layer_assignment(const Congestion& congestion, const RoutingRegion& rr_map, const std::string& outputFileNamePtr) :
        congestion { congestion }, rr_map { rr_map }, //
        layerInfo_map { congestion.congestionMap2d.getSize() }, //
        cur_map_3d { Coordinate_3d { congestion.congestionMap2d.getSize(), rr_map.get_layerNumber() } } 	//
{

    string outputFileName { outputFileNamePtr };

    global_via_cost = 1;

    max_xx = rr_map.get_gridx();
    max_yy = rr_map.get_gridy();
    //max_zz = rr_map.get_layerNumber();
    malloc_space();

    calculate_cap();
    overflow_max = congestion.find_overflow_max(cur_map_3d.getZSize());
    puts("Layer assignment processing...");

    sort_net_order();
    print_max_overflow();

    puts("Layer assignment complete.");

    calculate_wirelength();
    std::cout << "Outputing result file to " << outputFileName << std::endl;
    generate_all_output(std::cout);
    std::ofstream ofs(outputFileName, std::ofstream::out | std::ofstream::trunc);
    generate_all_output(ofs);
    ofs << std::flush;

}

