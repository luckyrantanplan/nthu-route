#include "Layerassignment.h"

#include <boost/range/combine.hpp>
#include <boost/range/detail/combine_cxx11.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/tuple/detail/tuple_basic.hpp>
#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <queue>
#include <stack>
#include <utility>

#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingComponent.h"
#include "../grdb/RoutingRegion.h"
#include "../spdlog/details/logger_impl.h"
#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "Congestion.h"
#include "DataDef.h"

namespace NTHUR {

void Layer_assignment::initial_overflow_map() {

    for (auto pair : boost::combine(congestion.congestionMap2d.all(), layerInfo_map.edges().all())) {
        pair.get<1>().overflow = pair.get<0>().overUsage() * 2;
    }
}

void Layer_assignment::initLayerInfo(const int max_z) {

    for (LayerInfo& layerInfo : layerInfo_map.allVertex()) {
        layerInfo.path = 0;  // non-visited
        layerInfo.klat.resize(max_z);
    }
    for (EdgeInfo& edge : layerInfo_map.edges().all()) {
        edge.overflow = 0;
        edge.path = 0;
    }

}

void Layer_assignment::update_cur_map_for_klat_xy(int cur_idx, const Coordinate_2d& start, const Coordinate_2d& end, int net_id) {

    Edge_3d& edge = output.cur_map_3d.edge(Coordinate_3d { end, cur_idx }, Coordinate_3d { start, cur_idx });
    ++edge.used_net[net_id];
    edge.cur_cap = (edge.used_net.size() * 2);	// need check
    if (edge.isOverflow()) {	// need check
        layerInfo_map.edges().edge(start, end).overflow -= 2;
    }

}

void Layer_assignment::update_cur_map_for_klat_z(int min, int max, const Coordinate_2d& start, int net_id) {
    Coordinate_3d previous { start, min };
    for (Coordinate_3d k { start, min + 1 }; k.z <= max; ++k.z) {
        Edge_3d& edge = output.cur_map_3d.edge(k, previous);
        ++edge.used_net[net_id];
        ++edge.cur_cap;

        previous = k;
    }

}

void Layer_assignment::update_path_for_klat(const Coordinate_2d& start) {
    int pin_num = 0;
    std::queue<ElementQueue<Coordinate_3d>> q;

// BFS
    q.emplace(Coordinate_3d { start, 0 }, Coordinate_3d { start, 0 });	// enqueue
    while (!q.empty()) {
        ElementQueue<Coordinate_3d>& head3d = q.front();

        Coordinate_2d h = head3d.coor.xy();
        int z_min = 0;
        if (layerInfo_map.vertex(h).path == 2) {	// a pin
            ++pin_num;
        } else {
            z_min = head3d.coor.z;
        }
        int z_max = head3d.coor.z;
        for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(h)) {
            if (handle.vertex() != head3d.parent.xy() && handle.edge().path == 1) {	// check legal
                Coordinate_2d& c = handle.vertex();
                if (layerInfo_map.vertex(c).path == 0) {
                    log_sp->error("inconsistency between traverse node {} and edge check parent ???", c.toString());
                    exit(-1);
                }
                int pi_z = layerInfo_map.vertex(c).klat[head3d.coor.z].pi_z;
                z_max = std::max(z_max, pi_z);
                z_min = std::min(z_min, pi_z);
                update_cur_map_for_klat_xy(pi_z, h, c, global_net_id);
                q.emplace(Coordinate_3d { c, pi_z }, Coordinate_3d { h, pi_z });	// enqueue
            }
        }	//
        update_cur_map_for_klat_z(z_min, z_max, h, global_net_id);
        layerInfo_map.vertex(h).path = 0;	// visited
        q.pop();	// dequeue
    }
    if (pin_num != global_pin_num) {
        log_sp->error("net : {}, pin number error {} vs {}", global_net_id, pin_num, global_pin_num);
        exit(-1);
    }
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

    const std::vector<Net::Pin>& pin_list = output.get_nPin(net_id);

    for (const Net::Pin& pin : pin_list) {
        layerInfo_map.vertex(pin.xy()).path = -2;	// pin
    }
// BFS speed up
    Coordinate_2d c = pin_list[0].xy();
    layerInfo_map.vertex(c).path = 2;	// visited

    for (KLAT_NODE& k : layerInfo_map.vertex(c).klat) {
        k.val = -1;
    }

    std::queue<ElementQueue<Coordinate_2d>> q;
    q.emplace(c, c);	// enqueue
    while (!q.empty()) {
        ElementQueue<Coordinate_2d>& front = q.front();
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
        if (l.output.cur_map_3d.edge(c2, Coordinate_3d { o, c2.z + 1 }).isOverflow()) {
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
        if (e.others >= 0) {
            Coordinate_2d child = neighbors[e.others];
            --e.others;

            if (e.cost <= vertexCost.cost) {
                bool noSolution = true;
                for (std::size_t k = 0; k < layerInfo_map.vertex(child).klat.size(); ++k) {
                    int cost = layerInfo_map.vertex(child).klat[k].val;
                    if (cost >= 0) {
                        ElementStack e1 = e;
                        e1.choice.push_back(Coordinate_3d { child, static_cast<int32_t>(k) });
                        e1.cost += cost;
                        stack.push(std::move(e1));
                        noSolution = false;
                    }

                }
                if (noSolution) {
                    stack.push(std::move(e));  //if there is nothing in the for loop , then the branch is finish, should see the choice value
                }
            }
        } else {
            VertexCost vcost(int_DP_k);
            vcost.addCost(o.xy(), e, *this);
            vcost.vertices = e.choice;
            if (vcost < vertexCost) {
                vertexCost = std::move(vcost);

            }

        }     // current e.val is >   min : we do nothing
    }
    klatNode.via_cost = vertexCost.via_cost;
    klatNode.via_overflow = vertexCost.cost;
    klatNode.val = vertexCost.cost;
    return vertexCost.vertices;  // vertices is empty !!!! should fill it correctly (perhaps with ElementStack choice ? )
}

void Layer_assignment::DP(const Coordinate_3d& c, const Coordinate_3d& parent) {

    Coordinate_2d c1(c.xy());
    KLAT_NODE& klatNode = layerInfo_map.vertex(c1).klat[c.z];

    if (klatNode.val == -1) {	// non-traversed
        bool is_end = true;
        for (EdgePlane<EdgeInfo>::Handle& handle : layerInfo_map.edges().neighbors(c1)) {	// direction
            if (handle.edge().path == 1 && handle.vertex() != parent.xy()) {	// check legal
                is_end = false;
                for (Coordinate_3d c2 { handle.vertex(), 0 }; c2.z < output.cur_map_3d.getZSize(); ++c2.z) {	// use global_max_layer to substitute max_zz

                    Edge_3d& edge = output.cur_map_3d.edge(Coordinate_3d { c1, c2.z }, c2);
                    if (((handle.edge().overflow <= 0) && edge.overUsage() < 0) ||	//
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

                if (output.cur_map_3d.edge(c2, Coordinate_3d { c1, c.z + 1 }).overUsage() >= 0) {
                    ++klatNode.via_overflow;
                }	//

            }
            klatNode.val = klatNode.via_overflow;

        } else {	// is_end == false
            for (const Coordinate_3d& n : rec_count(c, klatNode)) {
                layerInfo_map.vertex(n.xy()).klat[c.z].pi_z = n.z;
            }
        }
    }
}

int Layer_assignment::klat(int net_id) { //SOLAC + APEC

    const std::vector<Net::Pin>& pin_list = output.get_nPin(net_id);

    Coordinate_2d start = pin_list[0].xy();
    global_net_id = net_id; // LAZY global variable
    global_pin_num = pin_list.size();
    preprocess(net_id);
// Find a pin as starting point
// klat start with a pin

    DP(start, start);

    update_path_for_klat(start);
    /*
     log_sp->info("update_path_for_klat  ");
     plotNet(net_id);
     log_sp->info("end update_path_for_klat  ");*/
    return layerInfo_map.vertex(start).klat[0].val;
}

bool Layer_assignment::comp_temp_net_order(int p, int q) {

    return average_order[q].average < average_order[p].average || //
            (!(average_order[p].average < average_order[q].average) && //
                    output.get_nPin(p).size() < output.get_nPin(q).size());
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
        a.average = static_cast<double>(output.get_nPin(i).size()) / static_cast<double>((a.times));
    }
}

void Layer_assignment::sort_net_order() {

    int max = output.get_netNumber();
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

    log_sp->info("cost = {}", global_pin_cost);	//
    log_sp->info("time = {}s", elapsed_seconds.count());

}

Layer_assignment::Layer_assignment(const Congestion& congestion, OutputGeneration& output) :
        congestion { congestion }, //
        output { output }, //
        layerInfo_map { congestion.congestionMap2d.getSize() }  //

{
    log_sp = spdlog::get("NTHUR");

    global_via_cost = 1;

    max_xx = output.cur_map_3d.getXSize();
    max_yy = output.cur_map_3d.getYSize();

    initLayerInfo(output.cur_map_3d.getZSize());

    congestion.calculate_cap();
    overflow_max = congestion.find_overflow_max(output.cur_map_3d.getZSize());
    log_sp->info("Layer assignment processing...");

    sort_net_order();

    output.print_max_overflow();

    output.calculate_wirelength(global_via_cost);

}

} // namespace NTHUR
