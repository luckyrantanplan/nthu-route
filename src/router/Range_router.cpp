#include "Range_router.h"

#include <boost/multi_array/multi_array_ref.hpp>
#include <sys/types.h>
#include <algorithm>

#include "../grdb/EdgePlane.h"
#include "../grdb/RoutingRegion.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "MM_mazeroute.h"

//#define SPDLOG_TRACE_ON
#include "../spdlog/spdlog.h"

bool NTHUR::RangeRouter::double_equal(double a, double b) {
    double diff = a - b;
    if (diff > 0.00001 || diff < -0.00001)
        return false;
    else
        return true;
}

/*sort grid_edge in decending order*/
bool NTHUR::RangeRouter::comp_grid_edge(const Grid_edge_element& a, const Grid_edge_element& b) {
    return congestion.congestionMap2d.edge(a.grid, a.c2).congestion() > congestion.congestionMap2d.edge(b.grid, b.c2).congestion();
}

/*
 determine INTERVAL_NUM(10) intervals between min and max,
 and also compute average congestion value (sum of demand / sum of capacity)
 */
void NTHUR::RangeRouter::define_interval() {

    Congestion::Statistic s = congestion.stat_congestion();

    interval_list[0].begin_value = s.max;

    for (u_int32_t i = 1; i < interval_list.size(); ++i) {
        interval_list[i].begin_value = s.max - ((double) i / interval_list.size()) * (s.max - 1.0);
        interval_list[i - 1].end_value = interval_list[i].begin_value;
    }
    interval_list[interval_list.size() - 1].end_value = 1.;
    for (Interval_element& ele : interval_list) {
        ele.grid_edge_vector.clear();
    }

    SPDLOG_TRACE(print_interval());

}

std::string NTHUR::RangeRouter::print_interval() const {

    std::string s("interval value: ");
    for (uint32_t i = 0; i < interval_list.size(); ++i) {
        s += std::to_string(interval_list[i].begin_value) + " ";
    }
    s += std::to_string(interval_list[interval_list.size() - 1].end_value) + "\n";
    return s;
}

void NTHUR::RangeRouter::insert_to_interval(Coordinate_2d coor_2d, Coordinate_2d c2) {
    double cong_value = congestion.congestionMap2d.edge(coor_2d, c2).congestion();
    if (cong_value > 1) {
        for (int i = interval_list.size() - 1; i >= 0; --i) {
            Interval_element& ele = interval_list[i];
            if (((cong_value < ele.begin_value) || double_equal(cong_value, ele.begin_value)) && //
                    cong_value > ele.end_value) {
                ele.grid_edge_vector.push_back(Grid_edge_element(coor_2d, c2));
                return;
            }
        }
    }
}

void NTHUR::RangeRouter::divide_grid_edge_into_interval() {

    for (int i = 0; i < congestion.congestionMap2d.getXSize() - 1; ++i) {
        for (int j = 0; j < congestion.congestionMap2d.getYSize(); ++j) {
            insert_to_interval(Coordinate_2d { i, j }, Coordinate_2d { i + 1, j });
        }
    }
    for (int i = 0; i < congestion.congestionMap2d.getXSize(); ++i) {
        for (int j = 0; j < congestion.congestionMap2d.getYSize() - 1; ++j) {
            insert_to_interval(Coordinate_2d { i, j }, Coordinate_2d { i, j + 1 });
        }

    }

}

void NTHUR::RangeRouter::walkFrame(const Rectangle& r, std::function<void(Coordinate_2d& i, Coordinate_2d& before)> accumulate) {

    const Coordinate_2d& upLeft = r.upLeft;
    const Coordinate_2d& downRight = r.downRight;

    Coordinate_2d before { upLeft.x, std::min(upLeft.y + 1, downRight.y) };
    {
        Coordinate_2d corner { upLeft.x, upLeft.y };
        accumulate(corner, before);
        before.set(corner);
    }
    for (Coordinate_2d i { upLeft.x + 1, upLeft.y }; i.x < downRight.x; ++i.x) {
        accumulate(i, before);
        before.set(i);
        Coordinate_2d center { i.x, std::min(upLeft.y + 1, downRight.y) };
        accumulate(i, center);
    }

    {
        Coordinate_2d corner { downRight.x, upLeft.y };
        accumulate(corner, before);
        before.set(corner);
    }

    for (Coordinate_2d i { downRight.x, upLeft.y + 1 }; i.y < downRight.y; ++i.y) {
        accumulate(i, before);
        before.set(i);
        Coordinate_2d center { std::max(upLeft.x, downRight.x - 1), i.y };
        accumulate(i, center);

    }

    if (!downRight.isAligned(upLeft)) {
        {
            Coordinate_2d corner { downRight.x, downRight.y };
            accumulate(corner, before);
            before.set(corner);
        }
        for (Coordinate_2d i { downRight.x - 1, downRight.y }; i.x > upLeft.x; --i.x) {
            accumulate(i, before);
            before.set(i);
            if (upLeft.y < downRight.y - 1) {
                Coordinate_2d center { i.x, downRight.y - 1 };
                accumulate(i, center);
            }
        }
        {
            Coordinate_2d corner { upLeft.x, downRight.y };
            accumulate(corner, before);
            before.set(corner);
        }
        for (Coordinate_2d i { upLeft.x, downRight.y - 1 }; i.y > upLeft.y; --i.y) {
            accumulate(i, before);
            before.set(i);
            if (upLeft.x + 1 < downRight.x) {
                Coordinate_2d center { upLeft.x + 1, i.y };
                accumulate(i, center);
            }
        }
    }

}

void NTHUR::RangeRouter::expand_range(Coordinate_2d c1, Coordinate_2d c2, int interval_index) {

    Rectangle r { c1, c2 };
    Rectangle bound { Coordinate_2d { 0, 0 }, congestion.congestionMap2d.getSize() + Coordinate_2d { -1, -1 } };

    double total_cong = 0;
    int edge_num = 0;

    while (total_cong >= edge_num * interval_list[interval_index].end_value && !r.contains(bound)) {

        walkFrame(r, [&](Coordinate_2d& i,Coordinate_2d& before) {
            if (before != i) {
                if (bound.contains(i) && bound.contains(before)) {
                    total_cong += congestion.congestionMap2d.edge(i, before).congestion();
                    ++edge_num;
                    colorMap[i.x][i.y].expand = interval_index;
                }
            }
        });
        r.expand(1);
        SPDLOG_TRACE(log_sp, "r after  r.expand(1): {}", r.toString());
    }                                // end of while loop
    SPDLOG_TRACE(log_sp, "r: {} bound: {}", r.toString(), bound.toString());SPDLOG_TRACE(log_sp, "printIfBound:{}", printIfBound(r, bound, interval_index, c1, c2));

    r.expand(congestion.cur_iter / 10); // extraExpandRange
    bound.clip(r);
    SPDLOG_TRACE(log_sp, "r after clip: {}  bound: {}", r.toString(), bound.toString());
    range_vector.push_back(r);
}

std::string NTHUR::RangeRouter::printIfBound(const Rectangle& r, const Rectangle& bound, const int interval_index, const Coordinate_2d& c1, const Coordinate_2d& c2) const {
    std::string s;
    if (r.contains(bound)) {
        s += "for interval " + std::to_string(interval_index);
        s += ", its range is equal to the grid size, ";
        s += r.toString();
        s += " from edge (" + c1.toString() + ") (" + c2.toString() + ")";

    }
    return s;
}
//Rip-up the path that pass any overflowed edges, then route with monotonic 
//routing or multi-source multi-sink routing.
//If there is no overflowed path by using the two methods above, then remain 
//the original path.
void NTHUR::RangeRouter::range_router(Two_pin_element_2d& two_pin, int version) {
    if (!congestion.check_path_no_overflow(two_pin.path, two_pin.net_id, false)) {
        ++total_twopin;

        construct_2d_tree.NetDirtyBit[two_pin.net_id] = true;

        congestion.update_congestion_map_remove_two_pin_net(two_pin.path, two_pin.net_id);

        std::vector<Coordinate_2d> bound_path(two_pin.path);

        Bound bound;
        bool find_path_flag = monotonicRouter.monotonicRoute(two_pin, bound, bound_path);

        if (version == 2) {
            two_pin.done = construct_2d_tree.done_iter;
        }
        if ((find_path_flag == false) || !congestion.check_path_no_overflow(bound_path, two_pin.net_id, true)) {
            Coordinate_2d start;
            Coordinate_2d end;

            start.x = min(two_pin.pin1.x, two_pin.pin2.x);
            start.y = min(two_pin.pin1.y, two_pin.pin2.y);
            end.x = max(two_pin.pin1.x, two_pin.pin2.x);
            end.y = max(two_pin.pin1.y, two_pin.pin2.y);

            int size = construct_2d_tree.BOXSIZE_INC;
            start.x = max(0, start.x - size);
            start.y = max(0, start.y - size);
            end.x = min(construct_2d_tree.rr_map.get_gridx() - 1, end.x + size);
            end.y = min(construct_2d_tree.rr_map.get_gridy() - 1, end.y + size);

            find_path_flag = construct_2d_tree.mazeroute_in_range.mm_maze_route_p(two_pin, bound.cost, bound.distance, bound.via_num, start, end, version);

            if (find_path_flag == false) {
                two_pin.path.insert(two_pin.path.begin(), bound_path.begin(), bound_path.end());
            }
        }

        congestion.update_congestion_map_insert_two_pin_net(two_pin);

    }
}

void NTHUR::RangeRouter::query_range_2pin(const Rectangle& r, //
        std::vector<Two_pin_element_2d*>& twopin_list, boost::multi_array<Point_fc, 2>& gridCell) {

    static int done_counter = 0;	//only initialize once

    for (int x = r.upLeft.x; x <= r.downRight.x; ++x) {
        for (int y = r.upLeft.y; y <= r.downRight.y; ++y) {
            Point_fc& cell = (gridCell[x][y]);

            for (Two_pin_element_2d* twopin : cell.points) {   //for each pin or steiner point
                if (twopin->done != construct_2d_tree.done_iter) {
                    Coordinate_2d& p1 = twopin->pin1;
                    Coordinate_2d& p2 = twopin->pin2;
                    if (colorMap[p1.x][p1.y].routeState != done_counter && //
                            colorMap[p2.x][p2.y].routeState != done_counter) {
                        if (r.contains(p1) || r.contains(p2)) {
                            twopin->done = construct_2d_tree.done_iter;
                            twopin_list.push_back(twopin);
                        }
                    }
                }
            }
            colorMap[cell.x][cell.y].routeState = done_counter;
        }
    }

    ++done_counter;
}

void NTHUR::RangeRouter::specify_all_range(boost::multi_array<Point_fc, 2>& gridCell) {
    std::vector<Two_pin_element_2d *> twopin_list;
    std::vector<int> twopin_range_index_list;

    for (u_int32_t i = 0; i < colorMap.num_elements(); ++i) {
        colorMap.data()[i].set(-1, -1);
    }

    total_twopin = 0;

    for (int i = interval_list.size() - 1; i >= 0; --i) {
        Interval_element& ele = interval_list[i];
        range_vector.clear();
        sort(ele.grid_edge_vector.begin(), ele.grid_edge_vector.end(), [&](const Grid_edge_element& a, const Grid_edge_element& b) {
            return comp_grid_edge( a, b);
        });

        for (Grid_edge_element& gridEdge : ele.grid_edge_vector) {
            Coordinate_2d& c = gridEdge.grid;
            Coordinate_2d& nei = gridEdge.c2;

            if ((colorMap[c.x][c.y].expand != i) || (colorMap[nei.x][nei.y].expand != i)) {
                colorMap[c.x][c.y].expand = i;
                colorMap[nei.x][nei.y].expand = i;

                expand_range(c, nei, i);
            }
        }

        twopin_list.clear();
        twopin_range_index_list.clear();
        for (Rectangle r : range_vector) {
            query_range_2pin(r, twopin_list, gridCell);
        }

        sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
            return Two_pin_element_2d::comp_stn_2pin(*a,*b);});

        for (Two_pin_element_2d * two_pin : twopin_list) {

            range_router(*two_pin, 2);

        }
    }

    twopin_list.clear();
    int length = construct_2d_tree.two_pin_list.size();
    for (int i = 0; i < length; ++i) {
        if (construct_2d_tree.two_pin_list[i].done != construct_2d_tree.done_iter) {
            twopin_list.push_back(&construct_2d_tree.two_pin_list[i]);
        }
    }

    sort(twopin_list.begin(), twopin_list.end(), [&](const Two_pin_element_2d *a, const Two_pin_element_2d *b) {
        return Two_pin_element_2d::comp_stn_2pin(*a,*b);});
    for (int i = 0; i < (int) twopin_list.size(); ++i) {
        if (twopin_list[i]->boxSize() == 1)
            break;
        range_router(*twopin_list[i], 2);
    }

    construct_2d_tree.mazeroute_in_range.clear_net_tree();
}

NTHUR::RangeRouter::RangeRouter(Construct_2d_tree& construct2dTree, Congestion& congestion, bool monotonic_enable) :
        total_twopin(0),	//

        construct_2d_tree { construct2dTree }, //
        congestion { congestion }, //
        colorMap { boost::extents[congestion.congestionMap2d.getXSize()][congestion.congestionMap2d.getYSize()] }, monotonicRouter { congestion, monotonic_enable } {
    log_sp = spdlog::get("NTHUR");

}

