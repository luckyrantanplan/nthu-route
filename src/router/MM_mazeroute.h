#ifndef INC_MM_MAZEROUTE_H
#define INC_MM_MAZEROUTE_H

#include <boost/heap/pairing_heap.hpp>
#include <boost/multi_array.hpp>
#include <vector>

#include "../misc/geometry.h"
#include "DataDef.h"

class Congestion;

struct Construct_2d_tree;

using namespace std;

class Two_pin_element_2d;

class Multisource_multisink_mazeroute {
private:
    class Vertex_mmm {
    public:
        Coordinate_2d coor;

        vector<Vertex_mmm*> neighbor;
        int visit;

        Vertex_mmm(const Coordinate_2d& xy);

    };

    class MMM_element {

        class MMM_element_greater {

        public:

            bool operator()(const MMM_element* lhs, const MMM_element* rhs) const {

                if ((lhs->reachCost - rhs->reachCost) < neg_error_bound) {
                    return false;
                } else if ((lhs->reachCost - rhs->reachCost) > error_bound) {
                    return true;
                } else {
                    if (lhs->distance < rhs->distance)
                        return false;
                    else if (lhs->distance > rhs->distance)
                        return true;
                    else
                        return lhs->via_num > rhs->via_num;
                }

            }
        };
    public:

        typedef boost::heap::pairing_heap<MMM_element*, boost::heap::compare<MMM_element_greater>> MMMPriortyQueue;
        typedef MMMPriortyQueue::handle_type HandleType;

        Coordinate_2d coor;
        MMM_element *parent;
        double reachCost;       //Cost from source to current element
        int distance;           //Distance from source to current element
        int via_num;            //Via count from source to current element
        int visit;              //default: -1. If the element be visited, this value
                                //will be set to current iteration ID (visit_counter)
        int dst;                //defalut: -1. If the element is a sink, this value
                                //will be set to current dst ID (dst_counter)
        int walkableID;         //If this element is walkable, then ID = visit_counter
        HandleType handle;            //Index in MMMPriortyQueue

    public:
        MMM_element() :
                coor { }, parent(nullptr), reachCost(0.), distance(0), via_num(0), visit(-1), dst(-1), walkableID(-1), handle { } {
        }

    };

public:
    Multisource_multisink_mazeroute(Construct_2d_tree& construct_2d_tree, Congestion& congestion);
    ~Multisource_multisink_mazeroute();
    bool mm_maze_route_p(Two_pin_element_2d&element, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d& start, Coordinate_2d& end, int version);
    void clear_net_tree();

private:
    void setup_pqueue();
    void find_subtree(Vertex_mmm *v, int mode);
    void adjust_twopin_element();
    void trace_back_to_find_path_2d(MMM_element *end_point);

    //Cache System
    void putNetOnColorMap();
    void bfsSetColorMap(int x, int y);

    bool smaller_than_lower_bound(double total_cost, int distance, int via_num, double bound_cost, int bound_distance, int bound_via_num);

private:
    Construct_2d_tree& construct_2d_tree;
    Congestion& congestion;
    boost::multi_array<MMM_element, 2> mmm_map;

    vector<vector<Vertex_mmm> > net_tree;
    MMM_element::MMMPriortyQueue pqueue;
    Two_pin_element_2d *element;
    Vertex_mmm* pin1_v;
    Vertex_mmm* pin2_v;	//source,destination
    int visit_counter;
    int dst_counter;

    int gridxMinusOne;
    int gridyMinusOne;
};

#endif //INC_MM_MAZEROUTE_H
