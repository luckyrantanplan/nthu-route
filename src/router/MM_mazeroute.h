#ifndef INC_MM_MAZEROUTE_H
#define INC_MM_MAZEROUTE_H

#include <stddef.h>
#include <cassert>
#include <vector>

#include "../grdb/plane.h"
#include "../misc/geometry.h"

#include "Construct_2d_tree.h"

using namespace std;

class Two_pin_element_2d;

class Multisource_multisink_mazeroute {
private:
    class Vertex_mmm {
    public:
        Jm::Coordinate_2d *coor;
        vector<Vertex_mmm *> neighbor;
        int visit;

        Vertex_mmm(int x, int y);
    };

    class MMM_element {
    public:
        Jm::Coordinate_2d *coor;
        MMM_element *parent;
        double reachCost;       //Cost from source to current element
        int distance;           //Distance from source to current element
        int via_num;            //Via count from source to current element
        int visit;              //default: -1. If the element be visited, this value
                                //will be set to current iteration ID (visit_counter)
        int dst;                //defalut: -1. If the element is a sink, this value
                                //will be set to current dst ID (dst_counter)
        int walkableID;         //If this element is walkable, then ID = visit_counter
        int pqIndex;            //Index in MMMPriortyQueue

    public:
        MMM_element() :
                coor(NULL), parent(NULL), reachCost(0.), distance(0), via_num(0), visit(-1), dst(-1), walkableID(-1), pqIndex(-1) {
        }
    };

    class MMMPriortyQueue {
        typedef std::vector<MMM_element*> PQStorage;

    public:
        class comp_mmm_element {
        public:
            bool operator()(const MMM_element* a, const MMM_element* b) const;
        };

    public:
        MMMPriortyQueue();

        ~MMMPriortyQueue();

        void push(MMM_element*);

        void insert(MMM_element*);

        MMM_element* top();

        MMM_element* begin();

        void pop();

        void update(MMM_element*);

        bool empty();

        void clear();

    private:

        Construct_2d_tree& construct_2d_tree;
        PQStorage* storage_;
        // TroyLee
        size_t size_;
        // TroyLee
        comp_mmm_element compareMMM;

        void init();
        void close();
        void update(int indexToUpdate);

    };

public:
    Multisource_multisink_mazeroute();
    ~Multisource_multisink_mazeroute();
    bool mm_maze_route_p2(Two_pin_element_2d *element, double bound_cost, int bound_distance, int bound_via_num, Jm::Coordinate_2d start, Jm::Coordinate_2d end);
    bool mm_maze_route_p3(Two_pin_element_2d *element, double bound_cost, int bound_distance, int bound_via_num, Jm::Coordinate_2d start, Jm::Coordinate_2d end);
    void clear_net_tree();

private:
    void setup_pqueue();
    void find_subtree(Vertex_mmm *v, int mode);
    void adjust_twopin_element();
    void trace_back_to_find_path_2d(MMM_element *end_point);

    //Cache System
    void putNetOnColorMap();
    void bfsSetColorMap(int x, int y);

private:
    VertexPlane<MMM_element> *mmm_map;
    vector<vector<Vertex_mmm*> > *net_tree;
    MMMPriortyQueue* pqueue;
    Two_pin_element_2d *element;
    Vertex_mmm* pin1_v;
    Vertex_mmm* pin2_v;	//source,destination
    int visit_counter;
    int dst_counter;
    int boundary_l;
    int boundary_r;
    int boundary_b;
    int boundary_t;
    int gridxMinusOne;
    int gridyMinusOne;
};

inline
void Multisource_multisink_mazeroute::MMMPriortyQueue::push(MMM_element* m) {
    m->pqIndex = size_;
    (*storage_)[size_] = m;
    ++size_;
    update(m->pqIndex);
}

inline
void Multisource_multisink_mazeroute::MMMPriortyQueue::insert(MMM_element* m) {
    push(m);
}

inline Multisource_multisink_mazeroute::MMM_element*
Multisource_multisink_mazeroute::MMMPriortyQueue::top() {
    assert(empty() == false);
    return (*storage_)[0];
}

inline
void Multisource_multisink_mazeroute::MMMPriortyQueue::update(MMM_element* m) {
    if (m->pqIndex < 0)
        push(m);
    else
        update(m->pqIndex);
}
#endif //INC_MM_MAZEROUTE_H
