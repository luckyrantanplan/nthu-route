#include "MM_mazeroute.h"
#include "../misc/geometry.h"
#include "../misc/debug.h"
#include "Post_processing.h"
#include <iostream>
#include <climits>
#include <stack>
#include <queue>

using namespace Jm;
using namespace std;

void Multisource_multisink_mazeroute::MMMPriortyQueue::init() {
    storage_ = new vector<MMM_element*>(2 * rr_map->get_gridx() * rr_map->get_gridy(), static_cast<MMM_element*>(NULL));
    size_ = 0;
}

Multisource_multisink_mazeroute::Vertex_mmm::Vertex_mmm(int x, int y) :
        coor(&coor_array[x][y]), visit(-1) {
}

Multisource_multisink_mazeroute::Multisource_multisink_mazeroute() :
        gridxMinusOne(rr_map->get_gridx() - 1), gridyMinusOne(rr_map->get_gridy() - 1) {
    /*allocate space for mmm_map*/
    this->net_tree = new vector<vector<Vertex_mmm *> >(rr_map->get_netNumber(), vector<Vertex_mmm *>(0));

    mmm_map = new VertexPlane<MMM_element>(rr_map->get_gridx(), rr_map->get_gridy(), MMM_element());

    pqueue = new MMMPriortyQueue();

    //initialization
    for (int i = 0; i < rr_map->get_gridx(); ++i) {
        for (int j = 0; j < rr_map->get_gridy(); ++j) {
            mmm_map->vertex(i, j).coor = &coor_array[i][j];
        }
    }

    this->visit_counter = 0;
    this->dst_counter = 0;
}

Multisource_multisink_mazeroute::~Multisource_multisink_mazeroute() {
    delete this->net_tree;
    delete this->mmm_map;
    delete pqueue;
}

/*recursively traverse parent in maze_routing_map to find path*/
void Multisource_multisink_mazeroute::trace_back_to_find_path_2d(MMM_element *end_point) {
    MMM_element *cur_pos;

    cur_pos = end_point;
    while (1) {
        if (cur_pos == NULL)
            break;
        this->element->path.push_back(cur_pos->coor);
        cur_pos = cur_pos->parent;
    }
}

//store new 2pins and adjust dfs tree
void Multisource_multisink_mazeroute::adjust_twopin_element() {

    Coordinate_2d* new_pin1 = this->element->path.front();
    Coordinate_2d* new_pin2 = this->element->path.back();
    this->element->pin1 = coor_array[new_pin1->x][new_pin1->y];
    this->element->pin2 = coor_array[new_pin2->x][new_pin2->y];

    vector<Vertex_mmm *>::iterator it;
    int flag = 0;
    for (it = this->pin1_v->neighbor.begin(); it != this->pin1_v->neighbor.end(); ++it) {
        if ((*it) == this->pin2_v) {
            this->pin1_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    flag = 0;
    for (it = this->pin2_v->neighbor.begin(); it != this->pin2_v->neighbor.end(); ++it) {
        if ((*it) == this->pin1_v) {
            this->pin2_v->neighbor.erase(it);
            flag = 1;
            break;
        }
    }
    assert(flag == 1);

    int net_id = this->element->net_id;
    Vertex_mmm* v1 = NULL;
    Vertex_mmm* v2 = NULL;
    ;
    for (vector<Vertex_mmm *>::iterator it = (*this->net_tree)[net_id].begin(); it != (*this->net_tree)[net_id].end() && (v1 == NULL || v2 == NULL); ++it) {
        if ((*it)->coor == new_pin1) {
            v1 = (*it);
        } else if ((*it)->coor == new_pin2) {
            v2 = (*it);
        }
    }
    assert(v1 != NULL);
    assert(v2 != NULL);

    v1->neighbor.push_back(v2);
    v2->neighbor.push_back(v1);
}

void Multisource_multisink_mazeroute::find_subtree(Vertex_mmm *v, int mode) {
    v->visit = this->visit_counter;

    if (mode == 0) {
        MMM_element *cur = &mmm_map->vertex(v->coor->x, v->coor->y);
        cur->reachCost = 0;
        cur->distance = 0;
        cur->via_num = 0;
        cur->parent = NULL;
        cur->visit = this->visit_counter;
        pqueue->insert(cur);
    } else {
        mmm_map->vertex(v->coor->x, v->coor->y).dst = this->dst_counter;
    }
    for (vector<Vertex_mmm *>::iterator it = v->neighbor.begin(); it != v->neighbor.end(); ++it) {
        if ((*it)->visit != this->visit_counter)
            find_subtree((*it), mode);
    }
}

void Multisource_multisink_mazeroute::clear_net_tree() {
    for (int i = 0; i < rr_map->get_netNumber(); ++i) {
        int length = (*this->net_tree)[i].size();
        for (int j = 0; j < length; ++j)
            delete ((*this->net_tree)[i][j]);
    }

    delete net_tree;
    this->net_tree = new vector<vector<Vertex_mmm *> >(rr_map->get_netNumber(), vector<Vertex_mmm *>(0));
}

void Multisource_multisink_mazeroute::setup_pqueue() {

    int cur_net = this->element->net_id;
    if ((*this->net_tree)[cur_net].empty()) {
        Tree* t = &net_flutetree[cur_net];
        for (int i = 0; i < t->number; ++i) {
            (*this->net_tree)[cur_net].push_back(new Vertex_mmm((int) t->branch[i].x, (int) t->branch[i].y));
        }
        for (int i = 0; i < t->number; ++i) {
            Vertex_mmm* a = (*this->net_tree)[cur_net][i];
            Vertex_mmm* b = (*this->net_tree)[cur_net][t->branch[i].n];
            if (a->coor->x == b->coor->x && a->coor->y == b->coor->y)
                continue;
            a->neighbor.push_back(b);
            b->neighbor.push_back(a);
        }
    }

    pqueue->clear();

    //find pin1 and pin2 in tree
    this->pin1_v = this->pin2_v = NULL;
    for (vector<Vertex_mmm *>::iterator it = (*this->net_tree)[cur_net].begin(); (it != (*this->net_tree)[cur_net].end()) && (this->pin1_v == NULL || this->pin2_v == NULL); ++it) {
        if ((*it)->coor->x == this->element->pin1.x && (*it)->coor->y == this->element->pin1.y) {
            this->pin1_v = (*it);
            this->pin1_v->visit = this->visit_counter;
        } else if ((*it)->coor->x == this->element->pin2.x && (*it)->coor->y == this->element->pin2.y) {
            this->pin2_v = (*it);
            this->pin2_v->visit = this->visit_counter;
        }
    }

    assert(this->pin1_v != NULL);
    assert(this->pin2_v != NULL);

    this->find_subtree(this->pin1_v, 0);	//source
    this->find_subtree(this->pin2_v, 1);	//destination
}

void Multisource_multisink_mazeroute::bfsSetColorMap(int x, int y) {
    int net_id = this->element->net_id;
    stack<pair<int, int> > Q;

    Q.push(make_pair(x, y));
    while (!Q.empty()) {
        x = Q.top().first;
        y = Q.top().second;
        Q.pop();
        mmm_map->vertex(x, y).walkableID = visit_counter;

        for (int dir = 3; dir >= 0; --dir) {
            if ((dir == 3 && x >= gridxMinusOne) || (dir == 2 && x <= 0) || (dir == 1 && y <= 0) || (dir == 0 && y >= gridyMinusOne))
                continue;
            else {
                DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[dir]);
                if (cache->edge(x, y, dirType).MMVisitFlag != visit_counter && congestionMap2d->edge(x, y, dirType).lookupNet(net_id)) {
                    cache->edge(x, y, dirType).MMVisitFlag = visit_counter;
                    Q.push(make_pair(x + dir_array[dir][0], y + dir_array[dir][1]));
                }
            }
        }
    }
}
bool Multisource_multisink_mazeroute::mm_maze_route_p2(Two_pin_element_2d *element, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d start, Coordinate_2d end) {
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;
    element->path.clear();
    this->element = element;
    this->boundary_l = start.x;
    this->boundary_b = start.y;
    this->boundary_r = end.x;
    this->boundary_t = end.y;
    this->setup_pqueue();
    this->putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

    while (!this->pqueue->empty()) {
        cur_pos = pqueue->top();
        pqueue->pop();

        if (cur_pos->parent != NULL) {
            pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

        for (int i = 3; i >= 0; --i) {
            if ((i == 3 && cur_pos->coor->x >= gridxMinusOne) || (i == 2 && cur_pos->coor->x <= 0) || (i == 1 && cur_pos->coor->y <= 0) || (i == 0 && cur_pos->coor->y >= gridyMinusOne))
                continue;
            else {
                next_pos = &mmm_map->vertex(cur_pos->coor->x + dir_array[i][0], cur_pos->coor->y + dir_array[i][1]);

                if (next_pos != cur_pos->parent && next_pos->walkableID == visit_counter) {
                    DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[i]);
                    double reachCost = cur_pos->reachCost;
                    int total_distance = cur_pos->distance;
                    int via_num = cur_pos->via_num;
                    bool addDistance = false;
                    if (cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) {
                        reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                        ++total_distance;
                        addDistance = true;
                    }

                    if (((i & 0x02) != pre_dir) && (cur_pos->parent != NULL)) {
                        via_num += 3;
                        if (addDistance) {
                            total_distance += 3;
                            reachCost += via_cost;
                        }
                    }

                    bool needUpdate = false;
                    if (next_pos->visit != this->visit_counter) {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, bound_cost, bound_distance, bound_via_num)) {
                            needUpdate = true;
                        }
                    } else {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, next_pos->reachCost, next_pos->distance, next_pos->via_num)) {
                            needUpdate = true;
                        }
                    }

                    if (needUpdate == true) {
                        next_pos->parent = cur_pos;
                        next_pos->reachCost = reachCost;
                        next_pos->distance = total_distance;
                        next_pos->via_num = via_num;
                        next_pos->visit = this->visit_counter;

                        if (next_pos->dst == this->dst_counter) {
                            bound_cost = reachCost;
                            bound_distance = total_distance;
                            bound_via_num = via_num;
                            sink_pos = next_pos;
                        } else {
                            pqueue->update(next_pos);
                        }
                    }
                }
            }
        }            //end of direction for-loop

        if (sink_pos != NULL) {
            find_path_flag = true;
            this->trace_back_to_find_path_2d(sink_pos);
            this->adjust_twopin_element();
            break;
        }
    }

    ++this->visit_counter;
    ++this->dst_counter;
    return find_path_flag;
}

bool Multisource_multisink_mazeroute::mm_maze_route_p3(Two_pin_element_2d *element, double bound_cost, int bound_distance, int bound_via_num, Coordinate_2d start, Coordinate_2d end) {
    int pre_dir = 0;
    bool find_path_flag = false;
    MMM_element* cur_pos = NULL;
    MMM_element* next_pos = NULL;
    MMM_element* sink_pos = NULL;

    element->path.clear();
    this->element = element;
    this->boundary_l = start.x;
    this->boundary_b = start.y;
    this->boundary_r = end.x;
    this->boundary_t = end.y;
    this->setup_pqueue();
    this->putNetOnColorMap();

    for (int x = boundary_l; x <= boundary_r; ++x) {
        for (int y = boundary_b; y <= boundary_t; ++y) {
            mmm_map->vertex(x, y).walkableID = visit_counter;
        }
    }

    while (!this->pqueue->empty()) {
        cur_pos = pqueue->top();
        pqueue->pop();

        if (cur_pos->parent != NULL) {
            pre_dir = get_direction_2d_simple(cur_pos->coor, cur_pos->parent->coor);
            //We only need horizontal or vertical information of the direction,
            //so we can &0x02 first.
            //pre_dir &= 0x02;
        }

        for (int i = 3; i >= 0; --i) {
            if ((i == 3 && cur_pos->coor->x >= gridxMinusOne) || (i == 2 && cur_pos->coor->x <= 0) || (i == 1 && cur_pos->coor->y <= 0) || (i == 0 && cur_pos->coor->y >= gridyMinusOne))
                continue;
            else {
                next_pos = &mmm_map->vertex(cur_pos->coor->x + dir_array[i][0], cur_pos->coor->y + dir_array[i][1]);

                if (next_pos != cur_pos->parent && next_pos->walkableID == visit_counter) {
                    DirectionType dirType = static_cast<DirectionType>(Jr2JmDirArray[i]);
                    double reachCost = cur_pos->reachCost;
                    int total_distance = cur_pos->distance;
                    int via_num = cur_pos->via_num;

                    if ((cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).MMVisitFlag != this->visit_counter) && (cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost != 0.0)) {
                        reachCost += cache->edge(cur_pos->coor->x, cur_pos->coor->y, dirType).cost;
                        ++total_distance;
                    }

                    if (((i & 0x02) != pre_dir) && (cur_pos->parent != NULL)) {
                        via_num += 3;
                    }

                    bool needUpdate = false;
                    if (next_pos->visit != this->visit_counter) {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, bound_cost, bound_distance, bound_via_num)) {
                            needUpdate = true;
                        }
                    } else {
                        if (smaller_than_lower_bound(reachCost, total_distance, via_num, next_pos->reachCost, next_pos->distance, next_pos->via_num)) {
                            needUpdate = true;
                        }
                    }

                    if (needUpdate == true) {
                        next_pos->parent = cur_pos;
                        next_pos->reachCost = reachCost;
                        next_pos->distance = total_distance;
                        next_pos->via_num = via_num;
                        next_pos->visit = this->visit_counter;

                        if (next_pos->dst == this->dst_counter) {
                            bound_cost = reachCost;
                            bound_distance = total_distance;
                            bound_via_num = via_num;
                            sink_pos = next_pos;
                        } else {
                            pqueue->update(next_pos);
                        }
                    }
                }
            }
        }            //end of direction for-loop

        if (sink_pos != NULL) {
            find_path_flag = true;
            this->trace_back_to_find_path_2d(sink_pos);
            this->adjust_twopin_element();
            break;
        }
    }

    ++this->visit_counter;
    ++this->dst_counter;
    return find_path_flag;
}

/****************
 MMMPriortyQueue
 ***************/
Multisource_multisink_mazeroute::MMMPriortyQueue::MMMPriortyQueue() :
        storage_(NULL) {
    init();
}

Multisource_multisink_mazeroute::MMMPriortyQueue::~MMMPriortyQueue() {
    close();
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::close() {
    if (storage_ != NULL) {
        for (int ite = size_ - 1; ite >= 0; --ite) {
            (*storage_)[ite]->pqIndex = -1;
        }
        size_ = 0;
        delete storage_;
    }
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::pop() {
    assert(empty() == false);

    int storageSizeMinusOne = size_ - 1;

    (*storage_)[0]->pqIndex = -1;

    if (0 < storageSizeMinusOne) {
        (*storage_)[0] = (*storage_)[storageSizeMinusOne];
        (*storage_)[0]->pqIndex = 0;
    }

    int parentIndex = 0;
    while (parentIndex < storageSizeMinusOne) {
        register int LChildIndex = (parentIndex << 1) + 1;

        if (LChildIndex >= storageSizeMinusOne)
            break;

        int updateIndex = LChildIndex;
        register int RChildIndex = LChildIndex + 1;

        if (RChildIndex >= storageSizeMinusOne) {
            if (!compareMMM((*storage_)[LChildIndex], (*storage_)[parentIndex])) {
                break;
            }
        } else {
            if (!compareMMM((*storage_)[LChildIndex], (*storage_)[RChildIndex])) {
                updateIndex = RChildIndex;
            }
        }

        swap((*storage_)[parentIndex], (*storage_)[updateIndex]);
        (*storage_)[parentIndex]->pqIndex = parentIndex;
        (*storage_)[updateIndex]->pqIndex = updateIndex;

        parentIndex = updateIndex;
    }

    //Remove the last node from the array
    --size_;
}

void Multisource_multisink_mazeroute::MMMPriortyQueue::update(int indexToUpdate) {
    while (indexToUpdate > 0) {
        int parsentIndex = (indexToUpdate - 1) >> 1;
        if (compareMMM((*storage_)[parsentIndex], (*storage_)[indexToUpdate]))
            break;
        swap((*storage_)[parsentIndex], (*storage_)[indexToUpdate]);
        (*storage_)[parsentIndex]->pqIndex = parsentIndex;
        (*storage_)[indexToUpdate]->pqIndex = indexToUpdate;
        indexToUpdate = parsentIndex;
    }
}

bool Multisource_multisink_mazeroute::MMMPriortyQueue::comp_mmm_element::operator()(const MMM_element* a, const MMM_element* b) const {
    //If A has lower cost, return true, else return false

    if ((a->reachCost - b->reachCost) < neg_error_bound) {
        return true;
    } else if ((a->reachCost - b->reachCost) > error_bound) {
        return false;
    } else {
        if (a->distance < b->distance)
            return true;
        else if (a->distance > b->distance)
            return false;
        else
            return a->via_num < b->via_num;
    }
}

inline
void Multisource_multisink_mazeroute::putNetOnColorMap() {
    this->bfsSetColorMap(pin1_v->coor->x, pin1_v->coor->y);
    this->bfsSetColorMap(pin2_v->coor->x, pin2_v->coor->y);
}

//Inine Functions
/****************
 MMMPriortyQueue
 ***************/

inline
void Multisource_multisink_mazeroute::MMMPriortyQueue::clear() {
    for (PQStorage::iterator ite = storage_->begin(); ite != storage_->begin() + size_; ++ite) {
        (*ite)->pqIndex = -1;
    }
    size_ = 0;
}

inline
bool Multisource_multisink_mazeroute::MMMPriortyQueue::empty() {
    return (size_ == 0);
}
