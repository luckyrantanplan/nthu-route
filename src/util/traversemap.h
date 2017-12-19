#ifndef INC_TRAVERSE_MAP_H
#define INC_TRAVERSE_MAP_H

#include <list>
#include <utility>

#include "../grdb/EdgePlane.h"
#include "../misc/geometry.h"

typedef std::pair<int, int> IntPair;

class BSearchQueNode {
private:
    //cell information, such as x, y, z coordinate
    Coordinate_3d cell;
    //position of current wire segment in wire segments list
    int wirePos;
public:
    //Constructor
    BSearchQueNode(int x, int y, int z, int wirePos);
    //GET Functions
    int get_x();
    int get_y();
    int get_z();
    int get_wirePos();
};

class BSearchQue {
private:
    std::list<BSearchQueNode> que;
public:
    //SET Functions
    void push_back(int x, int y, int z, int wirePos);
    //remove the first cell
    void pop_front();
    //GET Functions
    int nextX();
    int nextY();
    int nextZ();
    int size();
    int get_wirePos();
};

// Inline Functions
/************
 * BSearchQue
 ************/
inline
void BSearchQue::push_back(int x, int y, int z, int wirePos) {
    this->que.push_back(BSearchQueNode(x, y, z, wirePos));
}

inline
void BSearchQue::pop_front() {
    this->que.pop_front();
}

inline
int BSearchQue::nextX() {
    return this->que.front().get_x();
}

inline
int BSearchQue::nextY() {
    return this->que.front().get_y();
}

inline
int BSearchQue::nextZ() {
    return this->que.front().get_z();
}

inline
int BSearchQue::size() {
    return this->que.size();
}

inline
int BSearchQue::get_wirePos() {
    return this->que.front().get_wirePos();
}

/****************
 * BSearchQueNode
 ****************/
inline
int BSearchQueNode::get_x() {
    return this->cell.x;
}

inline
int BSearchQueNode::get_y() {
    return this->cell.y;
}

inline
int BSearchQueNode::get_z() {
    return this->cell.z;
}

inline
int BSearchQueNode::get_wirePos() {
    return this->wirePos;
}
#endif //INC_TRAVERSE_MAP_H
