/*
 * EdgePlane3d.h
 *
 *  Created on: Nov 28, 2017
 *      Author: florian
 */

#ifndef SRC_GRDB_EDGEPLANE3D_H_
#define SRC_GRDB_EDGEPLANE3D_H_

#include <boost/multi_array.hpp>
#include <stddef.h>
#include <cassert>
#include <vector>

#include "../misc/geometry.h"

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
class EdgePlane3d {
public:
    EdgePlane3d(int xSize, int ySize, int zSize);

    EdgePlane3d(const EdgePlane3d&);

    ~EdgePlane3d();

    void operator=(const EdgePlane3d&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void resize(int xSize, int ySize, int zSize);

    ///@brief Get the map size in x-axis
    int getXSize() const;

    ///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Get the map size in y-axis
    int getZSize() const;

    ///@brief Reset every vertex to initial value.
    void reset();

    ///@brief Get the specified edge
    T& edge(int x, int y, int z, DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const T& edge(int x, int y, int z, DirectionType) const;

    ///@brief Get the specified edge.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Direction
    /// (North, South, East, West)
    T& edge(int x, int y, int z, OrientationType dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Direction
    /// (North, South, East, West)
    const T& edge(int x, int y, int z, OrientationType dir) const;

    ///@brief Get the specified edge between 2 vertices
    T& edge(const Coordinate_3d& c1, const Coordinate_3d& c2);

    ///@brief Get the specified edge between 2 vertices, and the edge is read-only.
    const T& edge(const Coordinate_3d& c1, const Coordinate_3d& c2) const;

    template<typename F>
    void foreach(const F& f);
private:
    ///The real data structure of plane
    boost::multi_array<T, 3> edgePlane_;

private:

};

template<class T>
EdgePlane3d<T>::EdgePlane3d(int xSize, int ySize, int zSize) :
        edgePlane_(boost::extents[xSize][ySize][2 * zSize]) {

}

template<class T>
EdgePlane3d<T>::EdgePlane3d(const EdgePlane3d& original) :
        edgePlane_(original.edgePlane_) {
    copyPlane(original);
}

template<class T>
EdgePlane3d<T>::~EdgePlane3d() {

}

template<class T>
void EdgePlane3d<T>::operator=(const EdgePlane3d& original) {

    copyPlane(original);
}

template<class T>
inline
int EdgePlane3d<T>::getXSize() const {
    return edgePlane_.size();
}

template<class T>
inline
int EdgePlane3d<T>::getYSize() const {
    return edgePlane_[0].size();
}

template<class T>
inline
int EdgePlane3d<T>::getZSize() const {
    return edgePlane_[0][0].size() / 2;
}

template<class T>
inline T& EdgePlane3d<T>::edge(int x, int y, int z, DirectionType dir) {

    //If the direction is South, West, Down edges, we will need to change it
    //to available direction (North, East, Up) and coordinate

    switch (dir) {
    case DirectionType::DIR_EAST:
        return edgePlane_[x][y][2 * z];
    case DirectionType::DIR_NORTH:
        return edgePlane_[x][y - 1][2 * z + 1];
    case DirectionType::DIR_SOUTH:
        return edgePlane_[x][y][2 * z + 1];
    case DirectionType::DIR_WEST:
        return edgePlane_[x - 1][y][2 * z];
    case DirectionType::DIR_UP:
        throw std::exception();
    case DirectionType::DIR_DOWN:
        throw std::exception();

    }

}

template<class T>
inline const T& EdgePlane3d<T>::edge(int x, int y, int z, DirectionType dir) const {
    return edge(x, y, z, dir);
}

template<class T>
inline T& EdgePlane3d<T>::edge(int x, int y, int z, OrientationType JrDir) {
    static const int Jr2JmTransferTable[4] = { 0, 1, 3, 2 };  //FRONT,BACK,LEFT,RIGHT <-> North, South, East, West
    DirectionType dir = static_cast<DirectionType>(Jr2JmTransferTable[JrDir]);
    return edge(x, y, z, dir);
}

template<class T>
inline const T& EdgePlane3d<T>::edge(int x, int y, int z, OrientationType JrDir) const {
    return edge(x, y, z, JrDir);
}

template<class T>
void EdgePlane3d<T>::resize(int xSize, int ySize, int zSize) {
    edgePlane_.resize(boost::extents[xSize][ySize][2 * zSize]);
}

template<class T>
inline
void EdgePlane3d<T>::reset() {
    edgePlane_.clear();
}

template<class T>
template<typename F>
void EdgePlane3d<T>::foreach(const F& f) {
    for (std::size_t i = 0; i < edgePlane_.num_elements(); ++i) {
        f(edgePlane_.data()[i]);
    }
}

template<class T>
T& EdgePlane3d<T>::edge(const Coordinate_3d& c1, const Coordinate_3d& c2) {

    if (c1.x < c2.x) {
        return edgePlane_[c1.x][c1.y][2 * c1.z];
    }
    if (c1.x > c2.x) {
        return edgePlane_[c2.x][c2.y][2 * c2.z];
    }
    if (c1.y < c2.y) {
        return edgePlane_[c1.x][c1.y][2 * c1.z + 1];
    }
    if (c1.y > c2.y) {
        return edgePlane_[c2.x][c2.y][2 * c2.z + 1];
    }
    throw std::exception();
}

template<class T>
const T& EdgePlane3d<T>::edge(const Coordinate_3d& c1, const Coordinate_3d& c2) const {
    return edge(c1, c2);
}

#endif /* SRC_GRDB_EDGEPLANE3D_H_ */
