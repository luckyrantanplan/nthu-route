/*
 * EdgePlane.h
 *
 *  Created on: Nov 28, 2017
 *      Author: florian
 */

#ifndef SRC_GRDB_EDGEPLANE_H_
#define SRC_GRDB_EDGEPLANE_H_

#include <boost/multi_array.hpp>
#include <stddef.h>
#include <cassert>
#include <vector>

#include "../misc/geometry.h"

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
class EdgePlane {
public:
    EdgePlane(int xSize, int ySize);

    EdgePlane(const EdgePlane&);

    ~EdgePlane();

    void operator=(const EdgePlane&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void resize(int xSize, int ySize);

    ///@brief Get the map size in x-axis
    int getXSize() const;

    ///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Reset every vertex to initial value.
    void reset();

    ///@brief Get the specified edge
    T& edge(int x, int y, DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const T& edge(int x, int y, DirectionType) const;

    ///@brief Get the specified edge.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Direction
    /// (North, South, East, West)
    T& edge(int x, int y, OrientationType dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Direction
    /// (North, South, East, West)
    const T& edge(int x, int y, OrientationType dir) const;

    template<typename F>
    void foreach(F& f);
private:
    ///The real data structure of plane
    boost::multi_array<T, 2> edgePlane_;

};

template<class T>
EdgePlane<T>::EdgePlane(int xSize, int ySize) :
        edgePlane_(boost::extents[xSize][2 * ySize]) {

}

template<class T>
EdgePlane<T>::EdgePlane(const EdgePlane& original) :
        edgePlane_(original.edgePlane_) {
    copyPlane(original);
}

template<class T>
EdgePlane<T>::~EdgePlane() {

}

template<class T>
void EdgePlane<T>::operator=(const EdgePlane& original) {

    copyPlane(original);
}

template<class T>
inline
int EdgePlane<T>::getXSize() const {
    return edgePlane_.size();
}

template<class T>
inline
int EdgePlane<T>::getYSize() const {
    return edgePlane_[0].size() / 2;
}

template<class T>
inline T& EdgePlane<T>::edge(int x, int y, DirectionType dir) {

    //If the direction is South, West, Down edges, we will need to change it
    //to available direction (North, East, Up) and coordinate

    switch (dir) {
    case DirectionType::DIR_EAST:
        return edgePlane_[x][2 * y];
    case DirectionType::DIR_NORTH:
        return edgePlane_[x][2 * y - 1];
    case DirectionType::DIR_SOUTH:
        return edgePlane_[x][2 * y + 1];
    case DirectionType::DIR_WEST:
        return edgePlane_[x - 1][2 * y];
    case DirectionType::DIR_UP:
        throw std::exception();
    case DirectionType::DIR_DOWN:
        throw std::exception();

    }
    return edgePlane_[x][2 * y]; // unreachable
}

template<class T>
inline const T& EdgePlane<T>::edge(int x, int y, DirectionType dir) const {
    return edge(x, y, dir);
}

template<class T>
inline T& EdgePlane<T>::edge(int x, int y, OrientationType JrDir) {
    static const int Jr2JmTransferTable[4] = { 0, 1, 3, 2 };  //FRONT,BACK,LEFT,RIGHT <-> North, South, East, West
    DirectionType dir = static_cast<DirectionType>(Jr2JmTransferTable[JrDir]);
    return edge(x, y, dir);
}

template<class T>
inline const T& EdgePlane<T>::edge(int x, int y, OrientationType JrDir) const {
    return edge(x, y, JrDir);
}

template<class T>
void EdgePlane<T>::resize(int xSize, int ySize) {
    edgePlane_.resize(boost::extents[xSize][2 * ySize]);
}

template<class T>
inline
void EdgePlane<T>::reset() {
    edgePlane_.clear();
}

template<class T>
template<typename F>
void EdgePlane<T>::foreach(F& f) {
    for (int i = 0; i < edgePlane_.num_elements(); ++i) {
        f(edgePlane_.data()[i]);
    }
}

#endif /* SRC_GRDB_EDGEPLANE_H_ */
