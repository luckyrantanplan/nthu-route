/*
 * EdgePlane.h
 *
 *  Created on: Nov 28, 2017
 *      Author: florian
 */

#ifndef SRC_GRDB_EDGEPLANE_H_
#define SRC_GRDB_EDGEPLANE_H_

#include "../misc/geometry.h"
#include "../misc/debug.h"

#include <vector>
#include <utility>

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
class EdgePlane {
public:
    EdgePlane(int xSize, int ySize, int edgeNumber = 2);

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

    ///@brief Get the initial value, and it can be changed.
    T& initialValue();

    ///@brief Get the initial value, and it is read-only.
    const T& initialValue() const;

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

private:
    ///The routing bins used to connect the routing edges.
    class Vertex {
    public:
        Vertex(T& initialValue);
        T edge[2];
    };

private:
    ///The real data structure of plane
    Vertex** edgePlane_;
    std::vector<Vertex>* edgePool_;

    ///Plane size
    int xSize_;
    int ySize_;

    int edgeNumber_;

    static const int transferTable[2][2];
    static const int Jr2JmTransferTable[4];

private:
    ///Copy the edgePlane
    void copyPlane(const EdgePlane&);

    ///Release the memory used by plane
    void releasePlane();

    ///Because the Vertex only contain the North, East edges, if the user
    ///want to access the South, West edges, we will need to transfer
    ///the coordinate and direction to the available value.
    void transferLocation(int* x, int* y, int index) const;

    ///This function will assign xxxPool_'s memeory resource to xxxPlane_ pointers
    void assignPoolResource();
};

template<class T>
EdgePlane<T>::EdgePlane(int xSize, int ySize, int edgeNumber) :
        edgePlane_(NULL), edgePool_(NULL), xSize_(xSize), ySize_(ySize), edgeNumber_(edgeNumber) {
    resize(xSize_, ySize_);
}

template<class T>
EdgePlane<T>::EdgePlane(const EdgePlane& original) :
        edgePlane_(NULL), edgePool_(NULL), xSize_(original.xSize_), ySize_(original.ySize_), edgeNumber_(original.edgeNumber_) {
    copyPlane(original);
}

template<class T>
EdgePlane<T>::~EdgePlane() {
    releasePlane();
}

template<class T>
void EdgePlane<T>::operator=(const EdgePlane& original) {
    edgeNumber_ = original.edgeNumber_;
    copyPlane(original);
}

template<class T>
const int EdgePlane<T>::transferTable[2][2] = { { 0, -1 }, { -1, 0 } };

template<class T>
const int EdgePlane<T>::Jr2JmTransferTable[4] = { 0, 1, 3, 2 };  //FRONT,BACK,LEFT,RIGHT <-> North, South, East, West

template<class T>
inline
int EdgePlane<T>::getXSize() const {
    return xSize_;
}

template<class T>
inline
int EdgePlane<T>::getYSize() const {
    return ySize_;
}

template<class T>
inline T& EdgePlane<T>::edge(int x, int y, DirectionType dir) {
    assert(x >= 0);
    assert(x < xSize_);
    assert(y >= 0);
    assert(y < ySize_);
    assert(dir >= 0);
    assert((static_cast<int>(dir) >> 1) < edgeNumber_);

    //If the direction is South, West, Down edges, we will need to change it
    //to available direction (North, East, Up) and coordinate
    if ((static_cast<int>(dir) & 0x01) != 0) {
        transferLocation(&x, &y, (static_cast<int>(dir) >> 1));
    }

    return edgePlane_[x][y].edge[(static_cast<int>(dir) >> 1)];
}

template<class T>
inline const T& EdgePlane<T>::edge(int x, int y, DirectionType dir) const {
    return edge(x, y, dir);
}

template<class T>
inline T& EdgePlane<T>::edge(int x, int y, OrientationType JrDir) {
    assert(JrDir >= 0 && JrDir < 4);

    DirectionType dir = static_cast<DirectionType>(Jr2JmTransferTable[JrDir]);
    return edge(x, y, dir);
}

template<class T>
inline const T& EdgePlane<T>::edge(int x, int y, OrientationType JrDir) const {
    return edge(x, y, JrDir);
}

template<class T>
void EdgePlane<T>::resize(int xSize, int ySize) {
    assert(xSize >= 0);
    assert(ySize >= 0);

    releasePlane();

    //Do not move the following 3 lines before releaseColorMap()
    xSize_ = xSize;
    ySize_ = ySize;
    //edgeNumber_ = edgeNumber;

    if ((xSize_ != 0) && (ySize_ != 0)) {
        edgePool_ = new std::vector<Vertex>((xSize_ * ySize_), Vertex { });
        assignPoolResource();
    }
}

template<class T>
inline
void EdgePlane<T>::reset() {
    resize(xSize_, ySize_);
}

template<class T>
void EdgePlane<T>::copyPlane(const EdgePlane& original) {
    releasePlane();

    xSize_ = original.xSize_;
    ySize_ = original.ySize_;

    edgePool_ = new std::vector<Vertex>(*original.edgePool_);
    assignPoolResource();
}

template<class T>
void EdgePlane<T>::assignPoolResource() {
    assert(edgePool_ != NULL);
    assert(edgePlane_ == NULL);

    edgePlane_ = new Vertex*[xSize_];

    int index = 0;
    for (int x = 0; x < xSize_; ++x) {
        edgePlane_[x] = &((*edgePool_)[index]);
        index += ySize_;
    }
}

template<class T>
void EdgePlane<T>::releasePlane() {
    if (edgePool_ != NULL) {
        delete edgePool_;
        edgePool_ = NULL;
    }
    if (edgePlane_ != NULL) {
        delete[] edgePlane_;
        edgePlane_ = NULL;
    }

    xSize_ = 0;
    ySize_ = 0;
}

template<class T>
inline
void EdgePlane<T>::transferLocation(int* x, int* y, int index) const {
    *x += transferTable[index][0];
    *y += transferTable[index][1];

    assert((*x) >= 0);
    assert((*x) < xSize_);
    assert((*y) >= 0);
    assert((*y) < ySize_);
}

template<class T>
EdgePlane<T>::Vertex::Vertex(T& value) {
    for (int i = 0; i < 2; ++i) {
        edge[i] = value;
    }
}

#endif /* SRC_GRDB_EDGEPLANE_H_ */
