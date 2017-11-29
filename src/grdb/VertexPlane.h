/*
 * VertexPlane.h
 *
 *  Created on: Nov 28, 2017
 *      Author: florian
 */

#ifndef SRC_GRDB_VERTEXPLANE_H_
#define SRC_GRDB_VERTEXPLANE_H_

#include "../misc/geometry.h"
#include "../misc/debug.h"

#include <vector>
#include <utility>

///@brief The data structure for presenting the routing bins in global routing area.
///@details User can specify the data structure of routing bins by their own, and
///         the default data structure of routing bins is a integer.
template<class T>
class VertexPlane {
public:
    VertexPlane(int xSize, int ySize, T initialValue);

    VertexPlane(const VertexPlane&);

    ~VertexPlane();

    void operator=(const VertexPlane&);

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

    ///@brief Get the specified vertex
    T& vertex(int x, int y); //, int z);

    ///@brief Get the specified vertex, and the vertex is read-only.
    const T& vertex(int x, int y) const; //, int z) const;

private:
    ///The real data structure of plane
    T** vertexPlane_;
    std::vector<T>* vertexPool_;

    ///Plane size
    int xSize_;
    int ySize_;

    ///The initial value
    T initialValue_;

private:
    ///Copy the vertexPlane from other VertexPlane. Used by copy constructor
    void copyPlane(const VertexPlane&);

    ///Release the memory used by plane
    void releasePlane();

    ///This function will assign xxxPool_'s memeory resource to xxxPlane_ pointers
    void assignPoolResource();
};

template<class T>
VertexPlane<T>::VertexPlane(int xSize, int ySize, T initialValue) :
        vertexPlane_(NULL), vertexPool_(NULL), xSize_(xSize), ySize_(ySize), initialValue_(initialValue) {
    resize(xSize_, ySize_);
}

template<class T>
VertexPlane<T>::VertexPlane(const VertexPlane& original) :
        vertexPlane_(NULL), vertexPool_(NULL), xSize_(original.xSize_), ySize_(original.ySize_), initialValue_(original.initialValue_) {
    copyPlane(original);
}

template<class T>
inline VertexPlane<T>::~VertexPlane() {
    releasePlane();
}

template<class T>
void VertexPlane<T>::operator=(const VertexPlane& original) {
    initialValue_ = original.initialValue_;
    copyPlane(original);
}

template<class T>
inline
int VertexPlane<T>::getXSize() const {
    return xSize_;
}

template<class T>
inline
int VertexPlane<T>::getYSize() const {
    return ySize_;
}

template<class T>
inline T& VertexPlane<T>::initialValue() {
    return initialValue_;
}

template<class T>
inline const T& VertexPlane<T>::initialValue() const {
    return initialValue_;
}

template<class T>
inline T& VertexPlane<T>::vertex(int x, int y) {
    assert(x >= 0);
    assert(x < xSize_);
    assert(y >= 0);
    assert(y < ySize_);

    return vertexPlane_[x][y];
}

template<class T>
inline const T& VertexPlane<T>::vertex(int x, int y) const {
    assert(x >= 0);
    assert(x < xSize_);
    assert(y >= 0);
    assert(y < ySize_);

    return vertexPlane_[x][y];
}

template<class T>
void VertexPlane<T>::resize(int xSize, int ySize) {
    assert(xSize >= 0);
    assert(ySize >= 0);

    releasePlane();

    //Do not move the following lines before releasePlane()
    xSize_ = xSize;
    ySize_ = ySize;

    if ((xSize_ != 0) && (ySize_ != 0)) {
        vertexPool_ = new std::vector<T>(xSize_ * ySize_, initialValue_);
        assignPoolResource();
    }
}

template<class T>
inline
void VertexPlane<T>::reset() {
    resize(xSize_, ySize_);
}

template<class T>
void VertexPlane<T>::copyPlane(const VertexPlane& original) {
    releasePlane();

    xSize_ = original.xSize_;
    ySize_ = original.ySize_;

    if ((xSize_ != 0) && (ySize_ != 0)) {
        vertexPool_ = new std::vector<T>(*original.vertexPool_);
        assignPoolResource();
    }
}

template<class T>
void VertexPlane<T>::assignPoolResource() {
    assert(vertexPool_ != NULL);
    assert(vertexPlane_ == NULL);

    vertexPlane_ = new T*[xSize_];

    int index = 0;
    for (int x = 0; x < xSize_; ++x, index += ySize_) {
        vertexPlane_[x] = &((*vertexPool_)[index]);
    }
}

template<class T>
void VertexPlane<T>::releasePlane() {
    if (vertexPool_ != NULL) {
        delete vertexPool_;
        vertexPool_ = NULL;
    }
    if (vertexPlane_ != NULL) {
        delete[] vertexPlane_;
        vertexPlane_ = NULL;
    }

    xSize_ = 0;
    ySize_ = 0;
}

#endif /* SRC_GRDB_VERTEXPLANE_H_ */
