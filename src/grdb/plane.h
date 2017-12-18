/** brief: This file provide two data structure for global routing layer.
 *  The first data structure is the routing bins,
 *  the other one is the routing edges.
 *  Both can be specified the data structure of routing bins or routing edges.
 *  author: Yen-Jung Chang
 * */
#ifndef INC_PLANE_H
#define INC_PLANE_H

#include <boost/multi_array/base.hpp>
#include <boost/multi_array.hpp>
#include <exception>

#include "../misc/geometry.h"
#include "EdgePlane.h"

template<class VertexT, class EdgeT>
class Plane {
public:
    Plane(int xSize, int ySize);

    Plane(const Coordinate_2d size);
    Plane(const Plane&);

    ~Plane();

  //  void operator=(const Plane&);

    ///@brief Get the map size in x-axis
    int getXSize() const;

    ///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Get the specified vertex
    VertexT& vertex(int x, int y);    //, int z);

    VertexT& vertex(const Coordinate_2d& c);

    const VertexT& vertex(const Coordinate_2d& c) const;

    ///@brief Get the specified vertex, and the vertex is read-only.
    const VertexT& vertex(int x, int y) const;

    EdgePlane<EdgeT>& edges();

    boost::iterator_range<VertexT*> allVertex();

private:

    EdgePlane<EdgeT> edgeplane_;
    boost::multi_array<VertexT, 2> plane_;
};

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(int xSize, int ySize) :
        edgeplane_ { xSize, ySize },    //
        plane_(boost::extents[xSize][ySize]) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(const Coordinate_2d size) :
        edgeplane_ { size },    //
        plane_(boost::extents[size.x][size.y]) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(const Plane& original) :
        edgeplane_ { original.edgeplane_ },    //
        plane_(original.plane_) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::~Plane() {
}

//template<class VertexT, class EdgeT>
//void Plane<VertexT, EdgeT>::operator=(const Plane& original) {
//    edgeplane_ = original.edgeplane_;
//    plane_ = original.plane_;
//}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getXSize() const {
    return plane_.size();
}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getYSize() const {
    return plane_[0].size();
}

template<class VertexT, class EdgeT>
inline VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) {
    return plane_[x][y];
}

template<class VertexT, class EdgeT>
inline VertexT& Plane<VertexT, EdgeT>::vertex(const Coordinate_2d& c) {
    return plane_[c.x][c.y];
}

template<class VertexT, class EdgeT>
inline const VertexT& Plane<VertexT, EdgeT>::vertex(const Coordinate_2d& c) const {
    return plane_[c.x][c.y];
}

template<class VertexT, class EdgeT>
inline const VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) const {
    return plane_[x][y];
}

template<class VertexT, class EdgeT>
inline EdgePlane<EdgeT>& Plane<VertexT, EdgeT>::edges() {
    return edgeplane_;
}

template<class VertexT, class EdgeT>
inline boost::iterator_range<VertexT*> Plane<VertexT, EdgeT>::allVertex() {
    return boost::iterator_range<VertexT*>(plane_.data(), &plane_.data()[plane_.num_elements()]);
}

#endif //INC_PLANE_H
