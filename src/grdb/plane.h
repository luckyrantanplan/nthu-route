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

#include "../misc/geometry.h"

template<class VertexT, class EdgeT>
class Plane {
public:
    Plane(int xSize, int ySize);

    Plane(const Plane&);

    ~Plane();

    void operator=(const Plane&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void resize(int xSize, int ySize);

    ///@brief Get the map size in x-axis
    int getXSize() const;

    ///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Reset every vertex to initial value.
    void reset();

    ///@brief Get the specified vertex
    VertexT& vertex(int x, int y);    //, int z);

    ///@brief Get the specified vertex, and the vertex is read-only.
    const VertexT& vertex(int x, int y) const;

    ///@brief Get the specified edge
    EdgeT& edge(int x, int y, DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const EdgeT& edge(int x, int y, DirectionType) const;

    ///@brief Get the specified edge.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    EdgeT& edge(int x, int y, OrientationType dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    const EdgeT& edge(int x, int y, OrientationType dir) const;

private:

    struct Vertex {
        VertexT v;
        EdgeT east;
        EdgeT south;
    };
    boost::multi_array<Vertex, 2> plane_;
};

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(int xSize, int ySize) :
        plane_(boost::extents[xSize][ySize]) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(const Plane& original) :
        plane_(original.plane_) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::~Plane() {
}

template<class VertexT, class EdgeT>
void Plane<VertexT, EdgeT>::operator=(const Plane& original) {
    plane_ = original.plane_;
}

template<class VertexT, class EdgeT>
inline
void Plane<VertexT, EdgeT>::resize(int xSize, int ySize) {
    plane_.resize(boost::extents[xSize][ySize]);

}

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
inline
void Plane<VertexT, EdgeT>::reset() {
    plane_.clear();
}

template<class VertexT, class EdgeT>
inline VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) {
    return plane_[x][y].v;
}

template<class VertexT, class EdgeT>
inline const VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) const {
    return plane_[x][y].v;
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, DirectionType dirType) {

    switch (dirType) {
    case DirectionType::DIR_EAST:
        return plane_[x][y].east;
    case DirectionType::DIR_NORTH:
        return plane_[x][y - 1].south;
    case DirectionType::DIR_SOUTH:
        return plane_[x][y].south;
    case DirectionType::DIR_WEST:
        return plane_[x - 1][y].east;
    case DirectionType::DIR_UP:

    case DirectionType::DIR_DOWN:
    default:
        throw std::exception();

    }
    return plane_[x][y].east; //unreachable
}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, DirectionType dirType) const {
    return edge(x, y, dirType);
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, OrientationType JrDir) {
    static const int Jr2JmTransferTable[4] = { 0, 1, 3, 2 };
    DirectionType dir = static_cast<DirectionType>(Jr2JmTransferTable[JrDir]);
    return edge(x, y, dir);

}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, OrientationType JrDir) const {
    return edge(x, y, JrDir);
}
#endif //INC_PLANE_H
