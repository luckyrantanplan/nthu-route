/** brief: This file provide two data structure for global routing layer.
 *  The first data structure is the routing bins,
 *  the other one is the routing edges.
 *  Both can be specified the data structure of routing bins or routing edges.
 *  author: Yen-Jung Chang
 * */
#ifndef INC_PLANE3D_H
#define INC_PLANE3D_H

#include <boost/multi_array/base.hpp>
#include <boost/multi_array.hpp>

#include "../misc/geometry.h"

template<class VertexT, class EdgeT>
class Plane3d {
public:
    Plane3d(int xSize, int ySize, int zSize);

    Plane3d(const Plane3d&);

    ~Plane3d();

    void operator=(const Plane3d&);

    ///@brief Change the size of Plane3d. Every vertex will reset to initial value.
    void resize(int xSize, int ySize, int zSize);

    ///@brief Get the map size in x-axis
    int getXSize() const;

    ///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Reset every vertex to initial value.
    void reset();

    ///@brief Get the specified vertex
    VertexT& vertex(int x, int y, int z);    //, int z);

    ///@brief Get the specified vertex, and the vertex is read-only.
    const VertexT& vertex(int x, int y, int z) const;

    ///@brief Get the specified edge
    EdgeT& edge(int x, int y, int z, DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const EdgeT& edge(int x, int y, int z, DirectionType) const;

    ///@brief Get the specified edge.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    EdgeT& edge(int x, int y, int z, OrientationType dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    const EdgeT& edge(int x, int y, int z, OrientationType dir) const;

private:

    struct Vertex {
        VertexT v;
        EdgeT east;
        EdgeT south;
        EdgeT down;
    };
    boost::multi_array<Vertex, 3> plane_;
};

template<class VertexT, class EdgeT>
Plane3d<VertexT, EdgeT>::Plane3d(int xSize, int ySize, int zSize) :
        plane_(boost::extents[xSize][ySize][zSize]) {
}

template<class VertexT, class EdgeT>
Plane3d<VertexT, EdgeT>::Plane3d(const Plane3d& original) :
        plane_(original.plane_) {
}

template<class VertexT, class EdgeT>
Plane3d<VertexT, EdgeT>::~Plane3d() {
}

template<class VertexT, class EdgeT>
void Plane3d<VertexT, EdgeT>::operator=(const Plane3d& original) {
    plane_ = original.plane_;
}

template<class VertexT, class EdgeT>
inline
void Plane3d<VertexT, EdgeT>::resize(int xSize, int ySize, int zSize) {
    plane_.resize(xSize, ySize, zSize);

}

template<class VertexT, class EdgeT>
inline
int Plane3d<VertexT, EdgeT>::getXSize() const {
    return plane_.size();
}

template<class VertexT, class EdgeT>
inline
int Plane3d<VertexT, EdgeT>::getYSize() const {
    return plane_[0].size();
}

template<class VertexT, class EdgeT>
inline
void Plane3d<VertexT, EdgeT>::reset() {
    plane_.clear();
}

template<class VertexT, class EdgeT>
inline VertexT& Plane3d<VertexT, EdgeT>::vertex(int x, int y, int z) {
    return plane_[x][y][z].v;
}

template<class VertexT, class EdgeT>
inline const VertexT& Plane3d<VertexT, EdgeT>::vertex(int x, int y, int z) const {
    return plane_[x][y][z].v;
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane3d<VertexT, EdgeT>::edge(int x, int y, int z, DirectionType dirType) {

    switch (dirType) {
    case DirectionType::DIR_EAST:
        return plane_[x][y][z].east;
    case DirectionType::DIR_NORTH:
        return plane_[x][y - 1][z].south;
    case DirectionType::DIR_SOUTH:
        return plane_[x][y][z].south;
    case DirectionType::DIR_WEST:
        return plane_[x - 1][y][z].east;
    case DirectionType::DIR_UP:
        return plane_[x][y][z - 1].down;
    case DirectionType::DIR_DOWN:
        return plane_[x][y][z].down;

    }
    return plane_[x][y].east; //unreachable
}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane3d<VertexT, EdgeT>::edge(int x, int y, int z, DirectionType dirType) const {
    return edge(x, y, z, dirType);
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane3d<VertexT, EdgeT>::edge(int x, int y, int z, OrientationType JrDir) {
    static const int Jr2JmTransferTable[6] = { 0, 1, 3, 2, 4, 5 };
    DirectionType dir = static_cast<DirectionType>(Jr2JmTransferTable[JrDir]);
    return edge(x, y, z, dir);

}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane3d<VertexT, EdgeT>::edge(int x, int y, int z, OrientationType JrDir) const {
    return edge(x, y, z, JrDir);
}
#endif //INC_PLANE3D_H
