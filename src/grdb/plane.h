/** brief: This file provide two data structure for global routing layer.
 *  The first data structure is the routing bins,
 *  the other one is the routing edges.
 *  Both can be specified the data structure of routing bins or routing edges.
 *  author: Yen-Jung Chang
 * */
#ifndef INC_PLANE_H
#define INC_PLANE_H

#include "../misc/geometry.h"
#include "../misc/debug.h"
#include "VertexPlane.h"
#include "EdgePlane.h"
#include <vector>
#include <utility>

template<class VertexT, class EdgeT>
class Plane {
public:
    Plane(int xSize, int ySize, VertexT vertexInitialValue, EdgeT edgeInitialValue, int edgeNumber = 2);

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

    ///@brief Get the initial value, and it can be changed.
    std::pair<VertexT&, EdgeT&>
    initialValue();

    ///@brief Get the initial value, and it is read-only.
    std::pair<const VertexT&, const EdgeT&>
    initialValue() const;

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
    EdgeT& edge(int x, int y, int dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    const EdgeT& edge(int x, int y, int dir) const;

private:
    VertexPlane<VertexT> vertexPlane_;
    EdgePlane<EdgeT> edgePlane_;
};

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(int xSize, int ySize, VertexT vertexInitialValue, EdgeT edgeInitialValue, int edgeNumber) :
        vertexPlane_(xSize, ySize, vertexInitialValue), edgePlane_(xSize, ySize,  edgeNumber) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(const Plane& original) :
        vertexPlane_(original.vertexPlane_), edgePlane_(original.edgePlane_) {
}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::~Plane() {
}

template<class VertexT, class EdgeT>
void Plane<VertexT, EdgeT>::operator=(const Plane& original) {
    edgePlane_ = original.edgePlane_;
    vertexPlane_ = original.vertexPlane_;
}

template<class VertexT, class EdgeT>
inline
void Plane<VertexT, EdgeT>::resize(int xSize, int ySize) {
    vertexPlane_.resize(xSize, ySize);
    edgePlane_.resize(xSize, ySize);
}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getXSize() const {
    return vertexPlane_.getXSize();
}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getYSize() const {
    return vertexPlane_.getYSize();
}

template<class VertexT, class EdgeT>
inline
void Plane<VertexT, EdgeT>::reset() {
    vertexPlane_.reset();
    edgePlane_.reset();
}

template<class VertexT, class EdgeT>
inline std::pair<VertexT&, EdgeT&> Plane<VertexT, EdgeT>::initialValue() {
    return std::pair<VertexT&, EdgeT&>(vertexPlane_.initialValue(), edgePlane_.initialValue());
}

template<class VertexT, class EdgeT>
inline std::pair<const VertexT&, const EdgeT&> Plane<VertexT, EdgeT>::initialValue() const {
    return std::pair<const VertexT&, const EdgeT&>(vertexPlane_.initialValue(), edgePlane_.initialValue());
}

template<class VertexT, class EdgeT>
inline VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) {
    return vertexPlane_.vertex(x, y);
}

template<class VertexT, class EdgeT>
inline const VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) const {
    return vertexPlane_.vertex(x, y);
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, DirectionType dirType) {
    return edgePlane_.edge(x, y, dirType);
}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, DirectionType dirType) const {
    return edgePlane_.edge(x, y, dirType);
}

template<class VertexT, class EdgeT>
inline EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, int JrDir) {
    return edgePlane_.edge(x, y, JrDir);
}

template<class VertexT, class EdgeT>
inline const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, int JrDir) const {
    return edgePlane_.edge(x, y, JrDir);
}
#endif //INC_PLANE_H
