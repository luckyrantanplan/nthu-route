/*
 * EdgePlane3d.h
 *
 *  Created on: Nov 28, 2017
 *      Author: florian
 */

#ifndef SRC_GRDB_EDGEPLANE3D_H_
#define SRC_GRDB_EDGEPLANE3D_H_

#include <boost/multi_array/base.hpp>
#include <boost/multi_array.hpp>
#include <boost/range/iterator_range_core.hpp>
#include <boost/range/adaptor/strided.hpp>
#include <array>
#include <cstddef>
#include <exception>

#include "../misc/geometry.h"

namespace boost {
namespace adaptors {
struct sliced;
} /* namespace adaptors */
} /* namespace boost */
namespace NTHUR {
///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
class EdgePlane3d {

public:

    enum EdgeDir {
        EAST = 0, SOUTH = 1, FRONT = 2
    };

    EdgePlane3d(const int xSize, const int ySize, const int zSize);

    EdgePlane3d(const Coordinate_3d& size);

    EdgePlane3d(const EdgePlane3d&);

    ~EdgePlane3d();

    void operator=(const EdgePlane3d&);

    ///@brief Get the map size in x-axis and y-axis
    Coordinate_3d getSize() const;

///@brief Get the map size in x-axis
    int getXSize() const;

///@brief Get the map size in y-axis
    int getYSize() const;

    ///@brief Get the map size in z-axis
    int getZSize() const;
    /*
     ///@brief Get the neighbors

     RangeExpression neighbors(const Coordinate_3d& c);
     */
    boost::iterator_range<T*> all();

    boost::iterator_range<const T*> all() const;

    ///@brief Get the specified edge between 2 vertices
    T& edge(const Coordinate_3d& c1, const Coordinate_3d& c2);

    ///@brief Get the specified edge between 2 vertices, and the edge is read-only.
    const T& edge(const Coordinate_3d& c1, const Coordinate_3d& c2) const;

    const std::size_t num_elements() const;

    ///@brief Get the specified edge
    const T& east(const Coordinate_3d& c) const;

    ///@brief Get the specified edge
    const T& south(const Coordinate_3d& c) const;

    ///@brief Get the specified edge
    T& east(const Coordinate_3d& c);

    ///@brief Get the specified edge
    T& south(const Coordinate_3d& c);

    const T& front(const Coordinate_3d& c) const;

    T& front(const Coordinate_3d& c);
private:

    T& get(const Coordinate_3d& c, const EdgeDir dir);

    const T& get(const Coordinate_3d& c, const EdgeDir dir) const;

///The real data structure of plane
    boost::multi_array<T, 4> edgePlane_;

};

template<class T>
EdgePlane3d<T>::EdgePlane3d(int xSize, int ySize, int zSize) :
        edgePlane_(boost::extents[xSize][ySize][zSize][3]) {

}

template<class T>
EdgePlane3d<T>::EdgePlane3d(const Coordinate_3d& size) :
        edgePlane_(boost::extents[size.x][size.y][size.z][3]) {

}

template<class T>
EdgePlane3d<T>::EdgePlane3d(const EdgePlane3d& original) :
        edgePlane_(original.edgePlane_) {

}

template<class T>
EdgePlane3d<T>::~EdgePlane3d() {

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
    return edgePlane_[0][0].size();
}

template<class T>
inline Coordinate_3d EdgePlane3d<T>::getSize() const {
    return Coordinate_3d(getXSize(), getYSize(), getZSize());
}

template<class T>
boost::iterator_range<T*> EdgePlane3d<T>::all() {
    return boost::iterator_range<T*>(edgePlane_.data(), &edgePlane_.data()[edgePlane_.num_elements()]);
}

template<class T>
boost::iterator_range<const T*> EdgePlane3d<T>::all() const {
    return boost::iterator_range<const T*>(edgePlane_.data(), &edgePlane_.data()[edgePlane_.num_elements()]);
}

template<class T>
T& EdgePlane3d<T>::edge(const Coordinate_3d& c1, const Coordinate_3d& c2) {
    if (c1.x < c2.x) {
        return get(c1, EAST);
    }
    if (c1.x > c2.x) {
        return get(c2, EAST);
    }
    if (c1.y < c2.y) {
        return get(c1, SOUTH);
    }
    if (c1.y > c2.y) {
        return get(c2, SOUTH);
    }
    if (c1.z < c2.z) {
        return get(c1, FRONT);
    }
    if (c1.z > c2.z) {
        return get(c2, FRONT);
    }
    throw std::exception();
}

template<class T>
const T& EdgePlane3d<T>::edge(const Coordinate_3d& c1, const Coordinate_3d& c2) const {
    if (c1.x < c2.x) {
        return get(c1, EAST);
    }
    if (c1.x > c2.x) {
        return get(c2, EAST);
    }
    if (c1.y < c2.y) {
        return get(c1, SOUTH);
    }
    if (c1.y > c2.y) {
        return get(c2, SOUTH);
    }
    if (c1.z < c2.z) {
        return get(c1, FRONT);
    }
    if (c1.z > c2.z) {
        return get(c2, FRONT);
    }
    throw std::exception();
}
template<class T>
const std::size_t EdgePlane3d<T>::num_elements() const {
    return edgePlane_.num_elements();
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline T& EdgePlane3d<T>::east(const Coordinate_3d& c) {
    return get(c, EAST);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline const T& EdgePlane3d<T>::east(const Coordinate_3d& c) const {
    return get(c, EAST);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline T& EdgePlane3d<T>::south(const Coordinate_3d& c) {
    return get(c, SOUTH);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline const T& EdgePlane3d<T>::south(const Coordinate_3d& c) const {
    return get(c, SOUTH);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline const T& EdgePlane3d<T>::front(const Coordinate_3d& c) const {
    return get(c, FRONT);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline T& EdgePlane3d<T>::front(const Coordinate_3d& c) {
    return get(c, FRONT);
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline T& EdgePlane3d<T>::get(const Coordinate_3d& c, const EdgeDir dir) {
    return edgePlane_[c.x][c.y][c.z][dir];
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
inline const T& EdgePlane3d<T>::get(const Coordinate_3d& c, const EdgeDir dir) const {
    return edgePlane_[c.x][c.y][c.z][dir];
}
} // namespace NTHUR

#endif /* SRC_GRDB_EDGEPLANE3D_H_ */
