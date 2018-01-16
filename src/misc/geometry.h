// File: misc/geometry.h
// Brief: Data container for storing information of coordinate, location, directions.
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_GEOMETRY_H
#define INC_GEOMETRY_H

#include <boost/functional/hash.hpp>
#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>

namespace NTHUR {
/**
 * @brief Defined the flags about position, i.e. Direction, Corner
 */

enum OrientationType {
    FRONT, BACK, LEFT, RIGHT, UP, DOWN
};

class Coordinate_2d {
public:
    int x;
    int y;

public:
    Coordinate_2d(int x = 0, int y = 0) :
            x { x }, y { y } {
    }

    Coordinate_2d(const Coordinate_2d& c) :
            x { c.x }, y { c.y } {
    }

    bool operator==(const Coordinate_2d& other) const {
        return (x == other.x && y == other.y);
    }
    bool operator!=(const Coordinate_2d& other) const {
        return (x != other.x || y != other.y);
    }
    void set(int ix, int iy) {
        x = ix;
        y = iy;
    }

    void set(const Coordinate_2d& other) {
        x = other.x;
        y = other.y;
    }

    Coordinate_2d operator +(const Coordinate_2d& a) {
        return Coordinate_2d(x + a.x, y + a.y);
    }

    bool isAligned(const Coordinate_2d& c) const {
        return x == c.x || y == c.y;
    }

    static const std::array<Coordinate_2d, 4> dir_array() {
        static std::array<Coordinate_2d, 4> arr { { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } } };
        return arr; ////FRONT,BACK,LEFT,RIGHT; //FRONT,BACK,LEFT,RIGHT
    }

    std::string toString() const {
        return std::to_string(x) + "," + std::to_string(y);
    }

};
} // namespace NTHUR
namespace std {

template<>
struct hash<NTHUR::Coordinate_2d> {
    std::size_t operator()(const NTHUR::Coordinate_2d& c) const {
        // Start with a hash value of 0    .
        std::size_t seed = 0;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:
        boost::hash_combine(seed, hash<int>()(c.x));
        boost::hash_combine(seed, hash<int>()(c.y));

        // Return the result.
        return seed;

    }
};

}        // namespace std

namespace NTHUR {

class Coordinate_3d {
public:
    int x;
    int y;
    int z;

public:
    Coordinate_3d(int x = 0, int y = 0, int z = 0) :
            x(x), y(y), z(z) {
    }

    Coordinate_3d(const Coordinate_2d& c, int z = 0) :
            x { c.x }, y { c.y }, z { z } {
    }
    void set(int ix, int iy, int iz) {
        x = ix;
        y = iy;
        z = iz;
    }

    Coordinate_2d xy() const {
        return Coordinate_2d { x, y };
    }

    bool isAligned(const Coordinate_3d& c) const {
        return (x == c.x && y == c.y) || //
                (y == c.y && z == c.z) || //
                (x == c.x && z == c.z);
    }

    bool operator==(const Coordinate_3d& other) const {
        return (x == other.x && y == other.y && z == other.z);
    }
    std::string toString() const {
        return "(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + ")";
    }
};

struct Segment3d {
    Coordinate_3d first;
    Coordinate_3d last;
};
class Rectangle {
public:
    Coordinate_2d upLeft;
    Coordinate_2d downRight;

public:
    Rectangle(const Coordinate_2d c1, const Coordinate_2d c2);
    bool contains(const Coordinate_2d& c) const;
    bool contains(const Rectangle& r) const;
    void expand(int i);

    void clip(Rectangle& r) const;

    void frame(std::function<void(const Coordinate_2d& e1, const Coordinate_2d& e2)> f);

    std::string toString() const;
};

inline Rectangle::Rectangle(const Coordinate_2d c1, const Coordinate_2d c2) :
        upLeft { std::min(c1.x, c2.x), std::min(c1.y, c2.y) }, //
        downRight { std::max(c1.x, c2.x), std::max(c1.y, c2.y) } {
}
inline
bool Rectangle::contains(const Coordinate_2d& c) const {
    return (upLeft.x <= c.x && c.x <= downRight.x && upLeft.y <= c.y && c.y <= downRight.y);
}
inline
bool Rectangle::contains(const Rectangle& r) const {
    return (upLeft.x <= r.upLeft.x && r.downRight.x <= downRight.x && //
            upLeft.y <= r.upLeft.y && r.downRight.y <= downRight.y);
}
inline
void Rectangle::expand(int i) {
    upLeft.x -= i;
    upLeft.y -= i;
    downRight.x += i;
    downRight.y += i;
}
inline
void Rectangle::clip(Rectangle& r) const {
    r.upLeft.x = std::max(upLeft.x, r.upLeft.x);
    r.upLeft.y = std::max(upLeft.y, r.upLeft.y);
    r.downRight.x = std::min(downRight.x, r.downRight.x);
    r.downRight.y = std::min(downRight.y, r.downRight.y);
}
inline
void Rectangle::frame(std::function<void(const Coordinate_2d& e1, const Coordinate_2d& e2)> f) {
    for (int i = upLeft.x; i < downRight.x; ++i) {
        f(Coordinate_2d { i, upLeft.y }, Coordinate_2d { i + 1, upLeft.y });
    }
    for (int i = upLeft.y; i < downRight.y; ++i) {
        f(Coordinate_2d { downRight.x, i }, Coordinate_2d { downRight.x, i + 1 });
    }
    if (!downRight.isAligned(upLeft)) {
        for (int i = upLeft.x; i < downRight.x; ++i) {
            f(Coordinate_2d { i, downRight.y }, Coordinate_2d { i + 1, downRight.y });
        }
        for (int i = upLeft.y; i < downRight.y; ++i) {
            f(Coordinate_2d { upLeft.x, i }, Coordinate_2d { upLeft.x, i + 1 });
        }
    }
}
inline std::string Rectangle::toString() const {
    std::string s = "upLeft: " + upLeft.toString();
    s += " downRight: " + downRight.toString();
    return s;
}
} // namespace NTHUR
#endif //INC_GEOMETRY_H
