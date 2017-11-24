// File: misc/geometry.h
// Brief: Data containor for storing information of coordinate, location, directions.
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_GEOMETRY_H
#define INC_GEOMETRY_H

#include <utility>
#include <cstdlib>

namespace Jm {

/**
 * @brief Defined the flags about position, i.e. Direction, Corner
 */
//class Position {
    //public:
    enum DirectionType {
        DIR_NORTH = 0,
        DIR_SOUTH,
        DIR_EAST,
        DIR_WEST,
        DIR_UP,
        DIR_DOWN
    };
    enum CornerType {
        CORNER_LL = 0,  //Left lower
        CORNER_LU,      //Left upper
        CORNER_RL,      //Right lower
        CORNER_RU       //Right upper
    };
//};

/**
 * @brief 3D Coordinate. If you only need to use 2D coordinate, just set z = 0.
 */
class Coordinate {
    public:
                Coordinate (int x, int y, int z);
                ~Coordinate ();
    void        x (int i);
    void        y (int i);
    void        z (int i);
    int         x () const;
    int         y () const;
    int         z () const;

    ///Check if two coordinate are the same
    bool        equal (const Coordinate& co) const;

    ///@brief Check if current coordinate smaller than the other
    ///@details The size is the sum of x, y, and z. If the size of
    ///two coordinate are the same, then the function will compare
    ///by the follow order: x -> y -> z. so even if y and z are 
    ///very huge, but x is smaller than the other, then the current 
    ///coordinate is smaller.
    ///@return true if current coordinate is smaller
    bool        smallerThan (const Coordinate& co) const;

    ///@brief Check if current coordinate bigger than the other
    ///@details The size is the sum of x, y, and z. If the size of
    ///two coordinate are the same, then the function will compare
    ///by the follow order: x -> y -> z. so even if y and z are 
    ///very small, but x is bigger than the other, then the current 
    ///coordinate is bigger.
    ///@return true if current coordinate is bigger
    bool        biggerThan (const Coordinate& co) const;

    ///Check if two coordinate is within distance 1
    bool        isNeighbor (const Coordinate& co) const;

    protected:
    int         x_;
    int         y_;
    int         z_;
};

typedef std::pair<Coordinate, Coordinate> CoordinatePair;
typedef std::pair<Coordinate, DirectionType> CoordinatePosition;

///@brief Callback function for sorting coordinate by their Manhattan
/// distance in ascending order
class ltCoordinate {
    public:
    bool operator()(const Coordinate* c1, const Coordinate* c2) const {
        return c1->smallerThan (*c2);
    }
};


//====== Coordinate ======
inline
Coordinate::Coordinate(int x = 0, int y = 0, int z = 0)
:x_(x),
 y_(y),
 z_(z)
{}

inline
Coordinate::~Coordinate ()
{}

inline
void Coordinate::x(int i)
{
    x_ = i;
}

inline
void Coordinate::y(int i)
{
    y_ = i;
}

inline
void Coordinate::z(int i)
{
    z_ = i;
}

inline
int Coordinate::x() const
{
    return x_;
}

inline
int Coordinate::y() const
{
    return y_;
}

inline
int Coordinate::z() const
{
    return z_;
}

inline
bool Coordinate::equal(const Coordinate& co) const
{
    if ( x_ != co.x() ) return false;
    if ( y_ != co.y() ) return false;
    if ( z_ != co.z() ) return false;
    return true;
}

inline
bool Coordinate::isNeighbor (const Coordinate& co) const
{
    if ( abs( (x_ + y_ + z_) - (co.x() + co.y() + co.z()) ) != 1 )
        return false;
    return true;
}

inline
bool Coordinate::smallerThan(const Coordinate& co) const
{
    if ( (x_ + y_ + z_) < (co.x() + co.y() + co.z()) ) return true;
    if ( (x_ + y_ + z_) > (co.x() + co.y() + co.z()) ) return false;
    //below comparsion will only excute when (x_ + y_ + z_) == (co.x() + co.y() + co.z())
    if ( x_ > co.x() ) return false;
    if ( x_ < co.x() ) return true;
    if ( y_ > co.y() ) return false;
    if ( y_ < co.y() ) return true;
    if ( z_ > co.z() ) return false;
    if ( z_ < co.z() ) return true;
    return false;
}

inline
bool Coordinate::biggerThan(const Coordinate& co) const
{
    if ( (x_ + y_ + z_) > (co.x() + co.y() + co.z()) ) return true;
    if ( (x_ + y_ + z_) < (co.x() + co.y() + co.z()) ) return false;
    //below comparsion will only excute when (x_ + y_ + z_) == (co.x() + co.y() + co.z())
    if ( z_ > co.z() ) return true;
    if ( z_ < co.z() ) return false;
    if ( y_ > co.y() ) return true;
    if ( y_ < co.y() ) return false;
    if ( x_ > co.x() ) return true;
    if ( x_ < co.x() ) return false;
    return false;
}



class Coordinate_2d
{
	public:
		int x;
        int y;
		
    public:
		Coordinate_2d(int x = 0, int y = 0) 
            :x(x), y(y) {}

        bool operator==(const Coordinate_2d& other) const {
            return (x == other.x && y == other.y);
        }
        bool operator!=(const Coordinate_2d& other) const {
            return (x != other.x || y != other.y);
        }
};

class Coordinate_3d
{
	public:
		int x;
        int y;
        int z;
		
    public:
		Coordinate_3d(int x = 0, int y = 0, int z = 0)
            :x(x), y(y), z(z) {}

};

}//namespace Jalamorm
#endif //INC_GEOMETRY_H
