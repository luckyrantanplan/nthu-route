#ifndef INC_TRAVERSE_MAP_H
#define INC_TRAVERSE_MAP_H

#include "../grdb/plane.h"
#include "../misc/geometry.h"

#include <fstream>
#include <string>
#include <list>
#include <vector>
#include <utility>
#include <iostream>

typedef std::pair<int, int> IntPair;

class BSearchQueNode{
	private:
		//cell information, such as x, y, z coordinate
        Coordinate cell;
		//position of current wire segment in wire segments list
		int wirePos;
	public:
		//Constructor
		BSearchQueNode(int x, int y, int z, int wirePos);
		//GET Functions
		int get_x();
		int get_y();
		int get_z();
		int get_wirePos();
};

class BSearchQue{
	private:
        std::list<BSearchQueNode> que;
	public:
		//SET Functions
		void push_back(int x, int y, int z, int wirePos);
        //remove the first cell
		void pop_front();
		//GET Functions
		int nextX();
		int nextY();
		int nextZ();
		int size();
		int get_wirePos();
};


/****************
 * VertexColorMap
 ***************/
template<class T>
class VertexColorMap{
	public:
    //Constructor
    VertexColorMap(int xSize = 0, int ySize = 0, /*int zSize = 0,*/ T initialColor_ = 0);

    ///Change the size of color map
    void resize(int xSize, int ySize);//, int zSize);

    ///Reset the color value to initial value
    void reset();
    ///Set the color value
    T& color(int x, int y);//, int z);

    ///Set the color value
    T& color(Coordinate& coor);
    
    ///Get the color value
    const T& color(int x, int y) const; //, int z) const;

    ///Get the color value
    const T& color(Coordinate& coor) const;

	private:
    VertexPlane<T> vertexPlane;
};

template<class T>
VertexColorMap<T>::VertexColorMap(int xSize, int ySize, /*int zSize,*/ T initialColor)
:vertexPlane(xSize, ySize, /*zSize,*/ initialColor)
{}

template<class T>
inline
void VertexColorMap<T>::resize(int xSize, int ySize)//, int zSize)
{
    vertexPlane.resize(xSize, ySize);//, zSize);
}

template<class T>
inline
void VertexColorMap<T>::reset(){
    vertexPlane.reset();
}

template<class T>
inline
const T& VertexColorMap<T>::color(int x, int y) const//, int z) const 
{
	return vertexPlane.vertex(x, y);//, z);
}

template<class T>
inline
const T& VertexColorMap<T>::color(Coordinate& coor) const 
{
	return vertexPlane.vertex(coor.x(), coor.y());//, coor.z());
}


template<class T>
inline
T& VertexColorMap<T>::color(int x, int y)//, int z)
{
	return vertexPlane.vertex(x, y);//, z);
}

template<class T>
inline
T& VertexColorMap<T>::color(Coordinate& coor)
{
	return vertexPlane.vertex(coor.x(), coor.y());//, coor.z());
}


/**************
 * EdgeColorMap
 *************/
template<class T>
class EdgeColorMap {
	public:
    //Constructor
    EdgeColorMap(int xSize = 0, int ySize = 0, /*int zSize = 0,*/ T initialColor = 0);

    ///Change the size of color map
    void resize(int xSize, int ySize);//, int zSize);

    ///Reset the color value to initial value
    void reset();

    ///Set color
    T& color(int x, int y, /*int z,*/ DirectionType);

    ///Set color
    T& color(Coordinate& coor, DirectionType);

    ///Get color
    const T& color(int x, int y, /*int z,*/ DirectionType) const;

    ///Get color
    const T& color(Coordinate& coor, DirectionType) const;

    private:
    EdgePlane<T> edgePlane;
};

//initial color map for breadth-first search algorithm
template<class T>
EdgeColorMap<T>::EdgeColorMap(int xSize, int ySize, /*int zSize,*/ T initialColor)
:edgePlane(xSize, ySize, /*zSize,*/ initialColor)
{}

template<class T>
inline
void EdgeColorMap<T>::resize(int xSize, int ySize)//, int zSize)
{
    edgePlane.resize(xSize, ySize);//, zSize);
}

template<class T>
inline
void EdgeColorMap<T>::reset(){
    edgePlane.reset();
}

template<class T>
inline
const T& EdgeColorMap<T>::color(int x, int y, /*int z,*/ DirectionType dir) const 
{
    return edgePlane.edge(x, y, /*z,*/ dir);
}

template<class T>
inline
const T& EdgeColorMap<T>::color(Coordinate& coor, DirectionType dir) const
{
    return edgePlane.edge(coor.x(), coor.y(), /*coor.z(),*/ dir);
}

template<class T>
inline
T& EdgeColorMap<T>::color(int x, int y, /*int z,*/ DirectionType dir)
{
    return edgePlane.edge(x, y, /*z,*/ dir);
}

template<class T>
inline
T& EdgeColorMap<T>::color(Coordinate& coor, DirectionType dir)
{
    return edgePlane.edge(coor.x(), coor.y(), /*coor.z(),*/ dir);
}

// Inline Functions
/************
 * BSearchQue
 ************/
inline
void BSearchQue::push_back(int x, int y, int z, int wirePos){
	this->que.push_back(BSearchQueNode(x, y, z, wirePos));
}

inline
void BSearchQue::pop_front(){
	this->que.pop_front();
}

inline
int BSearchQue::nextX(){
	return this->que.front().get_x();
}

inline
int BSearchQue::nextY(){
	return this->que.front().get_y();
}

inline
int BSearchQue::nextZ(){
	return this->que.front().get_z();
}

inline
int BSearchQue::size(){
	return this->que.size();
}

inline
int BSearchQue::get_wirePos(){
	return this->que.front().get_wirePos();
}

/****************
 * BSearchQueNode
 ****************/
inline
int BSearchQueNode::get_x(){
	return this->cell.x();
}

inline
int BSearchQueNode::get_y(){
	return this->cell.y();
}

inline
int BSearchQueNode::get_z(){
	return this->cell.z();
}

inline
int BSearchQueNode::get_wirePos(){
	return this->wirePos;
}
#endif //INC_TRAVERSE_MAP_H
