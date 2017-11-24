/** brief: This file provide two data structure for global routing layer.
 *  The first data structure is the routing bins,
 *  the other one is the routing edges.
 *  Both can be specified the data strucutre of routing bins or routing edges.
 *  author: Yen-Jung Chang
 * */
#ifndef INC_PLANE_H
#define INC_PLANE_H

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
                VertexPlane(int xSize,
                            int ySize,
                            T initialValue);

                VertexPlane(const VertexPlane&);

                ~VertexPlane();

    void        operator=(const VertexPlane&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void        resize(int xSize, int ySize);

    ///@brief Get the map size in x-axis
    int         getXSize () const;

    ///@brief Get the map size in y-axis
    int         getYSize () const;

    ///@brief Reset every vertex to initial value.
    void        reset();

    ///@brief Get the initial value, and it can be changed.
    T&          initialValue();

    ///@brief Get the initial value, and it is read-only.
    const T&    initialValue() const;

    ///@brief Get the specified vertex
    T&          vertex(int x, int y);//, int z);

    ///@brief Get the specified vertex, and the vertex is read-only.
    const T&    vertex(int x, int y) const;//, int z) const;

	private:
    ///The real data structure of plane
    T**         vertexPlane_;
    std::vector<T>* vertexPool_;

    ///Plane size
    int         xSize_;
    int         ySize_;

    ///The initial value
    T           initialValue_;

    private:
    ///Copy the vertexPlane from other VertexPlane. Used by copy constructor
    void        copyPlane(const VertexPlane&);

    ///Release the memory used by plane
    void        releasePlane();

    ///This function will assign xxxPool_'s memeory resource to xxxPlane_ pointers
    void        assignPoolResource ();
};

template<class T>
VertexPlane<T>::VertexPlane(int xSize, int ySize, T initialValue)
:vertexPlane_(NULL),
 vertexPool_(NULL),
 xSize_(xSize),
 ySize_(ySize),
 initialValue_(initialValue)
{
    resize(xSize_, ySize_);
}

template<class T>
VertexPlane<T>::VertexPlane(const VertexPlane& original)
:vertexPlane_(NULL),
 vertexPool_(NULL),
 xSize_(original.xSize_),
 ySize_(original.ySize_),
 initialValue_(original.initialValue_)
{
    copyPlane(original);
}

template<class T>
inline
VertexPlane<T>::~VertexPlane(){
    releasePlane ();
}

template<class T>
void VertexPlane<T>::operator=(const VertexPlane& original)
{
    initialValue_ = original.initialValue_;
    copyPlane(original);
}

template<class T>
inline
int VertexPlane<T>::getXSize () const
{
    return xSize_;
}

template<class T>
inline
int VertexPlane<T>::getYSize () const
{
    return ySize_;
}

template<class T>
inline
T& VertexPlane<T>::initialValue()
{
    return initialValue_;
}

template<class T>
inline
const T& VertexPlane<T>::initialValue() const
{
    return initialValue_;
}

template<class T>
inline
T& VertexPlane<T>::vertex(int x, int y)
{
    assert( x >= 0 );   assert( x < xSize_ );
    assert( y >= 0 );   assert( y < ySize_ );

    return vertexPlane_[x][y];
}

template<class T>
inline
const T& VertexPlane<T>::vertex(int x, int y) const
{
    assert( x >= 0 );   assert( x < xSize_ );
    assert( y >= 0 );   assert( y < ySize_ );

    return vertexPlane_[x][y];
}

template<class T>
void VertexPlane<T>::resize(int xSize, int ySize)
{
    assert( xSize >= 0 );
    assert( ySize >= 0 );

    releasePlane ();

    //Do not move the following lines before releasePlane()
	xSize_ = xSize;
	ySize_ = ySize;

    if( (xSize_ != 0) && (ySize_ != 0) ) {
        vertexPool_ = new std::vector<T>(xSize_ * ySize_ , initialValue_);
        assignPoolResource();
    }
}

template<class T>
inline
void VertexPlane<T>::reset(){
    resize(xSize_, ySize_);
}

template<class T>
void VertexPlane<T>::copyPlane(const VertexPlane& original)
{
    releasePlane();

	xSize_ = original.xSize_;
	ySize_ = original.ySize_;

    if( (xSize_ != 0) && (ySize_ != 0) ) {
        vertexPool_ = new std::vector<T>(*original.vertexPool_);
        assignPoolResource();
    }
}

template<class T>
void VertexPlane<T>::assignPoolResource()
{
    assert( vertexPool_ != NULL);
    assert( vertexPlane_ == NULL);

    vertexPlane_ = new T*[xSize_];

    int index = 0;
    for(int x = 0; x < xSize_; ++x, index += ySize_) {
        vertexPlane_[x] = &((*vertexPool_)[index]);
    }
}

template<class T>
void VertexPlane<T>::releasePlane ()
{
    if(vertexPool_ != NULL) {
        delete vertexPool_;
        vertexPool_ = NULL;
    }
	if(vertexPlane_ != NULL) {
		delete[]  vertexPlane_;
        vertexPlane_ = NULL;
	}

	xSize_ = 0;
	ySize_ = 0;
}

///@brief The data structure for presenting the routing edges in global routing area.
///@details User can specify the data structure of routing edges by their own, and
///         the default data structure of routing edges is a integer.
template<class T>
class EdgePlane {
    public:
                EdgePlane(int xSize,
                          int ySize,
                          T initialValue,
                          int edgeNumber = 2);

                EdgePlane(const EdgePlane&);

                ~EdgePlane();

                void operator=(const EdgePlane&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void        resize(int xSize, int ySize);

    ///@brief Get the map size in x-axis
    int         getXSize () const;

    ///@brief Get the map size in y-axis
    int         getYSize () const;

    ///@brief Reset every vertex to initial value.
    void        reset();

    ///@brief Get the initial value, and it can be changed.
    T&          initialValue();

    ///@brief Get the initial value, and it is read-only.
    const T&    initialValue() const;

    ///@brief Get the specified edge
    T&          edge(int x, int y, Jm::DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const T&    edge(int x, int y, Jm::DirectionType) const;

    ///@brief Get the specified edge. 
    ///The direction id is using JR Direction 
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    T&          edge(int x, int y, int dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction 
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    const T&    edge(int x, int y, int dir) const;

    private:
    ///The routing bins used to connect the routing edges.
    class Vertex {
        public:
                Vertex(T& initialValue);
                T edge[2];
    };

	private:
    ///The real data structure of plane
    Vertex**   edgePlane_;
    std::vector<Vertex>* edgePool_;

    ///Plane size
    int         xSize_;
    int         ySize_;

    ///The initial value
    T           initialValue_;
    int         edgeNumber_;

    static const int  transferTable[2][2];
    static const int  Jr2JmTransferTable[4];

    private:
    ///Copy the edgePlane
    void        copyPlane(const EdgePlane&);

    ///Release the memory used by plane
    void        releasePlane();

    ///Because the Vertex only contain the North, East edges, if the user
    ///want to access the South, West edges, we will need to transfer
    ///the coordinate and direction to the available value.
    void        transferLocation(int* x, int* y,
                                 int index) const;

    ///This function will assign xxxPool_'s memeory resource to xxxPlane_ pointers
    void        assignPoolResource ();
};

template<class T>
EdgePlane<T>::EdgePlane(int xSize, int ySize,
                        T initialValue, int edgeNumber)
:edgePlane_(NULL),
 edgePool_(NULL),
 xSize_(xSize),
 ySize_(ySize),
 initialValue_(initialValue),
 edgeNumber_(edgeNumber)
{
    resize(xSize_, ySize_);
}

template<class T>
EdgePlane<T>::EdgePlane(const EdgePlane& original)
:edgePlane_(NULL),
 edgePool_(NULL),
 xSize_(original.xSize_),
 ySize_(original.ySize_),
 initialValue_(original.initialValue_),
 edgeNumber_(original.edgeNumber_)
{
    copyPlane(original);
}

template<class T>
EdgePlane<T>::~EdgePlane(){
    releasePlane ();
}

template<class T>
void EdgePlane<T>::operator=(const EdgePlane& original)
{
    initialValue_ = original.initialValue_;
    edgeNumber_ = original.edgeNumber_;
    copyPlane(original);
}

template<class T>
const int EdgePlane<T>::transferTable[2][2] = 
{{0, -1}, {-1, 0}};

template<class T>
const int EdgePlane<T>::Jr2JmTransferTable[4] = 
{0, 1, 3, 2};

template<class T>
inline
int EdgePlane<T>::getXSize () const
{
    return xSize_;
}

template<class T>
inline
int EdgePlane<T>::getYSize () const
{
    return ySize_;
}

template<class T>
inline
T& EdgePlane<T>::initialValue ()
{
    return initialValue_;
}

template<class T>
inline
const T& EdgePlane<T>::initialValue () const
{
    return initialValue_;
}

template<class T>
inline
T& EdgePlane<T>::edge(int x, int y, Jm::DirectionType dir)
{
    assert( x >= 0 );   assert( x < xSize_ );
    assert( y >= 0 );   assert( y < ySize_ );
    assert( dir >= 0 ); assert( (static_cast<int>(dir) >> 1) < edgeNumber_ );

    //If the direction is South, West, Down edges, we will need to change it
    //to available direction (North, East, Up) and coordinate
    if( (static_cast<int>(dir) & 0x01) != 0) {
        transferLocation(&x, &y, (static_cast<int>(dir) >> 1));
    }

	return edgePlane_[x][y].edge[ (static_cast<int>(dir) >> 1) ];
}

template<class T>
inline
const T& EdgePlane<T>::edge(int x, int y, Jm::DirectionType dir) const
{
    return edge(x, y, dir);
}

template<class T>
inline
T& EdgePlane<T>::edge(int x, int y, int JrDir)
{
    assert( JrDir >= 0 && JrDir < 4); 

    Jm::DirectionType dir = static_cast<Jm::DirectionType>( Jr2JmTransferTable[JrDir] );
    return edge(x, y, dir);
}

template<class T>
inline
const T& EdgePlane<T>::edge(int x, int y, int JrDir) const
{
    return edge(x, y, JrDir);
}

template<class T>
void EdgePlane<T>::resize(int xSize, int ySize)
{
    assert( xSize >= 0 );
    assert( ySize >= 0 );

    releasePlane ();

    //Do not move the following 3 lines before releaseColorMap()
	xSize_ = xSize;
	ySize_ = ySize;
    //edgeNumber_ = edgeNumber;

    if( (xSize_ != 0) && (ySize_ != 0)) {
        edgePool_ = new std::vector<Vertex>( (xSize_ * ySize_), Vertex(initialValue_));
        assignPoolResource();
    }
}

template<class T>
inline
void EdgePlane<T>::reset(){
    resize(xSize_, ySize_);
}

template<class T>
void EdgePlane<T>::copyPlane(const EdgePlane& original)
{
    releasePlane();

	xSize_ = original.xSize_;
	ySize_ = original.ySize_;

    edgePool_ = new std::vector<Vertex>(*original.edgePool_);
    assignPoolResource();
}

template<class T>
void EdgePlane<T>::assignPoolResource()
{
    assert(edgePool_ != NULL);
    assert(edgePlane_ == NULL);

    edgePlane_ = new Vertex*[xSize_];

    int index = 0;
    for(int x = 0; x < xSize_; ++x) {
        edgePlane_[x] = &((*edgePool_)[index]);
        index += ySize_;
    }
}

template<class T>
void EdgePlane<T>::releasePlane ()
{
    if(edgePool_ != NULL) {
        delete edgePool_;
        edgePool_ = NULL;
    }
	if(edgePlane_ != NULL) {
        delete[]   edgePlane_;
        edgePlane_ = NULL;
	}

	xSize_ = 0;
	ySize_ = 0;
}

template<class T>
inline
void EdgePlane<T>::transferLocation(
        int* x,
        int* y,
        int  index) const
{
    *x += transferTable[index][0];
    *y += transferTable[index][1];

    assert( (*x) >= 0 );  assert( (*x) < xSize_ );
    assert( (*y) >= 0 );  assert( (*y) < ySize_ );
}

template<class T>
EdgePlane<T>::Vertex::Vertex(T& value)
{
    for(int i = 0; i < 2; ++i) {
        edge[i] = value;
    }
}

template<class VertexT, class EdgeT>
class Plane {
    public:
                Plane(int xSize,
                      int ySize,
                      VertexT vertexInitialValue,
                      EdgeT edgeInitialValue,
                      int edgeNumber = 2);

                Plane(const Plane&);

                ~Plane();

                void operator=(const Plane&);

    ///@brief Change the size of plane. Every vertex will reset to initial value.
    void        resize(int xSize, int ySize);

    ///@brief Get the map size in x-axis
    int         getXSize () const;

    ///@brief Get the map size in y-axis
    int         getYSize () const;

    ///@brief Reset every vertex to initial value.
    void        reset();

    ///@brief Get the initial value, and it can be changed.
    std::pair<VertexT&, EdgeT&>
                initialValue();

    ///@brief Get the initial value, and it is read-only.
    std::pair<const VertexT&, const EdgeT&>
                initialValue() const;

    ///@brief Get the specified vertex
    VertexT&    vertex(int x, int y);//, int z);

    ///@brief Get the specified vertex, and the vertex is read-only.
    const VertexT&  vertex(int x, int y) const;

    ///@brief Get the specified edge
    EdgeT&      edge(int x, int y, Jm::DirectionType);

    ///@brief Get the specified edge, and the edge is read-only.
    const EdgeT&    edge(int x, int y, Jm::DirectionType) const;

    ///@brief Get the specified edge. 
    ///The direction id is using JR Direction 
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    EdgeT&          edge(int x, int y, int dir);

    ///@brief Get the specified edge, and the edge is read-only.
    ///The direction id is using JR Direction 
    /// (North, South, West, East) which is different from JR Driection
    /// (North, South, East, West)
    const EdgeT&    edge(int x, int y, int dir) const;

    private:
    VertexPlane<VertexT> vertexPlane_;
    EdgePlane<EdgeT> edgePlane_;
};

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(int xSize, int ySize,
                             VertexT vertexInitialValue,
                             EdgeT edgeInitialValue,
                             int edgeNumber)
:vertexPlane_(xSize, ySize, vertexInitialValue),
 edgePlane_(xSize, ySize, edgeInitialValue, edgeNumber)
{}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::Plane(const Plane& original)
:vertexPlane_(original.vertexPlane_),
 edgePlane_(original.edgePlane_)
{}

template<class VertexT, class EdgeT>
Plane<VertexT, EdgeT>::~Plane()
{}

template<class VertexT, class EdgeT>
void Plane<VertexT, EdgeT>::operator=(const Plane& original)
{
    edgePlane_ = original.edgePlane_;
    vertexPlane_ = original.vertexPlane_;
}


template<class VertexT, class EdgeT>
inline
void Plane<VertexT, EdgeT>::resize(int xSize, int ySize)
{
    vertexPlane_.resize(xSize, ySize);
    edgePlane_.resize(xSize, ySize);
}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getXSize() const
{
    return vertexPlane_.getXSize();
}

template<class VertexT, class EdgeT>
inline
int Plane<VertexT, EdgeT>::getYSize() const
{
    return vertexPlane_.getYSize();
}

template<class VertexT, class EdgeT>
inline
void Plane<VertexT, EdgeT>::reset()
{
    vertexPlane_.reset();
    edgePlane_.reset();
}

template<class VertexT, class EdgeT>
inline
std::pair<VertexT&, EdgeT&> Plane<VertexT, EdgeT>::initialValue()
{
    return std::pair<VertexT&, EdgeT&> (
            vertexPlane_.initialValue(), 
            edgePlane_.initialValue() );
}

template<class VertexT, class EdgeT>
inline
std::pair<const VertexT&, const EdgeT&>
Plane<VertexT, EdgeT>::initialValue() const
{
    return std::pair<const VertexT&, const EdgeT&> (
            vertexPlane_.initialValue(), 
            edgePlane_.initialValue() );
}

template<class VertexT, class EdgeT>
inline
VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y)
{
    return vertexPlane_.vertex(x, y);
}

template<class VertexT, class EdgeT>
inline
const VertexT& Plane<VertexT, EdgeT>::vertex(int x, int y) const
{
    return vertexPlane_.vertex(x, y);
}

template<class VertexT, class EdgeT>
inline
EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, Jm::DirectionType dirType)
{
    return edgePlane_.edge(x, y, dirType);
}

template<class VertexT, class EdgeT>
inline
const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, Jm::DirectionType dirType) const
{
    return edgePlane_.edge(x, y, dirType);
}

template<class VertexT, class EdgeT>
inline
EdgeT& Plane<VertexT, EdgeT>::edge(int x, int y, int JrDir)
{
    return edgePlane_.edge(x, y, JrDir);
}

template<class VertexT, class EdgeT>
inline
const EdgeT&
Plane<VertexT, EdgeT>::edge(int x, int y, int JrDir) const
{
    return edgePlane_.edge(x, y, JrDir);
}
#endif //INC_PLANE_H
