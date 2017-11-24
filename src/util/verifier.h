#ifndef INC_VERIFIER_H
#define INC_VERIFIER_H

#include "traversemap.h"

#include "../grdb/RoutingRegion.h"
#include "../grdb/plane.h"
#include "../misc/filehandler.h"

#include <string>
#include <vector>

class WireSegment{
    public:
        int startX;
        int startY;
        int startZ;
        int endX;
        int endY;
        int endZ;

		//1: x-axis, 2: y-axis, 3: z-axis, or you can define yourself
		int direction;
	public:
		WireSegment(int x1, int y1, int z1, 
					int x2, int y2, int z2,
					int direction);
};

//this is a data structure for recording straight wire segments
class WireSegments{
	private:
        std::vector<WireSegment> segments;
		int lineDirection(
			int x1, int y1, int z1,
		    int x2, int y2, int z2
		);
	public:
		WireSegments();
		//SET Functions
		//true: new cell is the same direction with previous cell
		//false: new cell is not the same direction with previous 
		//		 cell, new wire segment need to be added.
		bool add_segment(
			int x1, int y1, int z1,
			int x2, int y2, int z2
		);
		bool extend_segment(int wirePos, int x, int y, int z);
		void remove(int wirePos);
		//GET Functions
		int size();
		int get_segmentNumber();
		int get_segmentStartX(int wirePos);
		int get_segmentStartY(int wirePos);
		int get_segmentStartZ(int wirePos);
		int get_segmentEndX(int wirePos);
		int get_segmentEndY(int wirePos);
		int get_segmentEndZ(int wirePos);
		//int get_length(int wirePos);
		//int get_totalLength();
};

class Verifier {
	public:
		Verifier(const char* inputFileName, const char* resultFileName);
        void    verify();

    private:
        struct RoutingTile {
                RoutingTile(int color, int viaCutCount)
                    : color(color), viaCutCount(viaCutCount) {}
            ///Used for checking wire connectivity
            int color;
            
            ///Via cut count of a tile
            int viaCutCount;
        };

        struct RoutingEdge {
                RoutingEdge(int color = -1, int usage = 0)
                    : color(color), usage(usage) {}
            ///Used for checking duplicate wires, calculating total wire length
            ///and routing edge usage. Every vertex of edgeTraverseMap need 3 edges -
            ///North, East, and Up. And EdgePlane provide 3 edges for every vertex. 
            ///So we use EdgePlane instead of EdgeColorMap which only provide 2 edges.
            int color;

            ///Track Usage
            int usage;
        };

	private:
        std::string inputFileName;
        std::string resultFileName;
        std::string delims;
        Jm::FileHandler fh_;        ///< Result file handler
		RoutingRegion* rr;          ///< Test case information
        std::vector< WireSegments >* netWires;  ///<Wire information of nets
        ///The tile count that passed by a net
        std::vector<int>* netTileNum;
        std::vector< Plane<RoutingTile, RoutingEdge> >* routingSpace_;
        int XYWireLength;
        int viaWireLength;
		int overflow;
		int maxOverflow;
        int viaOverflow;
        int viaMaxOverflow;
        int wireBendCount;
		int netWithoutOverflow;
		int netHasUnconnectedPin;
		int netHasUnconnectedWire;
		int netHasDuplicateWireCount;

    private:
        void initialVerifierMemorySpace ();

        void printResult ();

		///Parse result file, and update current congestion map
		void parseNetWires();

		///Called by parseNetWires(), only parse one net a time
		void parseOneNetWires(int netPos);

		///Count total overflow and get the max overflow
		void countTotalOverflow();

		///Count # of net without overflow
		void countNetWithoutOverflow();

		///Check if a segment is on a overflowed edge 
		bool isSegmentOverflow(int x1 , int y1, int z1,
                               int x2, int y2, int z2);

		///@brief Begin the verify processes
		void check();

        ///@brief This function will do 3 important tasks:
        ///1. Put the net on the traversal color map for further checking
        ///   process.
        ///2. Add wire length to total XY wire length. 
        ///3. Add usage to edge usage count.
		void initialNetRouteStatus(int netPos);

        ///@brief Route a wire which start from (x, y, z) and extend to dir direction
        ///with length = length on routingSpace_. Then update
        ///the total wire length and via wire length.
        bool routeWireOnMaps(int netPos, int x, int y, int z,
                             int length, int dir);

        ///@brief Check pin connection for specified net
        ///@pre The wire segment must be routed on routingSpace_
		void checkPinConnectivity(int netPos);

        ///@brief Check wire connection for specified net
        ///@pre The wire segment must be routed on routingSpace_
		void checkWireConnectivity(int netPos);

        ///@brief Check if the adjecent tile was routed by the same net
        ///@pre The wire segment must be routed on edgeTraverseMap
        ///@param dirId 0:North, 1:South, 2:West, 3:East, 4:Up, 5:Down
		bool isNeighborTileSameNet(int curX, int curY, int curZ,
                                   int netPos, int dirId);

        void releaseMemory ();
};
#endif /*INC_VERIFIER_H*/
