#include "verifier.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>

#include "../grdb/parser.h"

#include "../grdb/RoutingComponent.h"
#include "../misc/geometry.h"

#define MAX_STRING_BUFER_LENGTH 1024

using namespace std;
using namespace Jm;

Verifier::Verifier(const char* inputFileName, const char* resultFileName) :
        inputFileName(inputFileName), resultFileName(resultFileName), delims("(,)- \n"), fh_(resultFileName, FileHandler::AutoFileType), rr(NULL), netWires(NULL), netTileNum(NULL), routingSpace_(
        NULL), XYWireLength(0), viaWireLength(0), overflow(0), maxOverflow(0), viaOverflow(0), viaMaxOverflow(0), wireBendCount(0), netHasUnconnectedPin(0), netHasUnconnectedWire(0), netHasDuplicateWireCount(
                0) {
}

void Verifier::verify() {
    //Open result file
    if (!fh_.open(FileHandler::ReadAccessMode)) {
        cerr << "Error opening result file: " << resultFileName << endl;
        abort();
    }

    cout << "Begin to verify routing result." << endl;
    cout << "Parsing input file.." << endl;
    cout << "\tTest case:   " << inputFileName << endl;
    cout << "\tResult file: " << resultFileName << endl;

    //Readin Routing data
    RoutingRegion rr;
    Parser07 parser(inputFileName, FileHandler::AutoFileType, rr);
    parser.parse();

    initialVerifierMemorySpace();

    //parse result file (wire segments)
    parseNetWires();

    //Begin Evaluation
    cout << "Verifing..." << endl;
    check();

    printResult();

    releaseMemory();
}

void Verifier::printResult() {
    //output result
    cout << "########################## RESULT #################################" << endl;
    cout << "# of unconnected net because of pin: " << netHasUnconnectedPin << endl;
    cout << "# of unconnected net because of wire: " << netHasUnconnectedWire << endl;
    cout << "# of net has duplicate wires: " << netHasDuplicateWireCount << endl;
    cout << "# of overflow: " << overflow << endl;
    cout << "max overflow: " << maxOverflow << endl;
    cout << "# of net without overflow: " << netWithoutOverflow << " / " << rr->get_netNumber() << endl;

    //cout << "# of via overflow: " << viaOverflow << endl;
    //cout << "via max overflow: " << viaMaxOverflow << endl;
    cout << " Wire bend count: " << wireBendCount << endl;
    cout << "  XY  WireLength: " << XYWireLength << endl;
    cout << " Via  WireLength: " << viaWireLength << endl;
    cout << "Total WireLength: " << XYWireLength + viaWireLength << endl;
    cout << "Total    Cost:    " << XYWireLength + 3 * viaWireLength << endl;
}

void Verifier::initialVerifierMemorySpace() {
    netWithoutOverflow = rr->get_netNumber();
    netWires = new vector<WireSegments>(rr->get_netNumber());
    netTileNum = new vector<int>(rr->get_netNumber(), 0);

    routingSpace_.resize(rr->get_layerNumber(),
            Plane3d<RoutingTile, RoutingEdge>(this->rr->get_gridx(), this->rr->get_gridy(),1   ));
}

void Verifier::releaseMemory() {
    fh_.close();
    if (rr != NULL) {
        delete rr;
        rr = NULL;
    }
    if (netWires != NULL) {
        delete netWires;
        netWires = NULL;
    }
    if (netTileNum != NULL) {
        delete netTileNum;
        netTileNum = NULL;
    }
    if (routingSpace_ != NULL) {
        delete routingSpace_;
        routingSpace_ = NULL;
    }
}

void Verifier::parseNetWires() {
    //parse one net a time
    for (int netPos = 0; netPos < this->rr->get_netNumber(); ++netPos) {
        this->parseOneNetWires(netPos);
    }
}

void Verifier::parseOneNetWires(int netPos) {
    //char buf[BUFSIZE];
    char* stringBuffer = new char[MAX_STRING_BUFER_LENGTH];
    char* token;
    bool keepRead = true;
    int x1, y1, z1, x2, y2, z2;

    fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
    string netName = std::strtok(stringBuffer, this->delims.c_str());
    int netId = std::atoi(std::strtok(NULL, this->delims.c_str()));

    if (std::strcmp(netName.c_str(), rr->get_netName(netPos)) != 0) {
        cout << "The " << netId << "nd net of testcase and result file are not the same." << endl;
        cout << "\tNet Name of " << netPos << "th in input file: " << rr->get_netName(netPos) << endl;
        cout << "\tNet Name of " << netPos << "th in output file: " << netName.c_str() << endl;
        abort();
    }

    /*
     while(true) {
     //skip the first line
     fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
     string netName = strtok(stringBuffer, this->delims.c_str());
     int netId = atoi(strtok(NULL, this->delims.c_str()));
     int segmentNumber = atoi(strtok(NULL, this->delims.c_str()));

     if(segmentNumber != 0) {
     if(strcmp(netName.c_str(), rr->get_netName(netPos)) != 0) {
     cout << "Net order of output file and input file are not the same."
     << endl;
     cout << "\tNet Name of " << netPos << "th in input file: "
     << rr->get_netName(netPos) << endl;
     cout << "\tNet Name of " << netPos << "th in output file: "
     << netName.c_str() << endl;
     abort();
     }
     break;
     } else {
     //skip "!" line
     fh_.skipline();
     }
     }
     */

    while (keepRead == true) {
        //read one line
        fh_.getline(stringBuffer, MAX_STRING_BUFER_LENGTH);
        //x1 or !
        token = strtok(stringBuffer, this->delims.c_str());
        if (strncmp(token, "!", 1) == 0)
            keepRead = false;
        else {
            //x of first cell
            x1 = atoi(token);
            //y of first cell
            token = strtok(NULL, this->delims.c_str());
            y1 = atoi(token);
            //z of first cell
            token = strtok(NULL, this->delims.c_str());
            z1 = atoi(token);
            //x of second cell
            token = strtok(NULL, this->delims.c_str());
            x2 = atoi(token);
            //y of second cell
            token = strtok(NULL, this->delims.c_str());
            y2 = atoi(token);
            //z of second cell
            token = strtok(NULL, this->delims.c_str());
            z2 = atoi(token);
            //tranfer xyz from coordinate to tile
            x1 = (x1 - this->rr->get_llx()) / this->rr->get_tileWidth();
            x2 = (x2 - this->rr->get_llx()) / this->rr->get_tileWidth();
            y1 = (y1 - this->rr->get_lly()) / this->rr->get_tileHeight();
            y2 = (y2 - this->rr->get_lly()) / this->rr->get_tileHeight();
            z1--;
            z2--;
            (*netWires)[netPos].add_segment(x1, y1, z1, x2, y2, z2);
        }
    }
}

void Verifier::countNetWithoutOverflow() {
    int x1, y1, z1, x2, y2, z2;
    for (int netPos = 0; netPos < this->rr->get_netNumber(); ++netPos) {
        for (int wirePos = 0; wirePos < (*netWires)[netPos].size(); ++wirePos) {
            x1 = (*netWires)[netPos].get_segmentStartX(wirePos);
            y1 = (*netWires)[netPos].get_segmentStartY(wirePos);
            z1 = (*netWires)[netPos].get_segmentStartZ(wirePos);
            x2 = (*netWires)[netPos].get_segmentEndX(wirePos);
            y2 = (*netWires)[netPos].get_segmentEndY(wirePos);
            z2 = (*netWires)[netPos].get_segmentEndZ(wirePos);
            //check if any wire segments of a net pass any overflow edges
            if (isSegmentOverflow(x1, y1, z1, x2, y2, z2) == true) {
                --netWithoutOverflow;
                //begin to check next net
                break;
            }
        }                    //end of inner for loop
    }                    //end of outer for loop
}

bool Verifier::isSegmentOverflow(int x1, int y1, int z1, int x2, int y2, int z2) {
    int segmentLength;
    //x-direction wire segment
    if (x1 != x2 && y1 == y2 && z1 == z2) {
        segmentLength = abs(x2 - x1);
        //z1--;
        for (int i = 0; i < segmentLength; ++i) {
            if (this->rr->capacity(z1, x1 + i, y1, x1 + i + 1, y1) < (*routingSpace_)[z1].edge(x1 + i, y1, DIR_EAST).usage) {
                return true;
            }
        }
        return false;
    }
    //y-direction wire segment
    else if (x1 == x2 && y1 != y2 && z1 == z2) {
        segmentLength = abs(y2 - y1);
        //z1--;
        for (int i = 0; i < segmentLength; ++i) {
            if (this->rr->capacity(z1, x1, y1 + i, x1, y1 + i + 1) < (*routingSpace_)[z1].edge(x1, y1 + i, DIR_NORTH).usage) {
                return true;
            }
            return false;
        }
    }
    return false;
    //z-direction wire segment
}

void Verifier::countTotalOverflow() {
    int maxCapacity;
    int curCapacity;
    int diff;
    for (int layer = 0; layer < this->rr->get_layerNumber(); ++layer) {
        for (int x = 0; x < this->rr->get_gridx() - 1; ++x) {
            for (int y = 0; y < this->rr->get_gridy(); ++y) {
                maxCapacity = this->rr->capacity(layer, x, y, x + 1, y);
                curCapacity = (*routingSpace_)[layer].edge(x, y, DIR_EAST).usage;
                if (curCapacity > maxCapacity) {
                    diff = curCapacity - maxCapacity;
                    //add to # of overflow
                    this->overflow += diff;
                    //update max overflow
                    if (diff > this->maxOverflow)
                        maxOverflow = diff;
                }
            }
        }
        for (int y = 0; y < this->rr->get_gridy() - 1; ++y) {
            for (int x = 0; x < this->rr->get_gridx(); ++x) {
                maxCapacity = this->rr->capacity(layer, x, y, x, y + 1);
                curCapacity = (*routingSpace_)[layer].edge(x, y, DIR_NORTH).usage;
                if (curCapacity > maxCapacity) {
                    diff = curCapacity - maxCapacity;
                    //add to # of overflow
                    this->overflow += diff;
                    //update max overflow
                    if (diff > this->maxOverflow)
                        maxOverflow = diff;
                }
            }
        }
        for (int x = 0; x < this->rr->get_gridx(); ++x) {
            //maxCapacity = rr->get_tileWidth();
            maxCapacity = 0;
            for (int y = 0; y < this->rr->get_gridy(); ++y) {
                //curCapacity = routingSpace_->vertex(x, y, layer).viaCutCount;
                curCapacity = (*routingSpace_)[layer].vertex(x, y).viaCutCount;
                if (curCapacity > maxCapacity) {
                    diff = curCapacity - maxCapacity;
                    //add to # of overflow
                    viaOverflow += diff;
                    //update max overflow
                    if (diff > viaMaxOverflow)
                        viaMaxOverflow = diff;
                }
            }
        }
    }
    /* TroyLee: Dump Congestion Map in Graphical */
#if 1
    cout << "Starting Dumping Congestion Map in Graphical File: " << inputFileName + ".png" << endl;
    vector<vector<double> > DumpCongMap(this->rr->get_gridx() * 2 + 10, vector<double>(this->rr->get_gridy() * 2 + 10, 0.0));
    /*
     for( int x=0 ; x<this->rr->get_gridx() ; ++x ) {

     DumpCongMap[x].insert(DumpCongMap[x].begin(), this->rr->get_gridy()*2+10, 0.0);
     }
     */
    double max_cong = 0.0;
    for (int x = 0; x < this->rr->get_gridx(); ++x) {
        for (int y = 0; y < this->rr->get_gridy(); ++y) {
            if (x != this->rr->get_gridx() - 1) {
                // East
                double cap = 0, usage = 0;
                for (int layer = 0; layer < this->rr->get_layerNumber(); ++layer) {
                    cap += this->rr->capacity(layer, x, y, x + 1, y);
                    usage += (*routingSpace_)[layer].edge(x, y, DIR_EAST).usage;
                }
                DumpCongMap[x * 2 + 1][y * 2] = cap != 0.0 ? usage / cap : 0.0;
                max_cong = cap != 0.0 ? (max_cong < usage / cap ? usage / cap : max_cong) : max_cong;
            }
            if (y != this->rr->get_gridy() - 1) {
                //North
                double cap = 0, usage = 0;
                for (int layer = 0; layer < this->rr->get_layerNumber(); ++layer) {
                    cap += this->rr->capacity(layer, x, y, x, y + 1);
                    usage += (*routingSpace_)[layer].edge(x, y, DIR_NORTH).usage;
                }
                DumpCongMap[x * 2][y * 2 + 1] = cap != 0.0 ? usage / cap : 0.0;
                max_cong = cap != 0.0 ? (max_cong < usage / cap ? usage / cap : max_cong) : max_cong;
            }
        }
    }
    cout << "Drawing Congestion Map in Graphical File: " << inputFileName + ".png" << endl;
    fstream fcongmap((inputFileName + ".png").c_str(), ios::out);
    for (int y = 0; y < this->rr->get_gridy() * 2 - 1; ++y) {
        for (int x = 0; x < this->rr->get_gridx() * 2 - 1; ++x) {
            if ((!(x & 0x01) && !(y & 0x01)) || ((x & 0x01) && (y & 0x01))) {
                double avg_cong = 0.0;
                int cnt = 0;
                if (x < rr->get_gridx() * 2 - 1) {
                    // East
                    avg_cong += DumpCongMap[x + 1][y];
                    cnt++;
                }
                if (x > 0) {
                    // West
                    avg_cong += DumpCongMap[x - 1][y];
                    cnt++;
                }
                if (x < rr->get_gridy() * 2 - 1) {
                    // North
                    avg_cong += DumpCongMap[x][y + 1];
                    cnt++;
                }
                if (y > 0) {
                    // South
                    avg_cong += DumpCongMap[x][y - 1];
                    cnt++;
                }
                DumpCongMap[x][y] = avg_cong / cnt;
            }
            fcongmap << DumpCongMap[x][y] << '\t';
        }
        fcongmap << endl;
    }
    cout << "End Drawing: " << inputFileName + ".png" << endl;
#endif
    /* TroyLee: End Dump */
}

void Verifier::check() {
    for (int netPos = 0; netPos < rr->get_netNumber(); ++netPos) {
        initialNetRouteStatus(netPos);
        //check pin connectivity
        checkPinConnectivity(netPos);
        //check wire connectivity
        checkWireConnectivity(netPos);
    }

    countTotalOverflow();
    countNetWithoutOverflow();
}

void Verifier::initialNetRouteStatus(int netPos) {
    bool netHasDuplicateWire = false;

    for (int wirePos = 0; wirePos < (*netWires)[netPos].size(); ++wirePos) {
        bool hasDuplicateSegment = false;
        int x1 = (*netWires)[netPos].get_segmentStartX(wirePos);
        int y1 = (*netWires)[netPos].get_segmentStartY(wirePos);
        int z1 = (*netWires)[netPos].get_segmentStartZ(wirePos);
        int x2 = (*netWires)[netPos].get_segmentEndX(wirePos);
        int y2 = (*netWires)[netPos].get_segmentEndY(wirePos);
        int z2 = (*netWires)[netPos].get_segmentEndZ(wirePos);
        int segmentLength = 0;
        //x-direction wire segment
        if (x1 != x2 && y1 == y2 && z1 == z2) {
            segmentLength = abs(x2 - x1);
            hasDuplicateSegment = routeWireOnMaps(netPos, x1, y1, z1, segmentLength, 0);
        }
        //y-direction wire segment
        else if (x1 == x2 && y1 != y2 && z1 == z2) {
            segmentLength = abs(y2 - y1);
            hasDuplicateSegment = routeWireOnMaps(netPos, x1, y1, z1, segmentLength, 1);
        }
        //z-direction wire second
        else if (x1 == x2 && y1 == y2 && z1 != z2) {
            segmentLength = abs(z2 - z1);

            if (segmentLength & 1 || segmentLength == 4) {
                bool wireHasBend = false;
                for (int i = 0; i < this->rr->get_layerNumber() - 1; ++i) {
                    if ((*routingSpace_)[i].edge(x1, y1, DIR_UP).color == netPos) {
                        wireHasBend = true;
                        break;
                    }
                }
                if (wireHasBend == false) {
                    ++wireBendCount;
                }
            }

            hasDuplicateSegment = routeWireOnMaps(netPos, x1, y1, z1, segmentLength, 2);
        }

        if (netHasDuplicateWire == false && hasDuplicateSegment == true) {
            netHasDuplicateWire = true;
            ++netHasDuplicateWireCount;
        }
    }
}

bool Verifier::routeWireOnMaps(int netPos, int x, int y, int z, int length, int dir) {
    int xBasicShift = 0;
    int yBasicShift = 0;
    int zBasicShift = 0;
    int xTotalShift = 0;
    int yTotalShift = 0;
    int zTotalShift = 0;
    DirectionType dirType = DIR_EAST;
    bool hasDuplicate = false;

    switch (dir) {
    case 0:
        xBasicShift = 1;
        dirType = DIR_EAST;
        break;
    case 1:
        yBasicShift = 1;
        dirType = DIR_NORTH;
        break;
    case 2:
        zBasicShift = 1;
        dirType = DIR_UP;
        break;
    default:
        break;
    }

    //set edges and tiles pass by the wire
    for (int i = 0; i < length; ++i) {
        if ((*routingSpace_)[z + zTotalShift].edge(x + xTotalShift, y + yTotalShift, dirType).color != netPos) {
            (*routingSpace_)[z + zTotalShift].edge(x + xTotalShift, y + yTotalShift, dirType).color = netPos;

            if (static_cast<int>(dirType) < 4) {
                ++XYWireLength;

                //Add min. wire width to usage
                (*routingSpace_)[z + zTotalShift].edge(x + xTotalShift, y + yTotalShift, dirType).usage += rr->get_layerMinWidth(z + zTotalShift);

                //Add min. wire spacing to usage
                (*routingSpace_)[z + zTotalShift].edge(x + xTotalShift, y + yTotalShift, dirType).usage += rr->get_layerMinSpacing(z + zTotalShift);
            } else {
                ++viaWireLength;
            }
        } else {
            hasDuplicate = true;
        }

        if ((*routingSpace_)[z + zTotalShift].vertex(x + xTotalShift, y + yTotalShift).color != netPos) {
            (*netTileNum)[netPos]++;
            (*routingSpace_)[z + zTotalShift].vertex(x + xTotalShift, y + yTotalShift).color = netPos;

            ++(*routingSpace_)[z + zTotalShift].vertex(x + xTotalShift, y + yTotalShift).viaCutCount;
        }

        if ((*routingSpace_)[z + zTotalShift + zBasicShift].vertex(x + xTotalShift + xBasicShift, y + yTotalShift + yBasicShift).color != netPos) {
            (*netTileNum)[netPos]++;
            (*routingSpace_)[z + zTotalShift + zBasicShift].vertex(x + xTotalShift + xBasicShift, y + yTotalShift + yBasicShift).color = netPos;

            ++(*routingSpace_)[z + zTotalShift + zBasicShift].vertex(x + xTotalShift + xBasicShift, y + yTotalShift + yBasicShift).viaCutCount;
        }

        xTotalShift += xBasicShift;
        yTotalShift += yBasicShift;
        zTotalShift += zBasicShift;
    }

    return hasDuplicate;
}

void Verifier::checkPinConnectivity(int netPos) {
    const PinptrList& netPin = this->rr->get_nPin(netPos);
    int tileX, tileY, tileZ;
    for (int pinPos = 0; pinPos < (int) netPin.size(); ++pinPos) {
        tileX = netPin[pinPos]->get_tileX();
        tileY = netPin[pinPos]->get_tileY();
        tileZ = netPin[pinPos]->get_layerId();
        if ((*routingSpace_)[tileZ].vertex(tileX, tileY).color != netPos) {
            ++netHasUnconnectedPin;
            break;
        }
    }
}

void Verifier::checkWireConnectivity(int netPos) {
    int seedX = (*netWires)[netPos].get_segmentStartX(0);
    int seedY = (*netWires)[netPos].get_segmentStartY(0);
    int seedZ = (*netWires)[netPos].get_segmentStartZ(0);
    int traversedTileNumber = 0;
    BSearchQue que;

    //Enqueue
    que.push_back(seedX, seedY, seedZ, 0);
    traversedTileNumber++;
    (*routingSpace_)[seedZ].vertex(seedX, seedY).color = -1;

    //while que is not empty, keep breadth search
    while (que.size() > 0) {
        int x = que.nextX();
        int y = que.nextY();
        int z = que.nextZ();
        //Dequeue
        que.pop_front();
        //For each adjacent edge...
        for (int i = 0; i < 6; ++i) {
            //check if the adjacent tile contains path of current net
            if (this->isNeighborTileSameNet(x, y, z, netPos, i) == true) {
                int childX = x;
                int childY = y;
                int childZ = z;
                //Transfer the direction code to x,y,z-coordinate
                switch (i) {
                //Font
                case 0:
                    childY++;
                    break;
                    //Back
                case 1:
                    childY--;
                    break;
                    //Left
                case 2:
                    childX--;
                    break;
                    //Right
                case 3:
                    childX++;
                    break;
                    //Up
                case 4:
                    childZ++;
                    break;
                    //Down
                case 5:
                    childZ--;
                    break;
                }
                //do if color != netPos (the tile have not been visited before)
                //else DO NOTHING
                if ((*routingSpace_)[childZ].vertex(childX, childY).color == netPos) {
                    //Enqueue
                    que.push_back(childX, childY, childZ, 0);
                    traversedTileNumber++;
                    (*routingSpace_)[childZ].vertex(childX, childY).color = -1;
                }
            }
            //else{
            //this tile dose not containt any wire segments of current net
            //}
        }
    }			//end of while

    if (traversedTileNumber < (*netTileNum)[netPos]) {
        this->netHasUnconnectedWire++;
    }
}

bool Verifier::isNeighborTileSameNet(int curX, int curY, int curZ, int netPos, int dirId) {
    int colorID = -1;
    switch (dirId) {
    case 0:
        if (curY >= this->rr->get_gridy() - 1)
            return false;
        colorID = (*routingSpace_)[curZ].edge(curX, curY, DIR_NORTH).color;
        break;
    case 1:
        if (curY <= 0)
            return false;
        colorID = (*routingSpace_)[curZ].edge(curX, curY - 1, DIR_NORTH).color;
        break;
    case 2:
        if (curX <= 0)
            return false;
        colorID = (*routingSpace_)[curZ].edge(curX - 1, curY, DIR_EAST).color;
        break;
    case 3:
        if (curX >= this->rr->get_gridx() - 1)
            return false;
        colorID = (*routingSpace_)[curZ].edge(curX, curY, DIR_EAST).color;
        break;
    case 4:
        if (curZ >= this->rr->get_layerNumber() - 1)
            return false;
        colorID = (*routingSpace_)[curZ].edge(curX, curY, DIR_UP).color;
        break;
    case 5:
        if (curZ <= 0)
            return false;
        colorID = (*routingSpace_)[curZ - 1].edge(curX, curY, DIR_UP).color;
        break;
    }
    if (colorID == netPos) {
        return true;
    } else
        return false;
}

/*************
 * WireSegment
 *************/
/*	vector<WireCell> cells;
 //default is no direction
 //1: go x+, 2: go x-, 3: y+, 4: y-, 5: z+, 6: z-
 int direction = 0;
 */
//there must exists two cells before creating a new wire segment
WireSegment::WireSegment(int x1, int y1, int z1, int x2, int y2, int z2, int direction) :
        startX(x1), startY(y1), startZ(z1), endX(x2), endY(y2), endZ(z2), direction(direction) {
}

/*************
 * WireSegments
 *************/
//this is a data structure for recording straight wire segments
WireSegments::WireSegments() {
}

int WireSegments::lineDirection(int x1, int y1, int z1, int x2, int y2, int z2) {
    //x direction
    if (x1 != x2 && y1 == y2 && z1 == z2) {
        return 1;	//x direction
    }
    //y direction
    else if (x1 == x2 && y1 != y2 && z1 == z2) {
        return 2;	//y direction
    }
    //z direction
    else if (x1 == x2 && y1 == y2 && z1 != z2) {
        return 3;	//z direction
    }

    //not a rectilinear wire segment or 1 & 2 is the same point
    return 0;	//not rectilinear line
}

//true: new segment is the same direction with previous cell
//false: new segment is not the same direction with previous 
//		 cell, new wire segment need to be added.
bool WireSegments::add_segment(int x1, int y1, int z1, int x2, int y2, int z2) {
    //check if the wire is rectilinear wire segment
    //if it is, it will return a non-zero int
    int wireDirection = lineDirection(x1, y1, z1, x2, y2, z2);
    if (wireDirection != 0) {
        switch (wireDirection) {
        case 1:
            this->segments.push_back(WireSegment(min(x1, x2), y1, z1, max(x1, x2), y1, z1, wireDirection));
            return true;
            break;
        case 2:
            this->segments.push_back(WireSegment(x1, min(y1, y2), z1, x1, max(y1, y2), z1, wireDirection));
            return true;
            break;
        case 3:
            this->segments.push_back(WireSegment(x1, y1, min(z1, z2), x1, y1, max(z1, z2), wireDirection));
            return true;
            break;
        default:
            return false;
        }
    } else {
        //not a rectilinear wire segment
        return false;
    }
}

bool WireSegments::extend_segment(int wirePos, int x, int y, int z) {
    if (wirePos < (int) segments.size() && wirePos >= 0) {
        int direction = segments[wirePos].direction;
        int& startX = segments[wirePos].startX;
        int& startY = segments[wirePos].startY;
        int& startZ = segments[wirePos].startZ;
        int& endX = segments[wirePos].endX;
        int& endY = segments[wirePos].endY;
        int& endZ = segments[wirePos].endZ;
        //only add new point to the wire segment if this point is on the
        //extention line of the wire segment.
        //And it can be devide into 3 cases. (x, y, and z directions)
        switch (direction) {
        //x direction
        case 1:
            //decide the new point should add to x- or to x+
            //of current wire segment
            if (startX != x && startY == y && startZ == z) {
                if (startX > x) {
                    startX = x;
                    return true;
                } else if (endX < x) {
                    endX = x;
                    return true;
                }
                return false;
            }
            return false;
            break;
            //y direction
        case 2:
            //decide the new point should add to y- or to y+
            //of current wire segment
            if (startX == x && startY != y && startZ == z) {
                if (startY > y) {
                    startY = y;
                    return true;
                } else if (endY < y) {
                    endY = y;
                    return true;
                }
                return false;
            }
            return false;
            break;
            //z direction
        case 3:
            //decide the new point should add to z- or to z+
            //of current wire segment
            if (startX == x && startY == y && startZ != z) {
                if (startZ > z) {
                    startZ = z;
                    return true;
                } else if (endZ < z) {
                    endZ = z;
                    return true;
                }
                return false;
            }
            return false;
            break;
        }
    }

    //wire at wirePos is not exist
    return false;
}

void WireSegments::remove(int wirePos) {
    vector<WireSegment>::iterator ite = segments.begin();
    for (int i = 0; i < wirePos; ++i) {
        ite++;
    }
    this->segments.erase(ite);
}

int WireSegments::size() {
    return this->segments.size();
}
int WireSegments::get_segmentNumber() {
    return this->segments.size();
}

int WireSegments::get_segmentStartX(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].startX;
    }

    return -1;
}

int WireSegments::get_segmentStartY(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].startY;
    }

    return -1;
}

int WireSegments::get_segmentStartZ(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].startZ;
    }

    return -1;
}

int WireSegments::get_segmentEndX(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].endX;
    }

    return -1;
}

int WireSegments::get_segmentEndY(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].endY;
    }

    return -1;
}

int WireSegments::get_segmentEndZ(int wirePos) {
    if (wirePos < (int) this->segments.size()) {
        return this->segments[wirePos].endZ;
    }

    return -1;
}
