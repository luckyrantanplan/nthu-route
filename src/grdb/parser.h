// File: grdb/parser.h
// Brief: Parser for parsing test case of ISPD'07 and ISPD'98
// Author: Yen-Jung Chang
// $Date: 2007-11-26 12:48:59 +0800 (Mon, 26 Nov 2007) $
// $Revision: 12 $

#ifndef INC_PARSER_H
#define INC_PARSER_H

#include <string>

#include "../misc/filehandler.h"

namespace NTHUR {

class RoutingRegion;

/** 
 * @brief Global Router Parser for parsing ISPD'07 test case
 */
class Parser07 {
    std::string fname_;      ///< File name
    std::string delims_;
    FileHandler fh_;    ///< File Handler
public:
    Parser07(const std::string& fname, FileHandler::FileType ftype);
    ~Parser07();
    RoutingRegion parse();

private:
    /// Parse information of routing layers, tiles
    RoutingRegion parseRoutingRegion();

    /// Parse information of nets
    void parseNets(RoutingRegion& builder_);

    /// Parse information of one net
    void parseANet(RoutingRegion& builder_);

    /// Parse information of adjustmenting edge capacity
    void adjustCapacity(RoutingRegion& builder_);
};

/**
 * @brief Global Router Parser for parsing ISPD'98 test case
 */
class Parser98 {
    std::string fname_;      ///< File name
    std::string delims_;
    FileHandler fh_;    ///< File Handler

public:
    Parser98(const std::string& fname, FileHandler::FileType ftype);
    ~Parser98();

    RoutingRegion parse();

private:
    /// Parse the information of routing layers, tiles
    RoutingRegion parseRoutingRegion();

    /// Parse the information of nets
    void parseNets(RoutingRegion& builder_);

    /// Parse information of one net
    void parseANet(RoutingRegion& builder_);
};

//======= Inline Functions =======
inline Parser07::Parser07(const std::string& fname, FileHandler::FileType ftype) :
        fname_(fname), delims_(" \t\n"), fh_(fname.data(), ftype) {
}

inline Parser98::Parser98(const std::string& fname, FileHandler::FileType ftype) :
        fname_(fname), delims_(" \t\n"), fh_(fname.data(), ftype) {
}
} // namespace NTHUR

#endif //INC_PARSER_H
