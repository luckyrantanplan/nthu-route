#include <sys/time.h>
#include <cstdio>
#include <ctime>
#include <iostream>

#include "../grdb/parser.h"
#include "../grdb/RoutingRegion.h"
#include "../misc/filehandler.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "Layerassignment.h"
#include "parameter.h"

#define SPDLOG_TRACE_ON
#include "../spdlog/spdlog.h"

int main(int argc, char* argv[]) {

    auto console_sp = spdlog::stdout_logger_mt("NTHUR");
    spdlog::logger& log = *console_sp;
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("%Y%m%d%H%M%S.%f] %L %v");
    SPDLOG_TRACE(console_sp, "test the trace");

    log.info("======================================================="); //
    log.info("= NTHU-Route                                          =");
    log.info("= Version 3.0 is developed by                         =");
    log.info("= Yen-Jung Chang, Yu-ting Lee, Tsung-Hsien Lee        =");
    log.info("= Jhih-Rong Gao, Pei-Ci Wu, Florian Prud'homme        =");
    log.info("= Adviser: Ting-Chi Wang (tcwang@cs.nthu.edu.tw)      =");
    log.info("= http://www.cs.nthu.edu.tw/~tcwang/nthuroute/        =");
    log.info("=======================================================\n");
    log.info("=======================================================");
    log.info("= Running FLUTE for initial Steiner tree              =");
    log.info("= FLUTE is developed by Dr. Chris C. N. Chu           =");
    log.info("=                       Iowa State University         =");
    log.info("= http://home.eng.iastate.edu/~cnchu/                 =");
    log.info("=======================================================");
    log.info("= Using Boost Library                                 =");
    log.info("= For internal data structure                         =");
    log.info("=======================================================\n");

    auto t0 = std::chrono::system_clock::now();
    NTHUR::ParameterAnalyzer ap(argc, argv);

    NTHUR::RoutingRegion routingData;

    ap.dataPreparation(routingData);

    NTHUR::Congestion congestion(routingData.get_gridx(), routingData.get_gridy());

    auto t1 = std::chrono::system_clock::now();
    NTHUR::Construct_2d_tree tree(ap.routing_param(), ap.parameter(), routingData, congestion);
    auto t2 = std::chrono::system_clock::now();
    // now the post processing is handle by Construct_2d_tree

    std::chrono::duration<double> duration12 = t2 - t1;

    log.info(" time: {}", duration12.count());

    if (ap.caseType() == 0) {
        //IBM Cases
    } else {
        //ISPD'07 Cases
        NTHUR::Layer_assignment(congestion, tree.rr_map, ap.output());
        auto t4 = std::chrono::system_clock::now();
        std::chrono::duration<double> duration42 = t4 - t2;
        std::chrono::duration<double> duration40 = t4 - t0;
        log.info(" time: {} {}", duration42.count(), duration40.count());
    }

    return 0;
}

