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
#include <spdlog/spdlog.h>

void dataPreparation(ParameterAnalyzer& ap, RoutingRegion& builder);

int main(int argc, char* argv[]) {

    auto console_sp = spdlog::stdout_logger_mt("NTHUR");
    spdlog::logger& log = *console_sp;
    spdlog::set_level(spdlog::level::trace);
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

    clock_t t0 = clock();
    ParameterAnalyzer ap(argc, argv);

    RoutingRegion routingData;

    dataPreparation(ap, routingData);

    Congestion congestion(routingData.get_gridx(), routingData.get_gridy());

    clock_t t1 = clock();
    Construct_2d_tree tree(ap.routing_param(), ap.parameter(), routingData, congestion);
    clock_t t2 = clock();
    // now the post processing is handle by Construct_2d_tree
    clock_t t3 = clock();
    printf("\033[33mtime:\033[m %.2f %.2f %.2f\n", (double) (t2 - t1) / CLOCKS_PER_SEC, (double) (t3 - t2) / CLOCKS_PER_SEC, (double) (t3 - t1) / CLOCKS_PER_SEC);

    if (ap.caseType() == 0) {
        //IBM Cases
    } else {
        //ISPD'07 Cases
        Layer_assignment(congestion, tree.rr_map, ap.output());
        clock_t t4 = clock();
        printf("time: %.2f %.2f\n", (double) (t4 - t3) / CLOCKS_PER_SEC, (double) (t4 - t0) / CLOCKS_PER_SEC);
    }

    return 0;
}

void dataPreparation(ParameterAnalyzer& ap, RoutingRegion& builder) {

    if (ap.caseType() == 0) {
        Parser98 parser = Parser98(ap.input(), FileHandler::AutoFileType, builder);
        parser.parse();
    } else {
        Parser07 parser(ap.input(), FileHandler::AutoFileType, builder);
        parser.parse();
    }

}
