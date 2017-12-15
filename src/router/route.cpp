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

void dataPreparation(ParameterAnalyzer& ap, RoutingRegion& builder);

int main(int argc, char* argv[]) {
    std::cout << "=======================================================" << std::endl //
            << "= NTHU-Route                                          =" << std::endl //
            << "= Version 2.0 is deveploped by                        =" << std::endl //
            << "= Yen-Jung Chang, Yu-ting Lee, Tsung-Hsien Lee        =" << std::endl //
            << "= Jhih-Rong Gao, Pei-Ci Wu                            =" << std::endl //
            << "= Adviser: Ting-Chi Wang (tcwang@cs.nthu.edu.tw)      =" << std::endl //
            << "= http://www.cs.nthu.edu.tw/~tcwang/nthuroute/        =" << std::endl //
            << "=======================================================" << std::endl << std::endl //
            << "=======================================================" << std::endl //
            << "= Running FLUTE for initial steiner tree              =" << std::endl //
            << "= FLUTE is developed by Dr. Chris C. N. Chu           =" << std::endl //
            << "=                       Iowa State University         =" << std::endl //
            << "= http://home.eng.iastate.edu/~cnchu/                 =" << std::endl //
            << "=======================================================" << std::endl //
            << "= Using Google Sparse Hash Map                        =" << std::endl //
            << "= For internal data structure                         =" << std::endl //
            << "= http://code.google.com/p/google-sparsehash/         =" << std::endl //
            << "=======================================================" << std::endl;

    clock_t t0 = clock();
    ParameterAnalyzer ap(argc, argv);

    RoutingRegion routingData;

    dataPreparation(ap, routingData);

    Congestion congestion(routingData.get_gridx(), routingData.get_gridy());

    clock_t t1 = clock();
    Construct_2d_tree tree(ap.routing_param(), ap.parameter(), routingData);
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
