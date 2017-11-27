#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cassert>

#include "parameter.h"
#include "Construct_2d_tree.h"

#include "../grdb/RoutingRegion.h"
#include "../grdb/parser.h"
#include "../misc/filehandler.h"
#include "../util/verifier.h"

void dataPreparetion(ParameterAnalyzer& ap, RoutingRegion& builder);


extern void Post_processing();
extern void Layer_assignment(const char*);

int main(int argc, char* argv[]) {
    std::cout << "=======================================================" << std::endl << "= NTHU-Route                                          =" << std::endl
            << "= Version 2.0 is deveploped by                        =" << std::endl << "= Yen-Jung Chang, Yu-ting Lee, Tsung-Hsien Lee        =" << std::endl
            << "= Jhih-Rong Gao, Pei-Ci Wu                            =" << std::endl << "= Adviser: Ting-Chi Wang (tcwang@cs.nthu.edu.tw)      =" << std::endl
            << "= http://www.cs.nthu.edu.tw/~tcwang/nthuroute/        =" << std::endl << "=======================================================" << std::endl << std::endl
            << "=======================================================" << std::endl << "= Running FLUTE for initial steiner tree              =" << std::endl
            << "= FLUTE is developed by Dr. Chris C. N. Chu           =" << std::endl << "=                       Iowa State University         =" << std::endl
            << "= http://home.eng.iastate.edu/~cnchu/                 =" << std::endl << "=======================================================" << std::endl
            << "= Using Google Sparse Hash Map                        =" << std::endl << "= For internal data structure                         =" << std::endl
            << "= http://code.google.com/p/google-sparsehash/         =" << std::endl << "=======================================================" << std::endl;

    clock_t t0 = clock();
    ParameterAnalyzer ap(argc, argv);

    RoutingRegion routingData;

    dataPreparetion(ap, routingData);



    routing_parameter = ap.routing_param();
    pre_evaluate_congestion_cost_fp = pre_evaluate_congestion_cost_all;

    clock_t t1 = clock();
    Construct_2d_tree(ap.parameter(),routingData);
    clock_t t2 = clock();
    Post_processing();
    clock_t t3 = clock();
    printf("\033[33mtime:\033[m %.2f %.2f %.2f\n", (double) (t2 - t1) / CLOCKS_PER_SEC, (double) (t3 - t2) / CLOCKS_PER_SEC, (double) (t3 - t1) / CLOCKS_PER_SEC);

    if (ap.caseType() == 0) {
        //IBM Cases
    } else {
        //ISPD'07 Cases
        Layer_assignment(ap.output());
        clock_t t4 = clock();
        printf("time: %.2f %.2f\n", (double) (t4 - t3) / CLOCKS_PER_SEC, (double) (t4 - t0) / CLOCKS_PER_SEC);
    }

    return 0;
}

void dataPreparetion(ParameterAnalyzer& ap, RoutingRegion& builder) {

    if (ap.caseType() == 0) {
        Parser98 parser = Parser98(ap.input(), FileHandler::AutoFileType, builder);
        parser.parse();
    } else {
        Parser07 parser(ap.input(), FileHandler::AutoFileType, builder);
        parser.parse();
    }

}
