#include "parameter.h"

#include <getopt.h>
#include <cstdlib>
#include <iostream>

#include "../grdb/parser.h"
#include "../grdb/RoutingRegion.h"
#include "../misc/filehandler.h"
#include "../spdlog/details/logger_impl.h"
#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "DataDef.h"

namespace NTHUR {

using namespace std;

//pre-defined files' name
#define TEST_BENCH_DIR	"./"
#define OUTPUT_DIR		"./"
#define DEFAULT_INPUT	"sample/demo"
#define DEFAULT_OUTPUT	"output.txt"
#define A1              "adaptec1.capo70.3d.35.50.90.gr.gz"
#define A2              "adaptec2.mpl60.3d.35.20.100.gr.gz"
#define A3              "adaptec3.dragon70.3d.30.50.90.gr.gz"
#define A4              "adaptec4.aplace60.3d.30.50.90.gr.gz"
#define A5              "adaptec5.mfar50.3d.50.20.100.gr.gz"
#define N1              "newblue1.ntup50.3d.30.50.90.gr.gz"
#define N2              "newblue2.fastplace90.3d.50.20.100.gr.gz"
#define N3              "newblue3.kraftwerk80.3d.40.50.90.gr.gz"
#define N4              "newblue4.mpl50.3d.40.10.95.gr.gz"
#define N5              "newblue5.ntup50.3d.40.10.100.gr.gz"
#define N6              "newblue6.mfar80.3d.60.10.100.gr.gz"
#define N7              "newblue7.kraftwerk70.3d.80.20.82.m8.gr.gz"
#define B1              "bigblue1.capo60.3d.50.10.100.gr.gz"
#define B2              "bigblue2.mpl60.3d.40.60.60.gr.gz"
#define B3              "bigblue3.aplace70.3d.50.10.90.m8.gr.gz"
#define B4              "bigblue4.fastplace70.3d.80.20.80.gr.gz"

#define A12             "adaptec1.capo70.2d.35.50.90.gr.gz"
#define A22             "adaptec2.mpl60.2d.35.20.100.gr.gz"
#define A32             "adaptec3.dragon70.2d.30.50.90.gr.gz"
#define A42             "adaptec4.aplace60.2d.30.50.90.gr.gz"
#define A52             "adaptec5.mfar50.2d.50.20.100.gr.gz"
#define N12             "newblue1.ntup50.2d.30.50.90.gr.gz"
#define N22             "newblue2.fastplace90.2d.50.20.100.gr.gz"
#define N32             "newblue3.kraftwerk80.2d.40.50.90.gr.gz"
#define S1              "sample/demo"
#define ibm01           "ibm01.modified.txt.gz"
#define ibm02           "ibm02.modified.txt.gz"
#define ibm03           "ibm03.modified.txt.gz"
#define ibm04           "ibm04.modified.txt.gz"
#define ibm05           "ibm05.modified.txt.gz"
#define ibm06           "ibm06.modified.txt.gz"
#define ibm07           "ibm07.modified.txt.gz"
#define ibm08           "ibm08.modified.txt.gz"
#define ibm09           "ibm09.modified.txt.gz"
#define ibm10           "ibm10.modified.txt.gz"
//prefix of parameters
#define inputPrefix "in="
#define outputPrefix "out="
#define paraPrefix "par="

inline ParameterSet::ParameterSet(int pattern_route_obj, //
        int pattern_route_cost, //
        int iter_2d, //
        int maze_route_cost, //
        int maze_size_mode, //
        int maze_route_list_cost, //
        int maze_route_list_order, //
        int _overflow_threshold, //
        int _iter_p3) :
        pattern_route_obj(pattern_route_obj), //
        pattern_route_cost(pattern_route_cost), //
        iter_2d(iter_2d), //
        maze_route_cost(maze_route_cost), //
        maze_size_mode(maze_size_mode), //
        maze_route_list_cost(maze_route_list_cost), //
        maze_route_list_order(maze_route_list_order), //
        overflow_threshold(_overflow_threshold), //
        iter_p3(_iter_p3) {
    log_sp = spdlog::get("NTHUR");
}

void ParameterSet::setSet() {
    push_parameter(MIN_TOTAL_COST, FASTROUTE_COST, 200, MAZE_ROUTING_MADEOF_COST, WINDOW_MODE6, T_OVERFLOW, DEC); //2	//n3

    log_sp->info("parameter setting:{} {}   {} {} {} {} {}", pattern_route_obj, pattern_route_cost, iter_2d, maze_route_cost, maze_size_mode, maze_route_list_cost, maze_route_list_order);
}

void ParameterSet::setInputfile(std::string input) {
    inputFileName = input;
}
void ParameterSet::setOutputfile(std::string output) {
    outputFileName = output;
}

void ParameterSet::push_parameter(int a, int b, int d, int e, int f, int g, int h, int _overflow_threshold, int _iter_p3) {
    pattern_route_obj = a;
    pattern_route_cost = b;

    iter_2d = d;
    maze_route_cost = e;
    maze_size_mode = f;
    maze_route_list_cost = g;
    maze_route_list_order = h;
    overflow_threshold = _overflow_threshold;
    iter_p3 = _iter_p3;
}

ParameterAnalyzer::ParameterAnalyzer(int argc, char* argv[]) :
        argc(argc), argv(argv), type(1), routingParam() {
    analyze2();
    cout << "Input file: \"" << inputFileName << "\"" << endl;
    cout << "Output file: \"" << outputFileName << "\"" << endl;
}
void ParameterAnalyzer::analyze2() {
    char cmd;
    bool defineOutput = false;
    paraNO = 8;
    parameterSet.setSet();
    int long_option_index = 0;
    struct option long_option[] = { { "p2-max-iteration", 1, 0, 1 }, { "p3-max-iteration", 1, 0, 2 }, { "overflow-threshold", 1, 0, 3 }, { "p3-init-box-size", 1, 0, 4 }, { "p3-box-expand-size", 1, 0,
            5 }, { "p2-boxsize-inc", 1, 0, 6 }, { "p2-box-expand-size", 1, 0, 7 }, { "monotonic-routing", 1, 0, 8 }, { "simple", 0, 0, 9 }, { "input", 1, 0, 'i' }, { "output", 1, 0, 'o' }, {
            "p2-init-box-size", 1, 0, 6 }, { 0, 0, 0, 0 } };
    while ((cmd = getopt_long(argc, argv, "i:I:o:p:", long_option, &long_option_index)) != -1) {
        string parameter;
        bool enable;
        if (long_option[long_option_index].has_arg != 0) {
            parameter = optarg;
        }
        switch (cmd) {
        case 0:
            cout << "Wrong parameter entered!" << endl;
            break;
        case 1:
            cout << "Part 2 Max Iteration set to " << parameter << endl;
            parameterSet.iter_2d = atoi(parameter.c_str());
            routingParam.set_iteration_p2(atoi(parameter.c_str()));
            break;
        case 2:
            cout << "Part 3 Max Iteration set to " << parameter << endl;
            parameterSet.iter_p3 = atoi(parameter.c_str());
            routingParam.set_iteration_p3(atoi(parameter.c_str()));
            break;
        case 3:
            cout << "P2 to P3 overflow threashold set to " << parameter << endl;
            parameterSet.overflow_threshold = atoi(parameter.c_str());
            routingParam.set_overflow_threshold(atoi(parameter.c_str()));
            break;
        case 4:
            cout << "P3 Initial Box Size set to " << parameter << endl;
            routingParam.set_init_box_size_p3(atoi(parameter.c_str()));
            break;
        case 5:
            cout << "P3 Box Expand Size set to " << parameter << endl;
            routingParam.set_box_size_inc_p3(atoi(parameter.c_str()));

            break;
        case 6:
            cout << "P2 BOXSIZE_INC set to " << parameter << endl;
            routingParam.BOXSIZE_INC = atoi(parameter.c_str());
            routingParam.set_init_box_size_p2(atoi(parameter.c_str()));
            break;
        case 7:
            cout << "P2 Box Expand Size set to " << parameter << endl;
            routingParam.set_box_size_inc_p2(atoi(parameter.c_str()));
            break;
        case 8:
            cout << "Monotonic Routing ";
            enable = atoi(parameter.c_str()) == 1;
            routingParam.set_monotonic_en(enable);
            if (enable)
                cout << "Enabled!" << endl;
            else
                cout << "Disabled!" << endl;
            break;
        case 9:
            cout << "Simple Mode enable - Routing Parameter Auto Fitting!" << endl;
            routingParam.set_simple_mode_en(true);
            break;
        case 'i':
            cout << "Input file " << parameter << endl;
            this->inputFileName.append(parameter);
            this->analyzeInput();
            parameterSet.setInputfile(parameter);
            break;
        case 'I':
            cout << "Input file " << parameter << endl;
            this->inputFileName.append(parameter);
            parameterSet.setInputfile(parameter);
            break;
        case 'o':
            cout << "Output file " << parameter << endl;
            this->outputFileName.append(parameter);
            this->analyzeOutput();
            parameterSet.setOutputfile(parameter);
            defineOutput = true;
            break;
        case '?':
            cout << "Unknown parameter!" << endl;
            break;
        }
    }
    if (defineOutput != true) {
        this->outputFileName.append(this->inputFileName + ".output");
    }
}

void ParameterAnalyzer::analyzeInput() {
    bool usePredefine = false;
    if (this->inputFileName == "a1") {
        this->inputFileName = A1;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a2") {
        this->inputFileName = A2;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a3") {
        this->inputFileName = A3;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a4") {
        this->inputFileName = A4;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a5") {
        this->inputFileName = A5;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n1") {
        this->inputFileName = N1;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n2") {
        this->inputFileName = N2;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n3") {
        this->inputFileName = N3;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n4") {
        this->inputFileName = N4;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n5") {
        this->inputFileName = N5;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n6") {
        this->inputFileName = N6;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n7") {
        this->inputFileName = N7;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "b1") {
        this->inputFileName = B1;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "b2") {
        this->inputFileName = B2;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "b3") {
        this->inputFileName = B3;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "b4") {
        this->inputFileName = B4;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a12") {
        this->inputFileName = A12;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a22") {
        this->inputFileName = A22;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a32") {
        this->inputFileName = A32;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a42") {
        this->inputFileName = A42;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "a52") {
        this->inputFileName = A52;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n12") {
        this->inputFileName = N12;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n22") {
        this->inputFileName = N22;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "n32") {
        this->inputFileName = N32;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "s1") {
        this->inputFileName = S1;
        usePredefine = true;
        this->type = 1;
    } else if (this->inputFileName == "i1") {
        this->inputFileName = ibm01;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i2") {
        this->inputFileName = ibm02;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i3") {
        this->inputFileName = ibm03;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i4") {
        this->inputFileName = ibm04;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i5") {
        this->inputFileName = ibm05;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i6") {
        this->inputFileName = ibm06;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i7") {
        this->inputFileName = ibm07;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i8") {
        this->inputFileName = ibm08;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i9") {
        this->inputFileName = ibm09;
        usePredefine = true;
        this->type = 0;
    } else if (this->inputFileName == "i10") {
        this->inputFileName = ibm10;
        usePredefine = true;
        this->type = 0;
    }

    if (usePredefine == true) {
        this->inputFileName.insert(0, TEST_BENCH_DIR);
    }
}

void ParameterAnalyzer::analyzeOutput() {
}

const char* ParameterAnalyzer::input() {
    return this->inputFileName.c_str();
}

const std::string& ParameterAnalyzer::output() {
    return this->outputFileName;
}

ParameterSet& ParameterAnalyzer::parameter() {
    return parameterSet;
}

int ParameterAnalyzer::caseType() {
    return this->type;
}

RoutingRegion ParameterAnalyzer::dataPreparation() {

    if (caseType() == 0) {
        Parser98 parser = Parser98(input(), FileHandler::AutoFileType);
        return parser.parse();
    }
    Parser07 parser(input(), FileHandler::AutoFileType);
    return parser.parse();

}

RoutingParameters::RoutingParameters() {
    /* Presetting Parameter */
    /* Common Setting */
    monotonic_routing_en = false;
    simple_mode_en = false;

    /* Part 2 Setting */
    iteration_p2 = 150;
    init_box_size_p2 = 10;
    box_size_inc_p2 = 10;
    overflow_threshold = 200;

    /* Part 3 Setting */
    iteration_p3 = 20;
    init_box_size_p3 = 10;
    box_size_inc_p3 = 15;
    BOXSIZE_INC = 10;
}

} // namespace NTHUR
