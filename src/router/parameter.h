#ifndef INC_PARAMETER_H
#define INC_PARAMETER_H

#include <memory>
#include <string>

namespace spdlog {
class logger;
} /* namespace spdlog */

namespace NTHUR {
class RoutingRegion;
enum {
    MIN_MAX_COST, MIN_TOTAL_COST
};

enum {
    MAZE_ROUTING_FASTROUTE_COST, MAZE_ROUTING_OVERFLOW_COST, MAZE_ROUTING_CONGESTION_COST, MAZE_ROUTING_MADEOF_COST
};

enum {
    WINDOW_MODE1, WINDOW_MODE2, WINDOW_MODE3, WINDOW_MODE4, WINDOW_MODE5, WINDOW_MODE6, WINDOW_MODE7
};

enum {
    O_EDGE_NUM, T_OVERFLOW, M_OVERFLOW, B_SIZE
};

enum {
    INC, DEC
};

class ParameterSet {
public:
    //Construct_2d.cpp
    int pattern_route_obj;	//0:MIN_MAX_COST, 1:MIN_TOTAL_COST
    int pattern_route_cost;	//0:FASTROUTE_COST, 1:OVERFLOW_COST, 2:CONGESTION_COST

    int iter_2d;
    //Maze_routing_2d.cpp
    int maze_route_cost;	//0:MAZE_ROUTING_FASTROUTE_COST,
                            //1:MAZE_ROUTING_OVERFLOW_COST,
                            //2:MAZE_ROUTING_CONGESTION_COST
    int maze_size_mode;

    //Post_processing.cpp
    int maze_route_list_cost;	//O_EDGE_NUM, T_OVERFLOW, M_OVERFLOW, B_SIZE
    int maze_route_list_order;	//FRONT, BACK
    /* TroyLee: Parameter from command-line */
    int overflow_threshold;
    int iter_p3;
    std::string inputFileName;
    std::string outputFileName;
    std::shared_ptr<spdlog::logger> log_sp;
public:
    ParameterSet(int pattern_route_obj = 0, //
            int pattern_route_cost = 0, //
            int iter_2d = 0, //
            int maze_route_cost = 0, //
            int maze_size_mode = 0, //
            int maze_route_list_cost = 0, //
            int maze_route_list_order = 0, //
            int _overflow_threshold = 10, //
            int _iter_p3 = 10);

    void setSet();

    void setInputfile(std::string input);
    void setOutputfile(std::string output);

    void push_parameter(int pattern_route_obj = 0, int pattern_route_cost = 0,  //
            int iter_2d = 0, int maze_route_cost = 0, int maze_size_mode = 0, int maze_route_list_cost = 0, //
            int maze_route_list_order = 0, int _overflow_threshold = 0, int _iter_p3 = 10);
};

/* TroyLee: RouterParameter Fetch From Command-line */
/* Presetting Parameter Table */
struct PresettingParameters {
    /* Identifier */
    int net_to_route;
    int grid_x, grid_y;
    /* Common Setting */
    bool monotonic_routing_en;
    /* Part 2 Setting */
    int iteration_p2;
    int init_box_size_p2;
    int box_size_inc_p2;
    int overflow_threshold;
    /* Part 3 Setting */
    int iteration_p3;
    int init_box_size_p3;
    int box_size_inc_p3;
    /* Cost Function Selection */
    void (*cost_fp)(int i, int j, int dir);
};

class RoutingParameters {
public:
    /* Init Parameters */
    RoutingParameters();

    void operator=(const struct PresettingParameters &preset);

    /* Setting Parameter */
    void set_monotonic_en(bool en);
    void set_simple_mode_en(bool en);
    void set_iteration_p2(int it);
    void set_init_box_size_p2(int size);
    void set_box_size_inc_p2(int inc);
    void set_overflow_threshold(int th);

    void set_iteration_p3(int it);
    void set_init_box_size_p3(int size);
    void set_box_size_inc_p3(int inc);

    /* Fetching Parameter */
    bool get_monotonic_en() const;
    bool get_simple_mode_en() const;
    int get_iteration_p2() const;
    int get_init_box_size_p2() const;
    int get_box_size_inc_p2() const;
    int get_overflow_threshold() const;

    int get_iteration_p3() const;
    int get_init_box_size_p3() const;
    int get_box_size_inc_p3() const;

private:
    /* Common Setting */
    bool monotonic_routing_en;
    bool simple_mode_en;

    /* Part 2 Setting */
    int iteration_p2;
    int init_box_size_p2;
    int box_size_inc_p2;
    int overflow_threshold;

    /* Part 3 Setting */
    int iteration_p3;
    int init_box_size_p3;
    int box_size_inc_p3;
public:
    int BOXSIZE_INC;
};

//{{{ *RoutingParameters* inline functions
inline void RoutingParameters::operator=(const struct PresettingParameters &preset) {
    monotonic_routing_en = preset.monotonic_routing_en;
    iteration_p2 = preset.iteration_p2;
    init_box_size_p2 = preset.init_box_size_p2;
    box_size_inc_p2 = preset.box_size_inc_p2;
    overflow_threshold = preset.overflow_threshold;
    iteration_p3 = preset.iteration_p3;
    init_box_size_p3 = preset.init_box_size_p3;
    box_size_inc_p3 = preset.box_size_inc_p3;
}

/* Setting Parameter */
inline
void RoutingParameters::set_monotonic_en(bool en) {
    monotonic_routing_en = en;
}

inline
void RoutingParameters::set_simple_mode_en(bool en) {
    simple_mode_en = en;
}

inline
void RoutingParameters::set_iteration_p2(int it) {
    iteration_p2 = it;
}

inline
void RoutingParameters::set_init_box_size_p2(int size) {
    init_box_size_p2 = size;
}

inline
void RoutingParameters::set_box_size_inc_p2(int inc) {
    box_size_inc_p2 = inc;
}

inline
void RoutingParameters::set_overflow_threshold(int th) {
    overflow_threshold = th;
}

inline
void RoutingParameters::set_iteration_p3(int it) {
    iteration_p3 = it;
}

inline
void RoutingParameters::set_init_box_size_p3(int size) {
    init_box_size_p3 = size;
}

inline
void RoutingParameters::set_box_size_inc_p3(int inc) {
    box_size_inc_p3 = inc;
}

/* Fetching Parameter */
inline
bool RoutingParameters::get_monotonic_en() const {
    return this->monotonic_routing_en;
}

inline
bool RoutingParameters::get_simple_mode_en() const {
    return this->simple_mode_en;
}

inline
int RoutingParameters::get_iteration_p2() const {
    return this->iteration_p2;
}

inline
int RoutingParameters::get_init_box_size_p2() const {
    return this->init_box_size_p2;
}

inline
int RoutingParameters::get_box_size_inc_p2() const {
    return this->box_size_inc_p2;
}

inline
int RoutingParameters::get_overflow_threshold() const {
    return this->overflow_threshold;
}

inline
int RoutingParameters::get_iteration_p3() const {
    return this->iteration_p3;
}

inline
int RoutingParameters::get_init_box_size_p3() const {
    return this->init_box_size_p3;
}

inline
int RoutingParameters::get_box_size_inc_p3() const {
    return this->box_size_inc_p3;
}
//}}}
/* TroyLee: RouterParameter Fetch From Command-line */

class ParameterAnalyzer {
public:
    ParameterAnalyzer(int argc, char* argv[]);
    //return input file name
    const char* input();
    //return output file name
    const std::string& output();
    //return parameter group NO.
    ParameterSet& parameter();
    //return the input type: 0 for IBM test cases  and 1 for the others
    int caseType();
    RoutingParameters& routing_param();
    RoutingRegion dataPreparation();
private:
    int argc;
    char** argv;
    std::string inputFileName;
    std::string outputFileName;
    int paraNO; //parameter group NO.
    int type;   //0 for IBM test cases  and 1 for the others
    ParameterSet parameterSet;
    RoutingParameters routingParam;

private:
    void analyze2();  //begin to analyze parameters
    void analyzeInput();
    void analyzeOutput();
};

inline RoutingParameters&
ParameterAnalyzer::routing_param() {
    return routingParam;
}

} // namespace NTHUR

#endif //INC_PARAMETER_H
