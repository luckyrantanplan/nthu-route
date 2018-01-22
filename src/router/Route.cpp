#include "Route.h"

#include <memory>

#include "../grdb/RoutingRegion.h"
#include "../spdlog/common.h"

#include "../spdlog/details/spdlog_impl.h"
#include "../spdlog/logger.h"
#include "Congestion.h"
#include "Construct_2d_tree.h"
#include "Layerassignment.h"
#include "parameter.h"

#define SPDLOG_TRACE_ON
#include "../spdlog/spdlog.h"

NTHUR::OutputGeneration NTHUR::Route::process(const RoutingRegion& rr, const spdlog::level::level_enum& level) {

    auto console_sp = spdlog::get("NTHUR");
    if (console_sp == nullptr) {

        std::lock_guard<std::mutex> lock(g_mutex);
        console_sp = spdlog::get("NTHUR");
        if (console_sp == nullptr) {
            console_sp = spdlog::stdout_logger_mt("NTHUR");
        }

    }
    spdlog::set_level(level);

    Congestion congestion(rr.get_gridx(), rr.get_gridy());
    RoutingParameters routingparam; // default settings
    routingparam.set_overflow_threshold(0);
    Construct_2d_tree tree(routingparam, rr, congestion);

    OutputGeneration output(rr);
    Layer_assignment layerAssignement(congestion, output);

    return output;
}

