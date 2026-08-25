#include <iostream>

#include "instance/competition/business_engine.hpp"
#include "instance/competition/business_replay.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: doogle_replay TRACE_FILE\n";
        return 2;
    }
    doogle::competition::BusinessReplay replay{argv[1]};
    if (!replay.good()) {
        std::cerr << "trace_open_failed\n";
        return 3;
    }
    doogle::competition::BusinessEngine engine;
    while (auto item = replay.next()) {
        const auto decision = engine.tick(item->first, item->second);
        doogle::competition::write_business_decision_json(std::cout, decision);
        std::cout << '\n';
    }
    return 0;
}
