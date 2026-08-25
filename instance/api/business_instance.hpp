#pragma once

#include <string_view>

#include "instance/api/business_contract.hpp"

namespace doogle::instance {

class IBusinessInstance {
public:
    virtual ~IBusinessInstance() = default;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    [[nodiscard]] virtual competition::BusinessDecision tick(
        const competition::SensorFrame& frame) = 0;
    virtual void reset() = 0;
};

}  // namespace doogle::instance
