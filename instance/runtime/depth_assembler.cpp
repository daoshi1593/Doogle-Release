#include "instance/runtime/depth_assembler.hpp"

#include <zlib.h>

#include <algorithm>
#include <cstring>

namespace doogle::runtime {

DepthAssembler::DepthAssembler(std::size_t max_inflight)
    : max_inflight_(std::max<std::size_t>(1, max_inflight)) {}

void DepthAssembler::clear() {
    assemblies_.clear();
    insertion_order_.clear();
}

std::optional<DepthFrame> DepthAssembler::accept(const protocol::DepthFragment& fragment) {
    auto found = assemblies_.find(fragment.sequence);
    if (found == assemblies_.end()) {
        while (assemblies_.size() >= max_inflight_ && !insertion_order_.empty()) {
            assemblies_.erase(insertion_order_.front());
            insertion_order_.erase(insertion_order_.begin());
            ++discarded_frames_;
        }
        Assembly value;
        value.width = fragment.width;
        value.height = fragment.height;
        value.unit_micrometers = fragment.unit_micrometers;
        value.fragment_count = fragment.fragment_count;
        value.capture_stamp_ns = fragment.capture_stamp_ns;
        value.compressed = fragment.compressed;
        value.payload.resize(fragment.frame_bytes);
        value.received.resize(fragment.fragment_count);
        value.ranges.resize(fragment.fragment_count);
        found = assemblies_.emplace(fragment.sequence, std::move(value)).first;
        insertion_order_.push_back(fragment.sequence);
    }
    auto& assembly = found->second;
    if (assembly.width != fragment.width || assembly.height != fragment.height ||
        assembly.fragment_count != fragment.fragment_count || assembly.compressed != fragment.compressed ||
        assembly.payload.size() != fragment.frame_bytes ||
        fragment.offset + fragment.payload.size() > assembly.payload.size()) {
        assemblies_.erase(found);
        insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fragment.sequence),
                               insertion_order_.end());
        ++discarded_frames_;
        return std::nullopt;
    }
    if (!assembly.received[fragment.fragment_index]) {
        std::copy(fragment.payload.begin(), fragment.payload.end(),
                  assembly.payload.begin() + fragment.offset);
        assembly.received[fragment.fragment_index] = true;
        assembly.ranges[fragment.fragment_index] = {
            fragment.offset, fragment.offset + static_cast<std::uint32_t>(fragment.payload.size())};
        ++assembly.received_count;
    }
    if (assembly.received_count != assembly.fragment_count) return std::nullopt;

    auto ranges = assembly.ranges;
    std::sort(ranges.begin(), ranges.end());
    std::uint32_t covered = 0;
    for (const auto& [begin, end] : ranges) {
        if (begin != covered || end < begin) {
            ++discarded_frames_;
            assemblies_.erase(fragment.sequence);
            insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fragment.sequence),
                                   insertion_order_.end());
            return std::nullopt;
        }
        covered = end;
    }
    if (covered != assembly.payload.size()) return std::nullopt;

    const std::size_t raw_size = static_cast<std::size_t>(assembly.width) * assembly.height * sizeof(std::uint16_t);
    std::vector<std::uint8_t> raw(raw_size);
    if (assembly.compressed) {
        uLongf output_size = raw.size();
        if (::uncompress(raw.data(), &output_size, assembly.payload.data(), assembly.payload.size()) != Z_OK ||
            output_size != raw.size()) {
            ++discarded_frames_;
            assemblies_.erase(fragment.sequence);
            insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fragment.sequence),
                                   insertion_order_.end());
            return std::nullopt;
        }
    } else {
        if (assembly.payload.size() != raw.size()) {
            ++discarded_frames_;
            assemblies_.erase(fragment.sequence);
            insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fragment.sequence),
                                   insertion_order_.end());
            return std::nullopt;
        }
        raw = assembly.payload;
    }
    cv::Mat depth(assembly.height, assembly.width, CV_16UC1, raw.data());
    DepthFrame result{fragment.sequence, assembly.capture_stamp_ns, assembly.unit_micrometers, depth.clone()};
    assemblies_.erase(fragment.sequence);
    insertion_order_.erase(std::remove(insertion_order_.begin(), insertion_order_.end(), fragment.sequence),
                           insertion_order_.end());
    return result;
}

}  // namespace doogle::runtime
