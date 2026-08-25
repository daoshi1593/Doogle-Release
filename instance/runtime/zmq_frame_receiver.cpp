#include "instance/runtime/zmq_frame_receiver.hpp"

#include <zmq.h>

#include <vector>

namespace doogle::runtime {

ZmqFrameReceiver::ZmqFrameReceiver(std::string endpoint) : endpoint_(std::move(endpoint)) {}
ZmqFrameReceiver::~ZmqFrameReceiver() { close(); }

bool ZmqFrameReceiver::open() {
    close();
    context_ = zmq_ctx_new();
    if (!context_) return false;
    socket_ = zmq_socket(context_, ZMQ_SUB);
    if (!socket_) {
        close();
        return false;
    }
    int high_water = 1;
    int linger = 0;
    zmq_setsockopt(socket_, ZMQ_RCVHWM, &high_water, sizeof(high_water));
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket_, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(socket_, endpoint_.c_str()) != 0) {
        close();
        return false;
    }
    window_start_ = std::chrono::steady_clock::now();
    return true;
}

void ZmqFrameReceiver::close() {
    if (socket_) {
        zmq_close(socket_);
        socket_ = nullptr;
    }
    if (context_) {
        zmq_ctx_term(context_);
        context_ = nullptr;
    }
}

std::optional<protocol::DecodedImage> ZmqFrameReceiver::receive(std::chrono::milliseconds timeout) {
    if (!socket_) return std::nullopt;
    const int milliseconds = static_cast<int>(timeout.count());
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &milliseconds, sizeof(milliseconds));
    zmq_msg_t header_message;
    zmq_msg_init(&header_message);
    const int header_size = zmq_msg_recv(&header_message, socket_, 0);
    if (header_size < 0) {
        zmq_msg_close(&header_message);
        return std::nullopt;
    }
    std::string header(static_cast<const char*>(zmq_msg_data(&header_message)),
                       static_cast<std::size_t>(header_size));
    const bool more = zmq_msg_more(&header_message) != 0;
    zmq_msg_close(&header_message);
    if (!more) {
        ++decode_errors_;
        return std::nullopt;
    }
    zmq_msg_t payload_message;
    zmq_msg_init(&payload_message);
    const int payload_size = zmq_msg_recv(&payload_message, socket_, 0);
    if (payload_size < 0) {
        zmq_msg_close(&payload_message);
        ++decode_errors_;
        return std::nullopt;
    }
    const auto* begin = static_cast<const std::uint8_t*>(zmq_msg_data(&payload_message));
    std::vector<std::uint8_t> payload(begin, begin + payload_size);
    const bool unexpected_more = zmq_msg_more(&payload_message) != 0;
    zmq_msg_close(&payload_message);
    if (unexpected_more) {
        zmq_msg_t extra;
        do {
            zmq_msg_init(&extra);
            if (zmq_msg_recv(&extra, socket_, 0) < 0) {
                zmq_msg_close(&extra);
                break;
            }
            const bool more_parts = zmq_msg_more(&extra) != 0;
            zmq_msg_close(&extra);
            if (!more_parts) break;
        } while (true);
        ++decode_errors_;
        return std::nullopt;
    }
    std::string error;
    auto decoded = protocol::decode_image_packet(header, payload, &error);
    if (!decoded) {
        ++decode_errors_;
        return std::nullopt;
    }
    const auto sequence = decoded->header.sequence;
    if (last_sequence_ != 0 && sequence > last_sequence_ + 1) dropped_sequence_ += sequence - last_sequence_ - 1;
    last_sequence_ = sequence;
    ++received_total_;
    return decoded;
}

FrameReceiverHealth ZmqFrameReceiver::health() const {
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - window_start_).count();
    return {endpoint_, received_total_, decode_errors_, dropped_sequence_, last_sequence_,
            seconds > 0.0 ? static_cast<double>(received_total_) / seconds : 0.0};
}

}  // namespace doogle::runtime
