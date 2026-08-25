cmake_minimum_required(VERSION 3.16)
project(doogle LANGUAGES CXX)

set(DOOGLE_PROJECT_DISPLAY_NAME "Doogle比赛代码")
set(DOOGLE_PROJECT_DESCRIPTION "Doogle project")
set(DOOGLE_IMPLEMENTATION_LANGUAGE "Native C++")

include(CTest)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc imgcodecs dnn)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(ZLIB REQUIRED)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Service: stateless domain types, algorithms, protocols and infrastructure.
add_library(doogle_service_domain INTERFACE)
target_include_directories(doogle_service_domain INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(doogle_service_domain INTERFACE cxx_std_20)

add_library(doogle_service_control STATIC
    service/control/motion.cpp
    service/control/pid.cpp
)
target_link_libraries(doogle_service_control PUBLIC doogle_service_domain)

add_library(doogle_service_protocol STATIC
    service/protocol/audio_codec.cpp
    service/protocol/image_packet.cpp
    service/protocol/lcm/command_codec.cpp
    service/protocol/pose_protocol.cpp
    service/protocol/sensor_packets.cpp
)
target_include_directories(doogle_service_protocol PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(doogle_service_protocol PUBLIC
    doogle_service_domain ${OpenCV_LIBS} ZLIB::ZLIB
)

add_library(doogle_service_perception STATIC
    service/perception/geometry.cpp
    service/perception/vision.cpp
    service/perception/depth_tof.cpp
)
target_include_directories(doogle_service_perception PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(doogle_service_perception PUBLIC doogle_service_domain ${OpenCV_LIBS})

add_library(doogle_service_adapter STATIC service/adapter/udp_motion_sink.cpp)
target_link_libraries(doogle_service_adapter
    PUBLIC doogle_service_protocol doogle_service_domain
)

# Instance: state, lifecycle and competition orchestration.
add_library(doogle_instance_competition_core STATIC
    instance/competition/business_engine.cpp
    instance/competition/business_replay.cpp
    instance/competition/stage1_business.cpp
    instance/competition/stage2_business.cpp
    instance/competition/stage3_business.cpp
    instance/competition/stage4_business.cpp
    instance/competition/stage5_business.cpp
    instance/competition/stage6_business.cpp
    instance/competition/reducer.cpp
    instance/competition/stages.cpp
)
target_link_libraries(doogle_instance_competition_core
    PUBLIC doogle_service_domain doogle_service_control doogle_service_perception
)

add_library(doogle_instance_runtime STATIC
    instance/runtime/command_slot.cpp
    instance/runtime/depth_assembler.cpp
    instance/runtime/heartbeat.cpp
    instance/runtime/motion_relay.cpp
    instance/runtime/udp_command_receiver.cpp
    instance/runtime/udp_sensor_hub.cpp
    instance/runtime/zmq_frame_receiver.cpp
)
target_include_directories(doogle_instance_runtime PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(doogle_instance_runtime
    PUBLIC doogle_service_domain doogle_service_control doogle_service_protocol
    PRIVATE ${ZMQ_LIBRARIES}
)

add_library(doogle_instance STATIC instance/competition/stage_app.cpp)
target_link_libraries(doogle_instance
    PUBLIC doogle_instance_competition_core doogle_instance_runtime doogle_service_adapter
)

# External entry points are all owned by Instance.
add_executable(doogle_relay instance/app/doogle_relay.cpp)
target_link_libraries(doogle_relay PRIVATE doogle_instance_runtime doogle_service_adapter)

add_executable(doogle_runner instance/app/doogle_runner.cpp)
target_link_libraries(doogle_runner PRIVATE doogle_instance)

add_executable(doogle_replay instance/app/doogle_replay.cpp)
target_link_libraries(doogle_replay PRIVATE doogle_instance)

foreach(stage RANGE 1 6)
    add_executable(doogle_stage${stage} instance/app/doogle_stage${stage}.cpp)
    target_link_libraries(doogle_stage${stage} PRIVATE doogle_instance)
endforeach()

if(BUILD_TESTING)
    add_executable(doogle_unit_tests instance/test/core_tests.cpp)
    target_link_libraries(doogle_unit_tests PRIVATE
        doogle_instance
        doogle_service_protocol
        doogle_service_perception
    )
    add_test(NAME doogle_unit_tests COMMAND doogle_unit_tests)

    add_executable(doogle_business_tests instance/test/business_tests.cpp)
    target_link_libraries(doogle_business_tests PRIVATE
        doogle_instance_competition_core
        doogle_service_perception
    )
    add_test(NAME doogle_business_tests COMMAND doogle_business_tests)

    add_executable(doogle_transport_tests instance/test/transport_tests.cpp)
    target_link_libraries(doogle_transport_tests PRIVATE
        doogle_instance_runtime
        doogle_service_protocol
    )
    add_test(NAME doogle_transport_tests COMMAND doogle_transport_tests)
endif()
