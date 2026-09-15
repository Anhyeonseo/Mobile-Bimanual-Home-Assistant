cmake_minimum_required(VERSION 3.20)
project(stm32_actuator_core LANGUAGES C)

add_library(actuator_core STATIC
    src/shared_bus.c
    src/bus_schedule.c
    src/lift_endpoint.c
    src/device_startup.c
    src/mobile_feedback.c
    src/sts3215_response.c
    src/mobile_output.c
    src/mobile_supervisor.c
    src/mobile_wire.c
    src/mobile_endpoint.c
    src/mobile_framed.c
    src/lift_controller.c
    src/bus_router.c
    src/system_stop.c
    src/motor_groups.c
    src/bimanual_dispatch.c
    src/bimanual_goal_map.c
    src/buffered_command_route.c
    src/buffered_executor.c
    src/calibration.c
    src/cobs.c
    src/crc32c.c
    src/joint_unwrap.c
    src/protocol.c
    src/safety.c
    src/stream_contract_v2.c
    src/stream_executor_v2.c
    src/stream_session_v2.c
    src/sts3215_packet.c
    src/setpoint_queue.c
)
target_include_directories(actuator_core PUBLIC include)
target_compile_features(actuator_core PUBLIC c_std_11)

if(MSVC)
    target_compile_options(actuator_core PRIVATE /W4 /WX)
else()
    target_compile_options(actuator_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# Optional PC shared library for cross-language end-to-end tests; no HAL.
option(BUILD_PC_MOBILE_SIMULATOR "Build the deterministic native mobile simulator" ON)
if(BUILD_PC_MOBILE_SIMULATOR AND NOT CMAKE_CROSSCOMPILING)
    set_target_properties(actuator_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
    add_library(actuator_mobile_simulator SHARED tests/pc_mobile_simulator.c)
    target_link_libraries(actuator_mobile_simulator PRIVATE actuator_core)
    add_library(actuator_host_simulator SHARED tests/pc_mobile_simulator.c
        src/protocol.c src/cobs.c src/crc32c.c)
    target_compile_definitions(actuator_host_simulator PRIVATE ACTUATOR_PROTOCOL_VERSION=2)
    target_link_libraries(actuator_host_simulator PRIVATE actuator_core)
    target_compile_options(actuator_host_simulator PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(actuator_device_startup_tests tests/test_device_startup.c)
    target_link_libraries(actuator_device_startup_tests PRIVATE actuator_core)
    target_compile_options(actuator_device_startup_tests PRIVATE $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME actuator_device_startup_tests COMMAND actuator_device_startup_tests)
    add_executable(actuator_mobile_feedback_tests tests/test_mobile_feedback.c)
    target_link_libraries(actuator_mobile_feedback_tests PRIVATE actuator_core)
    target_compile_options(actuator_mobile_feedback_tests PRIVATE $<$<NOT:$<C_COMPILER_ID:MSVC>>:-Wall;-Wextra;-Wpedantic;-Werror>)
    add_test(NAME actuator_mobile_feedback_tests COMMAND actuator_mobile_feedback_tests)
    add_executable(actuator_mobile_output_tests tests/test_mobile_output.c)
    target_link_libraries(actuator_mobile_output_tests PRIVATE actuator_core)
    add_test(NAME actuator_mobile_output_tests COMMAND actuator_mobile_output_tests)
    add_executable(actuator_bus_schedule_tests tests/test_bus_schedule.c)
    target_link_libraries(actuator_bus_schedule_tests PRIVATE actuator_core)
    add_test(NAME actuator_bus_schedule_tests COMMAND actuator_bus_schedule_tests)
    add_executable(actuator_mobile_stack_tests tests/test_mobile_stack.c)
    target_link_libraries(actuator_mobile_stack_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_mobile_stack_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_mobile_stack_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_mobile_stack_tests COMMAND actuator_mobile_stack_tests)
    add_executable(actuator_mobile_wire_tests tests/test_mobile_wire.c)
    target_link_libraries(actuator_mobile_wire_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_mobile_wire_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_mobile_wire_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_mobile_wire_tests COMMAND actuator_mobile_wire_tests)
    add_executable(actuator_mobile_foundation_tests tests/test_mobile_foundation.c)
    target_link_libraries(actuator_mobile_foundation_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_mobile_foundation_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_mobile_foundation_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_mobile_foundation_tests COMMAND actuator_mobile_foundation_tests)
    add_executable(actuator_core_tests tests/test_main.c)
    target_link_libraries(actuator_core_tests PRIVATE actuator_core)
    add_executable(
        actuator_buffered_command_route_tests
        tests/test_buffered_command_route.c
    )
    target_link_libraries(actuator_buffered_command_route_tests PRIVATE actuator_core)
    add_executable(
        actuator_joint_unwrap_tests
        tests/test_joint_unwrap.c
    )
    target_link_libraries(actuator_joint_unwrap_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_contract_v2_tests
        tests/test_stream_contract_v2.c
    )
    target_link_libraries(actuator_stream_contract_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_executor_v2_tests
        tests/test_stream_executor_v2.c
    )
    target_link_libraries(actuator_stream_executor_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_stream_session_v2_tests
        tests/test_stream_session_v2.c
    )
    target_link_libraries(actuator_stream_session_v2_tests PRIVATE actuator_core)
    add_executable(
        actuator_sts3215_packet_tests
        tests/test_sts3215_packet.c
    )
    target_link_libraries(actuator_sts3215_packet_tests PRIVATE actuator_core)
    add_executable(
        actuator_bimanual_dispatch_tests
        tests/test_bimanual_dispatch.c
    )
    target_link_libraries(actuator_bimanual_dispatch_tests PRIVATE actuator_core)
    add_executable(
        actuator_bimanual_goal_map_tests
        tests/test_bimanual_goal_map.c
    )
    target_link_libraries(actuator_bimanual_goal_map_tests PRIVATE actuator_core)
    if(MSVC)
        target_compile_options(actuator_core_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_buffered_command_route_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_joint_unwrap_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_contract_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_executor_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_stream_session_v2_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_sts3215_packet_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_bimanual_dispatch_tests PRIVATE /W4 /WX)
        target_compile_options(actuator_bimanual_goal_map_tests PRIVATE /W4 /WX)
    else()
        target_compile_options(actuator_core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_buffered_command_route_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_joint_unwrap_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_contract_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_executor_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_stream_session_v2_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_sts3215_packet_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_bimanual_dispatch_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
        target_compile_options(actuator_bimanual_goal_map_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME actuator_core_tests COMMAND actuator_core_tests)
    add_test(NAME actuator_buffered_command_route_tests COMMAND actuator_buffered_command_route_tests)
    add_test(NAME actuator_joint_unwrap_tests COMMAND actuator_joint_unwrap_tests)
    add_test(NAME actuator_stream_contract_v2_tests COMMAND actuator_stream_contract_v2_tests)
    add_test(NAME actuator_stream_executor_v2_tests COMMAND actuator_stream_executor_v2_tests)
    add_test(NAME actuator_stream_session_v2_tests COMMAND actuator_stream_session_v2_tests)
    add_test(NAME actuator_sts3215_packet_tests COMMAND actuator_sts3215_packet_tests)
    add_test(NAME actuator_bimanual_dispatch_tests COMMAND actuator_bimanual_dispatch_tests)
    add_test(NAME actuator_bimanual_goal_map_tests COMMAND actuator_bimanual_goal_map_tests)
endif()
