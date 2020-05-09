package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "control_proto",
    deps = [
        ":control_proto_lib",
    ],
)

proto_library(
    name = "control_proto_lib",
    srcs = [
        "calibration_table.proto",
        "control_cmd.proto",
        "control_conf.proto",
        "gain_scheduler_conf.proto",
        "lat_controller_conf.proto",
        "lon_controller_conf.proto",
        "mpc_controller_conf.proto",
        "pad_msg.proto",
        "pid_conf.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        ":input_debug_lib",
        ":leadlag_conf_proto_lib",
        ":mrac_conf_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
    ],
)

# leadlag_conf
cc_proto_library(
    name = "leadlag_conf_proto",
    deps = [
        ":leadlag_conf_proto_lib",
    ],
)

proto_library(
    name = "leadlag_conf_proto_lib",
    srcs = ["leadlag_conf.proto"],
)

# mrac_conf
cc_proto_library(
    name = "mrac_conf_proto",
    deps = [
        ":mrac_conf_proto_lib",
    ],
)

proto_library(
    name = "mrac_conf_proto_lib",
    srcs = ["mrac_conf.proto"],
)


# input_debug_proto
cc_proto_library(
    name = "input_debug",
    deps = [
        ":input_debug_lib",
    ],
)

proto_library(
    name = "input_debug_lib",
    srcs = ["input_debug.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

