load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "controller_interface",
    srcs = [],
    hdrs = [
        "controller.h",
    ],
    deps = [],
)

cc_library(
    name = "lat_controller",
    srcs = [
        "lat_controller.cc",
    ],
    hdrs = [
        "lat_controller.h",
    ],
    deps = [
        ":controller_interface",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/filters:mean_filter",
        "//modules/common/math:euler_angles_zxy",
        "//modules/common/math:geometry",
        "//modules/common/math:lqr",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_1d",
        "//modules/control/common:trajectory_analyzer",
        "//modules/control/proto:control_proto",
        "//modules/control/common:leadlag_controller",
        "//modules/control/common:mrac_controller",
        "@eigen",
    ],
)

cc_library(
    name = "lon_controller",
    srcs = [
        "lon_controller.cc",
    ],
    hdrs = [
        "lon_controller.h",
    ],
    deps = [
        ":controller_interface",
        "//modules/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/control/common:control_gflags",
        "//modules/control/common:interpolation_2d",
        "//modules/control/common:leadlag_controller",
        "//modules/control/common:pid_controller",
        "//modules/control/common:trajectory_analyzer",
        "//modules/localization/common:localization_common",
    ],
)



cc_library(
    name = "controller",
    deps = [
        ":controller_agent",
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        "@eigen",
    ],
)

cc_library(
    name = "controller_agent",
    srcs = [
        "controller_agent.cc",
    ],
    hdrs = [
        "controller_agent.h",
    ],
    deps = [
        ":controller_interface",
        ":lat_controller",
        ":lon_controller",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util:factory",
        "//modules/control/proto:control_proto",
        "//modules/planning/proto:planning_proto",
        "@ros//:ros_common",
    ],
)



cpplint()
