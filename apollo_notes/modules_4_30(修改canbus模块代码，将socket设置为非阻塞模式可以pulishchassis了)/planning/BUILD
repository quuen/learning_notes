load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_lib",
    srcs = [
        "navi_planning.cc",
        "planning_base.cc",
        "std_planning.cc",
    ],
    hdrs = [
        "navi_planning.h",
        "planning.h",
        "planning_base.h",
        "std_planning.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/common/math:quaternion",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:thread_pool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/planner:planner_dispatcher",
        "//modules/planning/planner/public_road:public_road_planner",
        "//modules/planning/planner/lattice:lattice_planner",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:traffic_rule_config_proto",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/planning/traffic_rules:traffic_decider",
        "//modules/prediction/proto:prediction_proto",
        "//modules/planning/tasks:task_factory",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "planning",
    srcs = [
        "main.cc",
        "planning.h",
    ],
    deps = [
        ":planning_lib",
        "//external:gflags",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

filegroup(
    name = "planning_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()
