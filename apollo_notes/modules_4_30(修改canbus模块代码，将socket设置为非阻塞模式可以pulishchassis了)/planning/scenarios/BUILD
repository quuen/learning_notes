load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "stage",
    srcs = ["stage.cc"],
    hdrs = ["stage.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks:task_factory",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_context",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios/lane_follow",
        "//modules/planning/scenarios/park/valet_parking",
    ],
)

cpplint()
