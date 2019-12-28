### 导航模式下到录制轨迹终点Planning停不下来

***

* `/modules/planning/navi_planning.cc`

  >`FLAGS_publish_estop`在`/modules/planning/common/planning_gflags.cc`默认设置是`false`
  >**代码为**：`DEFINE_bool(publish_estop, false, "publish estop decision in planning");`


  ```c++
        if (FLAGS_publish_estop) {
          AERROR << "Planning failed and set estop";
          // Because the function "Control::ProduceControlCommand()" checks the
          // "estop" signal with the following line (Line 170 in control.cc):
          // estop_ = estop_ || trajectory_.estop().is_estop();
          // we should add more information to ensure the estop being triggered.
          EStop* estop = trajectory_pb->mutable_estop();
          estop->set_is_estop(true);
          estop->set_reason(status.error_message());
        }
  ```

* `/modules/control/control.cc`

  >`FLAGS_enable_persistent_estop`在`/modules/control/common/control_gflags.cc`默认设置是`false`
  >**代码为**：`DEFINE_bool(enable_persistent_estop, false, "True to persistent keep estop status, " "pad reset can reset the estop status.");`


  ```c++
    // check estop
    estop_ = FLAGS_enable_persistent_estop
                 ? estop_ || trajectory_.estop().is_estop()
                 : trajectory_.estop().is_estop();
  ```

***

