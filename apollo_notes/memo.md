### 导航模式下到录制轨迹终点Planning停不下来

***

* `/modules/planning/navi_planning.cc`

  >`FLAGS_publish_estop`在`/modules/planning/common/planning_gflags.cc`默认设置是`false`
  >**代码为**：`DEFINE_bool(publish_estop, false, "publish estop decision in planning");`
  >**这样会在规划不成功的时候生成`estop`（紧急刹车）指令**


  ```c++
        if (FLAGS_publish_estop) {//上面还有代码会判断规划是否成功
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
  >
  >**代码为**：`DEFINE_bool(enable_persistent_estop, false, "True to persistent keep estop status, " "pad reset can reset the estop status.");`


  ```c++
    // check estop
    estop_ = FLAGS_enable_persistent_estop
                 ? estop_ || trajectory_.estop().is_estop()
                 : trajectory_.estop().is_estop();
  ```

***

**主要原因：**

* 在路测中[参考线跑完仍会继续行驶](https://github.com/ApolloAuto/apollo/issues/5805)，到达终点后需要人工接管。目前相对地图的逻辑是 完成指引线后，会继续延实时车道线自动驾驶。

  也就是即使未发送参考线，也会有相对地图产生，默认车头方向生成一定距离的笔直车道，这也是抵达参考线终点会
  继续使用这种情况下产生的地图而不会停下来。要解决的问题是不发送参考线的时候就不应该产生相对地图。

***

* `/modules/control/control.cc`
  * 规划部分决策出`not_ready`（这种情况一般是`localization`丢失）的信息，也需要刹车
  * **导航规划部分快接近参考线终点时会产生`STOP_REASON_DESTINATION`的信息，是否要考虑加入停车判断当中？**


```c++
  if (estop_) {// 此处添加 || trajectory_.decision().main_decision().has_not_ready()
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
```

***

