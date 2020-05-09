/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/canbus.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

///添加相关头文件///
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h> 

#include <signal.h>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <typeinfo>
#include <thread>
#include <fstream>

#include <time.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <vector>

using namespace std;

#define PORT 10000
#define DATA_BUFFER  1024

typedef struct {
	int     ID;           //4个字节
	unsigned char INFO;    
	unsigned char LEN;    
	unsigned char DATA[8];//1个字节
}ChasisMsg;

int sClient,sServer;
struct sockaddr_in addr;
struct sockaddr_in client_addr;
int addrlen = sizeof(struct sockaddr_in);

namespace apollo {
namespace canbus {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::control::ControlCommand;
using apollo::drivers::canbus::CanClientFactory;
using apollo::guardian::GuardianCommand;


/////添加控制接口
void control_car(int throttle,int brake,int angle,int direction)//注意：刹车和油门至少一个为零
{
   //需要发送的数据
   ChasisMsg udp_buf;
   udp_buf.ID = 0x0e0;
   udp_buf.INFO = 1;
   udp_buf.LEN = 8;

   udp_buf.DATA[2] = throttle;       //油门值0x00-0xff，从最小到最大,可以直接用十进制调整，0-255(DATA前两个字节保留)
   udp_buf.DATA[3] = angle;          //前轮转角0x00-0xff，从零位到最左/最右,0-255
   udp_buf.DATA[4] = brake;          //刹车值0x00-0xff，从最小到最大
   udp_buf.DATA[5] = 4;              //固定等于4
//P档 bit76=0 驻车档，在熄火停放或汽车静止时使用； =1 D档 前进档； =2 R档 倒车挡，暂时考虑只用P和D档,bit5=0 left   1 right
   udp_buf.DATA[6] = direction;              //0表示左P 64表示左D 128表示左R  右P-32 右D-96  右R-160
   udp_buf.DATA[7] = 0x00;           //用于控制喇叭和灯的开关，如果都关掉就设为0x00，此处暂时关闭  
  

   
   int tmp = sendto(sClient, (char*)&udp_buf, sizeof(udp_buf), 0, (struct sockaddr *)(&addr), addrlen);
   //判断发送是否成功
   if(tmp == -1)
   {
     cout<<"sendto failed"<<endl;
     return;
   }    
}



std::string Canbus::Name() const { return FLAGS_canbus_module_name; }

Status Canbus::Init() {
  //客户端socket
  sClient = socket(AF_INET, SOCK_DGRAM, 0);
  if (sClient < 0)
  {
    cout<< "socket failed sClient"<<endl;
    return OnError("Failed to start sClient.");
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = inet_addr("192.168.4.111");

  // //绑定端口号
  // sServer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  // if (sServer < 0)
  // {
  //   cout<< "socket failed sServer."<<endl;
  //   return OnError("Failed to start sServer.");
  // }

  // set the fixed port to client
  client_addr.sin_family =AF_INET;
  client_addr.sin_port=htons(2222);
  client_addr.sin_addr.s_addr = 0;

  if(bind(sClient,(struct sockaddr*)&client_addr,sizeof(client_addr))<0)
  {	
	  cout<<"bind fail"<<endl;
	  //return OnError("Failed to bind.");
  }

 
  AERROR << "Canbus_1";
  AdapterManager::Init(FLAGS_canbus_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";

  // load conf
  if (!common::util::GetProtoFromFile(FLAGS_canbus_conf_file, &canbus_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_canbus_conf_file);
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
  ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

  // //Init can client
  // auto *can_factory = CanClientFactory::instance();
  // can_factory->RegisterCanClients();
  // can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
  // if (!can_client_) {
  //   return OnError("Failed to create can client.");
  // }
  // AINFO << "Can client is successfully created.";

  // VehicleFactory vehicle_factory;
  // vehicle_factory.RegisterVehicleFactory();
  // auto vehicle_object =
  //     vehicle_factory.CreateVehicle(canbus_conf_.vehicle_parameter());
  // if (!vehicle_object) {
  //   return OnError("Failed to create vehicle:");
  // }

  // message_manager_ = vehicle_object->CreateMessageManager();
  // if (message_manager_ == nullptr) {
  //   return OnError("Failed to create message manager.");
  // }
  // AINFO << "Message manager is successfully created.";

  // if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
  //                        canbus_conf_.enable_receiver_log()) != ErrorCode::OK) {
  //   return OnError("Failed to init can receiver.");
  // }
  // AINFO << "The can receiver is successfully initialized.";

  // if (can_sender_.Init(can_client_.get(), canbus_conf_.enable_sender_log()) !=
  //     ErrorCode::OK) {
  //   return OnError("Failed to init can sender.");
  // }
  // AINFO << "The can sender is successfully initialized.";

  // vehicle_controller_ = vehicle_object->CreateVehicleController();
  // if (vehicle_controller_ == nullptr) {
  //   return OnError("Failed to create vehicle controller.");
  // }
  // AINFO << "The vehicle controller is successfully created.";

  // if (vehicle_controller_->Init(canbus_conf_.vehicle_parameter(), &can_sender_,
  //                               message_manager_.get()) != ErrorCode::OK) {
  //   return OnError("Failed to init vehicle controller.");
  // }
  // AINFO << "The vehicle controller is successfully initialized.";

  CHECK(AdapterManager::GetControlCommand()) << "Control is not initialized.";
  //CHECK(AdapterManager::GetGuardian()) << "Guardian is not initialized.";
  // TODO(QiL) : depreacte this
  if (!FLAGS_receive_guardian) {
    AERROR << "Canbus_2";
    AdapterManager::AddControlCommandCallback(&Canbus::OnControlCommand, this);
  } else {
    AdapterManager::AddGuardianCallback(&Canbus::OnGuardianCommand, this);
  }

  return Status::OK();
}

Status Canbus::Start() {
  // // 1. init and start the can card hardware
  // if (can_client_->Start() != ErrorCode::OK) {
  //   return OnError("Failed to start can client");
  // }
  // AINFO << "Can client is started.";

  // // 2. start receive first then send
  // if (can_receiver_.Start() != ErrorCode::OK) {
  //   return OnError("Failed to start can receiver.");
  // }
  // AINFO << "Can receiver is started.";

  // // 3. start send
  // if (can_sender_.Start() != ErrorCode::OK) {
  //   return OnError("Failed to start can sender.");
  // }

  // // 4. start controller
  // if (vehicle_controller_->Start() == false) {
  //   return OnError("Failed to start vehicle controller.");
  // }

  // 5. set timer to triger publish info periodly
  const double duration = 1.0 / FLAGS_chassis_freq;
  timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                       &Canbus::OnTimer, this);
  AERROR << "Canbus_7";
  // last step: publish monitor messages
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}


////修改////
void Canbus::PublishChassis() {
  AERROR <<"Canbus_4";
  //Chassis chassis = vehicle_controller_->chassis();
  Chassis chassis;
   

  
  chassis.Clear();
  // // 21, 22, previously 1, 2
  // if (driving_mode() == Chassis::EMERGENCY_MODE) {
  //   set_chassis_error_code(Chassis::NO_ERROR);
  // }

  //chassis.set_driving_mode(driving_mode());
  //chassis.set_error_code(chassis_error_code());
  // 3
  chassis.set_engine_started(true);
  // 4
  chassis.set_engine_rpm(0);
  chassis.set_speed_mps(0);
  chassis.set_odometer_m(0);
  // 7
  // lincoln only has fuel percentage
  // to avoid confusing, just don't set
  chassis.set_fuel_range_m(0);
  chassis.set_throttle_percentage(0);
  chassis.set_brake_percentage(0);
  chassis.set_gear_location(Chassis::GEAR_NONE);
  chassis.set_steering_percentage(0);
  chassis.set_steering_torque_nm(0);
  chassis.set_parking_brake(false);
  chassis.mutable_signal()->set_high_beam(false);
  chassis.mutable_signal()->set_turn_signal(common::VehicleSignal::TURN_NONE);
  chassis.mutable_signal()->set_horn(false);

  AdapterManager::FillChassisHeader(FLAGS_canbus_node_name, &chassis);

  AdapterManager::PublishChassis(chassis);
  ADEBUG << chassis.ShortDebugString();
}

void Canbus::PublishChassisDetail() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  ADEBUG << chassis_detail.ShortDebugString();

  AdapterManager::PublishChassisDetail(chassis_detail);
}

void Canbus::OnTimer(const ros::TimerEvent &) {
  AERROR << "Canbus_5";
  unsigned char recv_buf[DATA_BUFFER];
  int len = sizeof(client_addr); 
  AERROR << "Canbus_6";  
  recvfrom(sClient, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&client_addr, (socklen_t *)&len);//阻塞模式！！！！！
       
  //cout << "len of recv_buf:" << strlen(recv_buf) <<endl;
  // 从recv_buf中解析需要的数据进行发布
  int id       = int(recv_buf[0]);
  int angle    = (int)(recv_buf[6]|recv_buf[7]<<8);
  int throtlle = (int)(recv_buf[10]|recv_buf[11]<<8);
  int speed    = int(recv_buf[12]);
  int brake    = (int)(recv_buf[8]|recv_buf[9]<<8);
  int gear     = int(recv_buf[13]);
 
  if(id== 240) //注意啊，每一组三个ID的消息，几乎同时到达，但是10HZ，消息发的再快，解析因为ROS机制只能强行等到0.1s才能去解析，这样正好是0.3s一次。也就是说尽管是一组，但是还是一条一条解析。
  { 
  PublishChassis();
  }
  if (FLAGS_enable_chassis_detail_pub) {
    PublishChassisDetail();
  }
}

void Canbus::Stop() {
  timer_.stop();

  // can_sender_.Stop();
  // can_receiver_.Stop();
  // can_client_->Stop();
  // vehicle_controller_->Stop();
}

void Canbus::OnControlCommand(const ControlCommand &control_command) {
  AERROR <<"Canbus_3"; 
  int64_t current_timestamp =
      apollo::common::time::AsInt64<common::time::micros>(Clock::Now());
  // if command coming too soon, just ignore it.
  if (current_timestamp - last_timestamp_ < FLAGS_min_cmd_interval * 1000) {
    ADEBUG << "Control command comes too soon. Ignore.\n Required "
              "FLAGS_min_cmd_interval["
           << FLAGS_min_cmd_interval << "], actual time interval["
           << current_timestamp - last_timestamp_ << "].";
    return;
  }

  last_timestamp_ = current_timestamp;
  ADEBUG << "Control_sequence_number:"
         << control_command.header().sequence_num() << ", Time_of_delay:"
         << current_timestamp - control_command.header().timestamp_sec();

  // if (vehicle_controller_->Update(control_command) != ErrorCode::OK) {
  //   AERROR << "Failed to process callback function OnControlCommand because "
  //             "vehicle_controller_->Update error.";
  //   return;
  // }
  // can_sender_.Update();

  ///////修改，直接将控制命令传入底层控制////////////
   AERROR << control_command.throttle() <<' '<< control_command.brake();
   if(control_command.throttle() != 0 && control_command.brake() != 0)
   {
     control_car(0,0,0,64);
    
   }
   else
   {
     control_car(control_command.throttle(),control_command.brake(),0,64);
   }

}

void Canbus::OnGuardianCommand(const GuardianCommand &guardian_command) {
  apollo::control::ControlCommand control_command;
  control_command.CopyFrom(guardian_command.control_command());
  OnControlCommand(control_command);
}

// Send the error to monitor and return it
Status Canbus::OnError(const std::string &error_msg) {
  apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace canbus
}  // namespace apollo
