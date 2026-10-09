// Deterministic source-contract probe; no ROS, controls or tracking SDK calls.
#include <cstdint>
#include <iostream>
#include "isaac_ros_visual_slam/impl/message_stream_sequencer.hpp"
using nvidia::isaac_ros::visual_slam::MessageStreamSequencer;

int before_new_imu(int64_t multiplier) {
  MessageStreamSequencer<int,int> seq(144,12*multiplier,100,60*multiplier);
  int callbacks=0;
  seq.RegisterCallback([&](const auto &,const auto &){++callbacks;});
  constexpr int64_t start=1000000000;
  seq.CallbackStream1(start,1);
  seq.CallbackStream2(start,1);
  for(int n=1;n<=9;++n)seq.CallbackStream1(start+n*4000000,1);
  seq.CallbackStream2(start+40000000,2);
  int before=callbacks;
  seq.CallbackStream1(start+40000000,1);
  if(callbacks!=2)return -1;
  return before;
}
int main() {
  unsigned int configured=400;
  MessageStreamSequencer<int,int> truncated(configured,12,100,60);
  for(int n=0;n<400;++n)truncated.CallbackStream1(1000000000+n*4000000LL,n);
  int capacity=truncated.GetSizeStream1();
  int unscaled=before_new_imu(1),scaled=before_new_imu(1000000);
  std::cout<<"{\"configured_capacity\":400,\"effective_capacity\":"<<capacity
    <<",\"unscaled_callbacks_before_current_imu\":"<<unscaled
    <<",\"scaled_callbacks_before_current_imu\":"<<scaled<<"}\n";
  return capacity==144 && unscaled==2 && scaled==1 ? 0 : 1;
}
