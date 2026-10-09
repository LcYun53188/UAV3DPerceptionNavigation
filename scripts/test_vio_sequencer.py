"""Compile the actual SDK constructor wiring to catch unit-conversion regressions."""
from pathlib import Path
import re
import subprocess

ROOT=Path(__file__).resolve().parents[1]


def test_actual_constructor_waits_for_imu_within_configured_millisecond_tolerance(tmp_path):
    impl=(ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp').read_text()
    args=re.search(r'  sequencer\((.*?)\),\n  tf_buffer',impl,re.S).group(1)
    source='''
#include <cstdint>
#include "isaac_ros_visual_slam/impl/message_stream_sequencer.hpp"
using nvidia::isaac_ros::visual_slam::MessageStreamSequencer;
int probe(int64_t lag) {
 struct { unsigned int imu_buffer_size_=144;double imu_jitter_threshold_ms_=12;
 unsigned int image_buffer_size_=100;double image_jitter_threshold_ms_=60; } node;
 MessageStreamSequencer<int,int> seq(ARGS);
 int callbacks=0;
 seq.RegisterCallback([&](const auto &,const auto &){++callbacks;});
 int64_t start=1000000000;
 seq.CallbackStream1(start,1);seq.CallbackStream2(start,1);
 seq.CallbackStream1(start+40000000-lag,1);seq.CallbackStream2(start+40000000,2);
 int before=callbacks;seq.CallbackStream1(start+40000000,1);
 if(callbacks!=2)return -1;
 return before;
}
int main() {return probe(4000000)==1 && probe(12000000)==1 && probe(12000001)==2 ? 0 : 1;}
'''.replace('ARGS',args)
    path=tmp_path/'probe.cpp';path.write_text(source);binary=tmp_path/'probe'
    subprocess.run(['g++','-std=c++17','-I',str(ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/include'),
        '-I',str(ROOT/'src/isaac_ros_common/isaac_common/include'),str(path),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
