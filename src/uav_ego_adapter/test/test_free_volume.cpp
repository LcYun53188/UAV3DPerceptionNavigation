// SPDX-License-Identifier: GPL-3.0-only
#include <gtest/gtest.h>
#include <limits>
#include "../src/free_volume.hpp"

TEST(FreeVolume, ClearCentreCannotHideBlockedBodyVoxel) {
  const std::array<uint32_t,3> shape{8,7,6};
  std::vector<float> distance(8*7*6,3.f);
  std::vector<uint8_t> observed(distance.size(),1);
  const Eigen::Vector3i lo(2,2,2),hi(5,4,4);
  const size_t obstacle=(size_t(5)*7+3)*6+3;
  FreeVolume volume;
  volume.update(shape,distance,observed);
  ASSERT_TRUE(volume.isFree(lo,hi));
  for(float value : {-.1f,0.f,std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity()}) {
    distance[obstacle]=value;
    volume.update(shape,distance,observed);
    EXPECT_FALSE(volume.isFree(lo,hi));
    EXPECT_TRUE(volume.isFree(Eigen::Vector3i(3,3,3),Eigen::Vector3i(3,3,3)));
  }
  distance[obstacle]=3.f;observed[obstacle]=0;
  volume.update(shape,distance,observed);
  EXPECT_FALSE(volume.isFree(lo,hi));
  EXPECT_FALSE(volume.isFree(Eigen::Vector3i(-1,0,0),hi));
  EXPECT_FALSE(volume.isFree(lo,Eigen::Vector3i(8,4,4)));
}

TEST(FreeVolume, PrefixMatchesBruteForceForAllBoxes) {
  const std::array<uint32_t,3> shape{5,4,3};
  std::vector<float> distance(60,3.f);
  std::vector<uint8_t> observed(60,1);
  observed[(1*4+2)*3+1]=0;
  distance[(3*4+1)*3+2]=-.1f;
  FreeVolume volume;volume.update(shape,distance,observed);
  for(int x=0;x<5;++x)for(int y=0;y<4;++y)for(int z=0;z<3;++z)
    for(int a=x;a<5;++a)for(int b=y;b<4;++b)for(int c=z;c<3;++c) {
      bool expected=true;
      for(int i=x;i<=a;++i)for(int j=y;j<=b;++j)for(int k=z;k<=c;++k) {
        const size_t n=(i*4+j)*3+k;
        expected&=observed[n] && distance[n]>0;
      }
      EXPECT_EQ(volume.isFree(Eigen::Vector3i(x,y,z),Eigen::Vector3i(a,b,c)),expected);
    }
}
