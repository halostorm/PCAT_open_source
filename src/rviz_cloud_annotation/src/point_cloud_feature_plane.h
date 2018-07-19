#include <sys/stat.h>
#include <sys/types.h>

// STL
#include <dirent.h>
#include <iostream>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64MultiArray.h>
#include <termios.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/colors.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define _M_PI 3.141592654

#define R_SIZE 10
#define THETA_SIZE 10
#define PHI_SIZE 360

typedef int64_t int64;
typedef uint64_t uint64;
typedef std::vector<uint64> Uint64Vector;

typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;

class PointCloudFeaturePlane
{
private:
  // Plane Params
  const float m_r_step = 0.05;
  const float m_theta_step = 1 / 180 * _M_PI;
  const float m_phi_step = 1 / 180 * _M_PI;

  const float m_r_max = 2;
  const float m_r_min = 1.5;
  const float m_theta_max = _M_PI * 5 / 6;
  const float m_theta_min = _M_PI * 17 / 18;
  const float m_phi_max = 2 * _M_PI;
  const float m_phi_min = 0;

  int64 m_blocks[R_SIZE][THETA_SIZE][PHI_SIZE] = {{{0}}};

  float r_value[R_SIZE];
  float theta_value[THETA_SIZE];
  float phi_value[PHI_SIZE];

  float sinTheta[THETA_SIZE];
  float cosTheta[THETA_SIZE];
  float sinPhi[PHI_SIZE];
  float cosPhi[PHI_SIZE];

public:
  float m_sqrt(float x)
  {
    float half_x = 0.5 * x;
    int i = *((int *)&x);              // 以整数方式读取X
    i = 0x5f3759df - (i >> 1);         // 神奇的步骤
    x = *((float *)&i);                // 再以浮点方式读取i
    x = x * (1.5 - (half_x * x * x));  // 牛顿迭代一遍提高精度
    return 1 / x;
  }

  void generate_blocks_value()
  {
    for (int i = 0; i < R_SIZE; i++)
    {
      r_value[i] = m_r_step * i + m_r_min;
    }
    for (int i = 0; i < THETA_SIZE; i++)
    {
      theta_value[i] = m_theta_step * i + m_theta_min;
      sinTheta[i] = sin(theta_value[i]);
      cosTheta[i] = cos(theta_value[i]);
    }
    for (int i = 0; i < PHI_SIZE; i++)
    {
      phi_value[i] = m_phi_step * i + m_phi_min;
      sinPhi[i] = sin(phi_value[i]);
      cosPhi[i] = cos(phi_value[i]);
    }
  }

  bool InBlock(int i, int j, int k, float x, float y, float z)
  {
    float r= r_value[i];
    float sinT = sinTheta[j];
    float cosT = cosTheta[j];
    float sinP = sinPhi[k];
    float cosP = cosPhi[k];

    if(fabs(r - (x*sinT*cosP+y*sinT*sinP+z*cosT))<0.001){//r = x0·sinθcosφ + y0·sinθsinφ + z0·cosθ
        return true;
    }else{
        return false;
    }
  }

  Uint64Vector vote_blocks(const PointXYZRGBNormalCloud &cloud);
};