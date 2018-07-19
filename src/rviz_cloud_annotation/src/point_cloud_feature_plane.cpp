#include "point_cloud_feature_plane.h"

Uint64Vector PointCloudFeaturePlane::vote_blocks(const PointXYZRGBNormalCloud &cloud)
{
  Uint64Vector res;

  const int64 cloud_size = cloud.size();
  ROS_INFO("point_cloud_feature_plane: cloud_size: %ld", cloud_size);
  int64 max = 0;
  float r;
  float theta;
  float phi;
  for (int64 i = 0; i < cloud_size; i++)
  {
    float x = cloud[i].x;
    float y = cloud[i].y;
    float z = cloud[i].z;
    bool find = false;
    for (int a = 0; a < R_SIZE && !find; a++)
    {
      for (int b = 0; b < THETA_SIZE && !find; b++)
      {
        for (int c = 0; c < PHI_SIZE && !find; c++)
        {
          if (InBlock(a, b, c, x, y, z))
          {
            m_blocks[a][b][c]++;
            find = true;
            // goto NEXT;
            if (max < m_blocks[a][b][c])
            {
              max = m_blocks[a][b][c];
              r = r_value[a];
              theta = theta_value[b];
              phi = phi_value[c];
            }
          }
        }
      }
    }
  }
  ROS_INFO("point_cloud_feature_plane: max point: %ld", max);

  ROS_INFO("point_cloud_feature_plane: params: %f %f %f", r, theta, phi);
  return res;
}