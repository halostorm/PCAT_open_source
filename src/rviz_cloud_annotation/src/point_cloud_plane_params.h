#pragma once
#ifndef POINT_CLOUD_PLANE_PARAMS
#define POINT_CLOUD_PLANE_PARAMS
// STL
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// System Params
#define _lowerBound -15
#define _upperBound 15
#define _numOfRings 16
#define _horizontalAngleResolution 0.4
//Filter Params
#define _basicRadius 6.9 //基准圆半径
#define _planeRings 6    //Ground ring Size
#define _defaultCurveSize 2000
#define _curveSizeThreshold 50   // Curve点数阈值
// Density Filter Params
#define _srcLenThreshold 0.2
#define _arcNumThreshold 7
// Radius Filter Params
#define _AngleGridResolution 2.0
#define _numOfAngleGrid 180 //  = 360 / _AngleGridResolution
#define _radiusScaleThreshold 0.2
// Size Filter Params
#define _breakingDistanceThreshold 0.2
#define _breakingSizeThreshold 30

#define _max(a, b) (((a) > (b)) ? (a) : (b))
#define _min(a, b) (((a) > (b)) ? (b) : (a))

typedef int64_t int64;
typedef uint64_t uint64;
typedef unsigned short ushort;
typedef std::vector<uint64> Uint64Vector;
typedef std::vector<int64> Int64Vector;

typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;

float m_sqrt(float x); //Fast Sqrt
float getVar(float x[], int len);
#endif