#ifndef POINT_CLOUD_PLANE_CURVES
#define POINT_CLOUD_PLANE_CURVES

#include "point_cloud_plane_params.h"

class PointCloudPlaneCurvesExtract
{
public:
  PointXYZRGBNormalCloud mCurvesVector[_numOfRings];
  PointXYZRGBNormalCloud mDensityCurvesVector[_numOfRings];
  PointXYZRGBNormalCloud mRadiusCurvesVector[_numOfRings];
  PointXYZRGBNormalCloud mSizeCurvesVector[_numOfRings];

  Uint64Vector mCurvesId[_numOfRings];
  Uint64Vector mDensityCurvesId[_numOfRings];
  Uint64Vector mSizeCurvesId[_numOfRings];
  Uint64Vector mRadiusCurvesId[_numOfRings];

  Uint64Vector mAnglePointId[_numOfRings][_numOfAngleGrid];
  PointXYZRGBNormalCloud mAnglePointVector[_numOfRings][_numOfAngleGrid];

  float mSentorMeanRadius[_numOfRings][_numOfAngleGrid] = {{0}};
  Uint64Vector mSentorIds[_numOfRings][_numOfAngleGrid];
  Uint64Vector mSentorLabelVector[_numOfRings];

  float mScanringRadius[_numOfRings];

  void SearchCurves(const PointXYZRGBNormalCloud &PointCloud);
  void CurveDensityFilter(const PointXYZRGBNormalCloud &Curve, const int64 ringID, const Uint64Vector &curveId,
                          PointXYZRGBNormalCloud &outCurve);
  void *CurvesRadiusFilter(PointXYZRGBNormalCloud *CurvesVector, const Uint64Vector *CurvesId);
  void CurveSizeFilter(const PointXYZRGBNormalCloud &Curve, const int64 ringID, const Uint64Vector &curveId,
                       PointXYZRGBNormalCloud &outCurve);

  int64 GetScanringID(const float &angle);
  float GetScanringRadius(const int64 ID);
};
#endif