#include "point_cloud_plane_curves_extract.h"

void PointCloudPlaneCurvesExtract::SearchCurves(const PointXYZRGBNormalCloud &PointCloud)
{
  for (int i = 0; i < _numOfRings; i++)
  {
    mCurvesVector[i].reserve(_defaultCurveSize);
    mCurvesId[i].reserve(_defaultCurveSize);
  }
  for (uint64 i = 0; i < PointCloud.size(); i++)
  {
    PointXYZRGBNormal point = PointCloud[i];
    float angle = std::atan(point.z / m_sqrt(point.x * point.x + point.y * point.y)) / (1.0f * M_PI) * 180.0f;
    int64 ringID = GetScanringID(angle);
    if (ringID < _numOfRings && ringID >= 0)
    {
      mCurvesVector[ringID].push_back(point);
      mCurvesId[ringID].push_back(i);
    }
  }

  for (int i = 0; i < _planeRings; i++)
  {
    mScanringRadius[i] = GetScanringRadius(i);
  }
  // DensityFilter
  for (int i = 0; i < _planeRings; i++)
  {
    if (mCurvesVector[i].size() > _curveSizeThreshold)
    {
      CurveDensityFilter(mCurvesVector[i], i, mCurvesId[i], mDensityCurvesVector[i]);
    }
  }
  // RadiusFilter
  CurvesRadiusFilter(mDensityCurvesVector, mDensityCurvesId);
  // SizeFilter
  for (int i = 0; i < _planeRings; i++)
  {
    if (mRadiusCurvesVector[i].size() > _curveSizeThreshold)
    {
      CurveSizeFilter(mRadiusCurvesVector[i], i, mRadiusCurvesId[i], mSizeCurvesVector[i]);
    }
  }
}

void PointCloudPlaneCurvesExtract::CurveDensityFilter(const PointXYZRGBNormalCloud &Curve, const int64 ringID,
                                                      const Uint64Vector &curveId, PointXYZRGBNormalCloud &outCurve)
{
  outCurve.reserve(Curve.size());
  mDensityCurvesId[ringID].reserve(curveId.size());
  float TrueRadius = mScanringRadius[ringID];
  if (TrueRadius < 0)
  {
    return;
  }
  float arcLen = 0;
  float radiusMean = 0;
  float radius = TrueRadius;
  float radius_1 = TrueRadius;
  int64 arcNum = 0;
  for (int i = 1; i < Curve.size(); i += 1)
  {
    float dx = Curve[i].x - Curve[i - 1].x;
    float dy = Curve[i].y - Curve[i - 1].y;
    float dz = Curve[i].z - Curve[i - 1].z;
    arcNum++;
    radius = m_sqrt(Curve[i].x * Curve[i].x + Curve[i].y * Curve[i].y);
    radiusMean += radius;
    arcLen += (m_sqrt(dx * dx + dy * dy + dz * dz)) * (2 * _basicRadius / (radius + radius_1));
    radius_1 = radius;
    if (arcLen >= _srcLenThreshold)
    {
      radiusMean /= arcNum;
      if (arcNum > _arcNumThreshold)
      {
        for (int k = i - arcNum + 1; k <= i; k++)
        {
          mDensityCurvesId[ringID].push_back(curveId[k]);
          outCurve.push_back(Curve[k]);
        }
      }
      arcLen = 0;
      arcNum = 0;
      radiusMean = 0;
    }
  }
}

void *PointCloudPlaneCurvesExtract::CurvesRadiusFilter(PointXYZRGBNormalCloud *CurvesVector,
                                                       const Uint64Vector *CurvesId)
{
  for (int i = 0; i < _planeRings; i++)
  {
    mSentorLabelVector[i].reserve(CurvesVector[i].size());
    for (int j = 0; j < CurvesVector[i].size(); j++)
    {
      float x = CurvesVector[i][j].x;
      float y = CurvesVector[i][j].y;
      int AngleGridId = (int)(((atan2f(y, x) / M_PI * 180 + 180) / _AngleGridResolution));
      if (AngleGridId < 0 || AngleGridId >= _numOfAngleGrid)
      {
        continue;
      }
      CurvesVector[i][j].rgba = (uint)AngleGridId;
      float radius = m_sqrt(x * x + y * y);
      CurvesVector[i][j].curvature = radius;  // / mScanringRadius[i];

      mSentorIds[i][AngleGridId].push_back(j);
      mSentorLabelVector[i].push_back(0);
      mSentorMeanRadius[i][AngleGridId] =
          (mSentorMeanRadius[i][AngleGridId] * mSentorIds[i][AngleGridId].size() + radius) /
          (mSentorIds[i][AngleGridId].size() + 1);
    }
  }

  for (int i = 0; i < _planeRings - 1; i++)
  {
    for (int j = 0; j < _numOfAngleGrid; j++)
    {
      if (fabs(mSentorMeanRadius[i][j] - mSentorMeanRadius[i + 1][j]) <
          _radiusScaleThreshold * fabs(mScanringRadius[i + 1] - mScanringRadius[i]))
      {
        if (i == 0)
        {
          for (int k = 0; k < mSentorIds[i][j].size(); k++)
          {
            uint64 id = mSentorIds[i][j][k];
            mSentorLabelVector[i][id] = -1;
          }
          mSentorIds[i][j].clear();
        }
        for (int k = 0; k < mSentorIds[i + 1][j].size(); k++)
        {
          uint64 id = mSentorIds[i + 1][j][k];
          mSentorLabelVector[i + 1][id] = -1;
        }
        mSentorIds[i + 1][j].clear();
      }
    }
  }
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < CurvesVector[i].size(); j++)
    {
      if (mSentorLabelVector[i][j] != -1)
      {
        mRadiusCurvesId[i].push_back(CurvesId[i][j]);
        mRadiusCurvesVector[i].push_back(CurvesVector[i][j]);
      }
    }
  }
}

void PointCloudPlaneCurvesExtract::CurveSizeFilter(const PointXYZRGBNormalCloud &Curve, const int64 ringID,
                                                   const Uint64Vector &curveId, PointXYZRGBNormalCloud &outCurve)
{
  outCurve.reserve(Curve.size());
  mSizeCurvesId[ringID].reserve(curveId.size());
  float TrueRadius = mScanringRadius[ringID];
  float ratio = _basicRadius / TrueRadius;
  if (TrueRadius < 0 || Curve.size() < 1)
  {
    return;
  }
  int LabelArray[Curve.size()];
  int64 begin = 0;
  int64 end = 0;
  float meanZ = 0;
  for (uint64 i = 1; i < Curve.size(); i += 1)
  {
    LabelArray[i] = 0;
    float dx = Curve[i].x - Curve[i - 1].x;
    float dy = Curve[i].y - Curve[i - 1].y;
    float dz = Curve[i].z - Curve[i - 1].z;
    meanZ += dz;
    float dis = (m_sqrt(dx * dx + dy * dy + dz * dz)) * (ratio);
    if (dis > _breakingDistanceThreshold)
    {
      end = i;
      meanZ /= (end - begin);
      if (end - begin < _breakingSizeThreshold)
      {
        for (uint64 k = begin; k < end; k++)
        {
          LabelArray[k] = -1;
        }
      }
      begin = end;
    }
  }
  for (uint64 i = 0; i < Curve.size(); i += 1)
  {
    if (LabelArray[i] != -1)
    {
      mSizeCurvesId[ringID].push_back(curveId[i]);
      outCurve.push_back(Curve[i]);
    }
  }
}

int64 PointCloudPlaneCurvesExtract::GetScanringID(const float &angle)
{
  return static_cast<int64>((angle - _lowerBound) / (1.0f * (_upperBound - _lowerBound)) * (_numOfRings - 1) + 0.5);
}

float PointCloudPlaneCurvesExtract::GetScanringRadius(const int64 ID)
{
  float TrueRadius = -1;
  float angle = (_upperBound - _lowerBound) * 1.0 / _numOfRings * ID + _lowerBound;
  if (angle < -2)
  {
    TrueRadius = _basicRadius * fabs(tan(_lowerBound * 1.0 / 180 * M_PI)) * fabs(tan((90 + angle) * 1.0 / 180 * M_PI));
  }
  return TrueRadius;
}