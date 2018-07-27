#ifndef RVIZ_CLOUD_ANNOTATION_POINT_PLANE_H
#define RVIZ_CLOUD_ANNOTATION_POINT_PLANE_H

#include <stdint.h>
#include <vector>
#include <queue>
#include <set>

#include <boost/shared_ptr.hpp>

#include "point_neighborhood.h"

class RVizCloudAnnotationPointsPointPlane
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<bool> BoolVector;
  typedef std::queue<uint64> Uint64Queue;
  typedef std::set<uint64> Uint64Set;
  typedef boost::shared_ptr<RVizCloudAnnotationPointsPointPlane> Ptr;
  typedef boost::shared_ptr<const RVizCloudAnnotationPointsPointPlane> ConstPtr;

  RVizCloudAnnotationPointsPointPlane(const uint64 cloud_size,
                                      const PointNeighborhood & point_neighborhood,
                                      const float multiplier);

  void UpdateRegionGrowing(const Uint64Vector & seeds,
                           BoolVector & touched_labels,
                           Uint64Set & touched_points);
  void RemoveLabel(const uint64 label_id,
                   const uint64 point_id,
                   BoolVector & touched_labels,
                   Uint64Set & touched_points);

  void Clear();

  void SetSeed(const uint64 point_id,const uint32 label_id);

  uint32 GetLabel(const uint64 point_id) const {return m_labels_assoc[point_id]; }
  float GetTotDist(const uint64 point_id) const {return m_last_generated_tot_dists[point_id]; }

  private:
  Uint64Vector m_labels_assoc;
  FloatVector m_last_generated_tot_dists;

  uint64 m_cloud_size;

  const PointNeighborhood & m_point_neighborhood;

  float m_multiplier;
};

#endif // RVIZ_CLOUD_ANNOTATION_POINT_PLANE_H
