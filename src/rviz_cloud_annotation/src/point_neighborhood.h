#ifndef POINT_NEIGHBORHOOD_H
#define POINT_NEIGHBORHOOD_H

// STL
#include <vector>
#include <stdint.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "point_neighborhood_search.h"

class PointNeighborhood
{
  public:
  typedef uint64_t uint64;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<FloatVector> FloatVectorVector;

  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef boost::shared_ptr<PointNeighborhood> Ptr;
  typedef boost::shared_ptr<const PointNeighborhood> ConstPtr;

  struct Conf
  {
    float color_importance;
    float position_importance;
    float normal_importance;

    float max_distance;

    PointNeighborhoodSearch::Searcher::ConstPtr searcher;
  };

  struct Neighs
  {
    uint64 size;
    const uint64 * neighbors;
    const float * total_dists;
    const float * position_dists;
  };

  typedef std::vector<Neighs> NeighsVector;

  PointNeighborhood(PointXYZRGBNormalCloud::ConstPtr cloudptr,const Conf & conf);
  virtual ~PointNeighborhood() {}

  float TotalDistance(const PointXYZRGBNormal & a,const PointXYZRGBNormal & b,const Conf & conf) const;

  uint64 GetNeighborhoodAsPointer(const uint64 i,const uint64 * & neighbors,
                                 const float * & total_dists,const float * & position_dists) const
  {
    neighbors = m_index[i].neighbors;
    total_dists = m_index[i].total_dists;
    position_dists = m_index[i].position_dists;
    return m_index[i].size;
  }

  // conf used at creation
  const Conf & GetConf() const {return m_conf; }

  private:
  void RemoveUnidirectionalLinks(Uint64Vector & temporary_indices_vector);
  void AddUnidirectionalLinks(Uint64Vector & temporary_indices_vector);

  NeighsVector m_index;
  Uint64Vector m_neighbors;
  FloatVector m_total_dists;
  FloatVector m_position_dists;

  Conf m_conf;
};

#endif // POINT_NEIGHBORHOOD_H
