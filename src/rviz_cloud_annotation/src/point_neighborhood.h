/*
 * Copyright (c) 2016-2017, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
