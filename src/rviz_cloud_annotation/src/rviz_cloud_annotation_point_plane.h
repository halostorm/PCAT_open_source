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
