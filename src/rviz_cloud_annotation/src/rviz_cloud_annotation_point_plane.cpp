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

#include "rviz_cloud_annotation_point_plane.h"

#include <cmath>

RVizCloudAnnotationPointsPointPlane::RVizCloudAnnotationPointsPointPlane(const uint64 cloud_size,
  const PointNeighborhood & point_neighborhood,const float multiplier): m_point_neighborhood(point_neighborhood)
{
  m_multiplier = multiplier;
  m_cloud_size = cloud_size;

  m_labels_assoc.resize(cloud_size,0);
  m_last_generated_tot_dists.resize(cloud_size,0.0);
}

void RVizCloudAnnotationPointsPointPlane::Clear()
{
  m_labels_assoc.assign(m_cloud_size,0);
  m_last_generated_tot_dists.assign(m_cloud_size,0.0);
}

void RVizCloudAnnotationPointsPointPlane::RemoveLabel(const uint64 label_id,
                                                      const uint64 point_id,
                                                      BoolVector & touched_labels,
                                                      Uint64Set & touched_points)
{
  Uint64Set seeds_set;

  Uint64Queue queue;
  queue.push(point_id);

  m_labels_assoc[point_id] = 0;
  m_last_generated_tot_dists[point_id] = 0.0;
  touched_points.insert(point_id);

  while (!queue.empty())
  {
    const uint64 curr_point_id = queue.front();
    queue.pop();

    const float * neigh_dists;
    const float * neigh_tot_dists;
    const uint64 * neighs;
    const uint64 neighs_size =
      m_point_neighborhood.GetNeighborhoodAsPointer(curr_point_id,neighs,neigh_tot_dists,neigh_dists);

    for (uint64 h = 0; h < neighs_size; h++)
    {
      const uint64 next_point_id = neighs[h];

      const uint32 label = m_labels_assoc[next_point_id];
      if (label == 0)
        continue;

      if (label != label_id)
        seeds_set.insert(next_point_id);
      if (label == label_id)
      {
        queue.push(next_point_id);

        m_labels_assoc[next_point_id] = 0;
        m_last_generated_tot_dists[next_point_id] = 0.0;
        touched_points.insert(next_point_id);
      }
    }
  }

  const Uint64Vector seeds(seeds_set.begin(),seeds_set.end());

  UpdateRegionGrowing(seeds,touched_labels,touched_points);
  touched_labels[label_id - 1] = true;
}

void RVizCloudAnnotationPointsPointPlane::UpdateRegionGrowing(const Uint64Vector & seeds,
                                                              BoolVector & touched_labels,
                                                              Uint64Set & touched_points)
{
  Uint64Queue queue;
  BoolVector in_queue(m_cloud_size,false);
  for (uint64 i = 0; i < seeds.size(); i++)
  {
    const uint64 first = seeds[i];
    queue.push(first);
    in_queue[first] = true;
    touched_points.insert(first);
  }

  if (!std::isfinite(m_multiplier))
    return; // do not expand if multiplier is infinite (i.e. weight is zero)

  while (!queue.empty())
  {
    const uint64 current = queue.front();
    queue.pop();
    in_queue[current] = false;

    const float current_tot_dist = m_last_generated_tot_dists[current];
    const uint64 current_label = m_labels_assoc[current];

    const float * neigh_dists;
    const float * neigh_tot_dists;
    const uint64 * neighs;
    const uint64 neighs_size = m_point_neighborhood.GetNeighborhoodAsPointer(current,neighs,neigh_tot_dists,neigh_dists);

    for (uint64 i = 0; i < neighs_size; i++)
    {
      const uint64 next = neighs[i];
      const float neigh_tot_dist = neigh_tot_dists[i] * m_multiplier;
      const float next_tot_dist = neigh_tot_dist + current_tot_dist;
      const uint64 next_label = m_labels_assoc[next];

      if (next_tot_dist > 1.0)
        continue;

      if (next_label != 0 && m_last_generated_tot_dists[next] <= next_tot_dist)
        continue;

      if (next_label != 0)
        touched_labels[next_label - 1] = true;
      touched_labels[current_label - 1] = true;

      m_last_generated_tot_dists[next] = next_tot_dist;
      m_labels_assoc[next] = current_label;
      touched_points.insert(next);

      if (!in_queue[next])
      {
        in_queue[next] = true;
        queue.push(next);
      }
    }
  }
}

void RVizCloudAnnotationPointsPointPlane::SetSeed(const uint64 point_id,const uint32 label_id)
{
  m_labels_assoc[point_id] = label_id;
  m_last_generated_tot_dists[point_id] = 0.0;
}
