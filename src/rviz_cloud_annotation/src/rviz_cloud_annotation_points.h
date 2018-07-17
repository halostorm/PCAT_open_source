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

#ifndef RVIZ_CLOUD_ANNOTATION_POINTS_H
#define RVIZ_CLOUD_ANNOTATION_POINTS_H

// STL
#include <stdint.h>
#include <vector>
#include <string>
#include <istream>
#include <ostream>
#include <queue>
#include <set>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/common/colors.h>
#include <pcl/point_types.h>

// boost
#include <boost/shared_ptr.hpp>

#include "point_neighborhood.h"
#include "rviz_cloud_annotation_point_plane.h"

class RVizCloudAnnotationPoints
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef uint8_t uint8;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint8> Uint8Vector;
  typedef std::vector<float> FloatVector;
  typedef std::queue<uint64> Uint64Queue;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<std::string> StringVector;
  typedef std::set<uint64> Uint64Set;
  struct CPData
  {
    uint64 point_id;
    uint32 weight_step_id;
    uint32 label_id;

    CPData(uint64 pi,uint32 wsi,uint32 li): point_id(pi), weight_step_id(wsi), label_id(li) {}
    CPData(): point_id(0), weight_step_id(0), label_id(0) {}
  };
  typedef std::vector<CPData> CPDataVector;
  typedef RVizCloudAnnotationPointsPointPlane PointPlane;
  typedef std::vector<PointPlane::Ptr> PointPlanePtrVector;

  typedef boost::shared_ptr<const RVizCloudAnnotationPoints> ConstPtr;
  typedef boost::shared_ptr<RVizCloudAnnotationPoints> Ptr;

  explicit RVizCloudAnnotationPoints(const uint64 cloud_size,
                                     const uint32 weight_steps,
                                     const PointNeighborhood::ConstPtr neighborhood);

  struct IOE // IO exception
  {
    IOE(const std::string & d): description(d) {}
    std::string description;
  };

  static RVizCloudAnnotationPoints::Ptr Deserialize(std::istream & ifile,
    const uint32 weight_steps,
    PointNeighborhood::ConstPtr neighborhood);
  void Serialize(std::ostream & ofile) const;

  // returns the list of affected labels
  Uint64Vector SetControlPoint(const uint64 point_id,const uint32 weight_step,const uint64 label);
  Uint64Vector SetControlPointVector(const Uint64Vector & ids,
                                   const Uint32Vector & weight_steps,
                                   const Uint64Vector & labels);
  Uint64Vector SetControlPoint(const CPData & control_point_data);
  Uint64Vector SetControlPointList(const CPDataVector & control_points_data,const uint64 label);
  Uint64Vector SetControlPointList(const CPDataVector & control_points_data);

  Uint64Vector Clear();
  Uint64Vector ClearLabel(const uint64 label); // clear label, returns list of affected labels
  Uint64Vector SetNameForLabel(const uint64 label,const std::string & name);

  CPDataVector GetControlPointList(const uint64 label) const;
  Uint64Vector GetLabelPointList(const uint64 label) const;

  uint64 GetWeightStepsCount() const {return m_weight_steps_count; }

  // 0 if none
  uint64 GetLabelForPoint(const uint64 idx) const
  {
    if (!m_labels_assoc[idx])
      return 0;
    return m_control_points[m_labels_assoc[idx] - 1].label_id;
  }

  CPData GetControlPointForPoint(const uint64 idx) const
  {
    if (!m_control_points_assoc[idx])
      return CPData();
    return m_control_points[m_control_points_assoc[idx] - 1].ToCPData();
  }

  std::string GetNameForLabel(const uint64 label) const
  {
    if (label > GetMaxLabel())
      return "";
    return m_ext_label_names[label - 1];
  }

  uint64 GetNextLabel() const {return m_control_points_for_label.size() + 1; }
  uint64 GetMaxLabel() const {return m_control_points_for_label.size(); }
  uint64 GetCloudSize() const {return m_cloud_size; }

  uint64 GetLabelPointCount(const uint64 label) const
  {
    if (label > GetMaxLabel())
      return 0;
    return m_control_points_for_label[label - 1].size();
  }

  template <class PointT>
    void LabelCloud(pcl::PointCloud<PointT> & cloud) const;
  template <class PointT>
    void LabelCloudWithColor(pcl::PointCloud<PointT> & cloud) const;

  private:
  struct ControlPoint
  {
    uint32 label_id;
    uint32 weight_step_id;
    uint64 point_id;

    ControlPoint(const uint64 point_id,const uint32 weight_step,const uint32 label_id):
      label_id(label_id), weight_step_id(weight_step), point_id(point_id) {}
    void Invalidate() {label_id = 0; }
    bool Valid() const {return label_id; }
    bool Invalid() const {return !label_id; }

    CPData ToCPData() const {return CPData(point_id,weight_step_id,label_id); }
  };
  typedef std::vector<ControlPoint> ControlPointVector;

  void ExpandLabelsUntil(const uint64 label);
  void ExpandPointPlane(const uint32 weight_id);

  void RegenerateLabelAssoc(BoolVector & touched);
  void UpdateLabelAssocAdded(const uint64 added_index,
                             const uint32 added_weight,
                             BoolVector & touched);
  void UpdateLabelAssocAddedPlane(const uint64 added_index,
                                  const uint32 added_weight,
                                  BoolVector & touched,
                                  Uint64Set & touched_plane);
  void UpdateLabelAssocDeleted(const uint64 removed_index,BoolVector & touched_labels);
  void UpdateLabelAssocDeletedVector(const Uint64Vector & removed_indices,BoolVector & touched);

  void UpdateMainWeightPlane(const Uint64Set & touched_points,BoolVector & touched);
  void RebuildMainWeightPlane(BoolVector & touched);

  uint64 InternalSetControlPoint(const uint64 point_id,
                                 const uint32 weight_step,
                                 const uint32 label);
  template <typename T>
    static void EraseFromVector(std::vector<T> & vector,const T value);
  Uint64Vector TouchedBoolVectorToExtLabel(const BoolVector & touched) const;

  // assoc from cloud point to control point, 0 otherwise
  Uint64Vector m_control_points_assoc;
  // assoc from cloud point to region grown control point, 0 otherwise
  Uint64Vector m_labels_assoc;
  // control points
  ControlPointVector m_control_points;
  Uint64Vector m_erased_control_points;

  // from external label to list of control points with that label
  Uint64VectorVector m_control_points_for_label;
  Uint64VectorVector m_control_points_for_weight;

  uint64 m_cloud_size;

  FloatVector m_last_generated_tot_dists;

  StringVector m_ext_label_names;

  PointPlanePtrVector m_weight_steps_planes;
  uint32 m_weight_steps_count;

  PointNeighborhood::ConstPtr m_point_neighborhood;
};

template <class PointT>
  void RVizCloudAnnotationPoints::LabelCloud(pcl::PointCloud<PointT> & cloud) const
{
  for (uint64 i = 0; i < m_cloud_size; i++)
    cloud[i].label = (m_labels_assoc[i] ? (m_control_points[m_labels_assoc[i] - 1].label_id) : 0);
}

template <class PointT>
  void RVizCloudAnnotationPoints::LabelCloudWithColor(pcl::PointCloud<PointT> & cloud) const
{
  LabelCloud(cloud);
  for (uint64 i = 0; i < m_cloud_size; i++)
  {
    const uint32 label = cloud[i].label;
    if (!label)
    {
      cloud[i].r = 0;
      cloud[i].g = 0;
      cloud[i].b = 0;
      continue;
    }
    const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);
    cloud[i].r = color.r;
    cloud[i].g = color.g;
    cloud[i].b = color.b;
  }
}

#endif // RVIZ_CLOUD_ANNOTATION_POINTS_H
