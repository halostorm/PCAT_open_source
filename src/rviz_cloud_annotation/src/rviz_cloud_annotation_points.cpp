#include "rviz_cloud_annotation_points.h"
#include "rviz_cloud_annotation.h"

#include <boost/lexical_cast.hpp>

#include <algorithm>
#include <cmath>

RVizCloudAnnotationPoints::RVizCloudAnnotationPoints(const uint64 cloud_size,
                                                     const uint32 weight_steps,
                                                     const PointNeighborhood::ConstPtr neighborhood)
{
  m_cloud_size = cloud_size;

  m_control_points_assoc.resize(m_cloud_size,0);
  m_labels_assoc.resize(m_cloud_size,0);
  m_last_generated_tot_dists.resize(m_cloud_size,0.0);

  m_weight_steps_count = weight_steps;
  m_weight_steps_planes.resize(m_weight_steps_count + 1);
  m_control_points_for_weight.resize(m_weight_steps_count + 1);

  m_point_neighborhood = neighborhood;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::Clear()
{
  Uint64Vector result;
  result.resize(m_control_points_for_label.size());
  for (uint64 i = 0; i < result.size(); i++)
    result[i] = i + 1;

  m_control_points.clear();
  m_erased_control_points.clear();
  m_ext_label_names.clear();
  m_control_points_for_label.clear();

  m_weight_steps_planes.assign(m_weight_steps_count + 1,PointPlane::Ptr());
  m_control_points_for_weight.assign(m_weight_steps_count + 1,Uint64Vector());

  m_control_points_assoc.assign(m_cloud_size,0);
  m_labels_assoc.assign(m_cloud_size,0);
  m_last_generated_tot_dists.assign(m_cloud_size,0.0);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPointVector(const Uint64Vector & ids,
                                                                                       const Uint32Vector & weight_steps,
                                                                                       const Uint64Vector & labels)
{
  for (uint64 i = 0; i < labels.size(); i++)
    ExpandLabelsUntil(labels[i]);
  BoolVector touched(GetMaxLabel() + 1,false);

  // call all the SetControlPoint s and merge the results
  for (uint64 i = 0; i < ids.size(); i++)
  {
    const uint64 point_id = ids[i];
    const uint64 label = labels[i];
    const uint32 weight_step = weight_steps[i];

    Uint64Vector touched_vec;
    touched_vec = SetControlPoint(point_id,weight_step,label);

    for (uint64 h = 0; h < touched_vec.size(); h++)
      touched[touched_vec[h]] = true;
  }

  Uint64Vector result;
  for (uint64 i = 0; i < touched.size(); i++)
    if (touched[i])
      result.push_back(i);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPoint(const uint64 point_id,
                                                                                   const uint32 weight_step,
                                                                                   const uint64 label)
{
  ExpandLabelsUntil(label);

  const uint64 prev_control_point_id = m_control_points_assoc[point_id];

  // ADDING
  if (prev_control_point_id == 0)
  {
    if (label == 0)
      return Uint64Vector(); // nothing to do

    const uint64 new_id = InternalSetControlPoint(point_id,weight_step,label);
    BoolVector touched;
    UpdateLabelAssocAdded(new_id,weight_step,touched);

    return TouchedBoolVectorToExtLabel(touched);
  }

  // DELETING
  if (prev_control_point_id != 0 && label == 0)
  {
    BoolVector touched;
    UpdateLabelAssocDeleted(prev_control_point_id,touched);
    touched[prev_control_point_id - 1] = true;
    const Uint64Vector result = TouchedBoolVectorToExtLabel(touched);

    InternalSetControlPoint(point_id,0,0);

    return result;
  }

  const ControlPoint & control_point = m_control_points[prev_control_point_id - 1];
  const uint64 prev_weight_step = control_point.weight_step_id;
  const uint64 prev_label = control_point.label_id;

  // CHANGE LABEL, SAME WEIGHT - INTERNAL ONLY
  if (prev_weight_step == weight_step && prev_label != label)
  {
    InternalSetControlPoint(point_id,weight_step,label);

    Uint64Vector result;
    result.push_back(prev_label);
    result.push_back(label);
    return result;
  }

  // CHANGE WEIGHT: delete and re-create
  if (prev_weight_step != weight_step)
  {
    BoolVector touched;
    UpdateLabelAssocDeleted(prev_control_point_id,touched);
    touched[prev_control_point_id - 1] = true;
    const Uint64Vector result1 = TouchedBoolVectorToExtLabel(touched);

    const uint64 new_id = InternalSetControlPoint(point_id,weight_step,label);

    UpdateLabelAssocAdded(new_id,weight_step,touched);
    const Uint64Vector result2 = TouchedBoolVectorToExtLabel(touched);

    Uint64Set result_set(result1.begin(),result1.end());
    result_set.insert(result2.begin(),result2.end());
    return Uint64Vector(result_set.begin(),result_set.end());
  }

  return Uint64Vector();
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPointList(
  const CPDataVector & control_points_data,const uint64 label)
{
  CPDataVector cpdatas = control_points_data;
  for (uint64 i = 0; i < cpdatas.size(); i++)
    cpdatas[i].label_id = label;
  return SetControlPointList(cpdatas);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetControlPointList(const CPDataVector & control_points_data)
{
  Uint64Set touched;

  const uint64 size = control_points_data.size();
  for (uint64 i = 0; i < size; i++)
  {
    const CPData & data = control_points_data[i];
    const Uint64Vector local_touched = SetControlPoint(data.point_id,data.weight_step_id,data.label_id);
    touched.insert(local_touched.begin(),local_touched.end());
  }

  return Uint64Vector(touched.begin(),touched.end());
}

RVizCloudAnnotationPoints::CPDataVector RVizCloudAnnotationPoints::GetControlPointList(const uint64 label) const
{
  if (label > GetMaxLabel())
    return CPDataVector();

  const Uint64Vector & control_point_ids = m_control_points_for_label[label - 1];
  CPDataVector result;
  result.reserve(control_point_ids.size());
  for (uint64 i = 0; i < control_point_ids.size(); i++)
    result.push_back(m_control_points[control_point_ids[i] - 1].ToCPData());

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::GetLabelPointList(const uint64 label) const
{
  Uint64Vector result;

  if (label > GetMaxLabel())
    return result;

  if (label == 0)
  {
    for (uint64 i = 0; i < m_cloud_size; i++)
      if (m_labels_assoc[i] == 0)
        result.push_back(i);
    return result;
  }

  for (uint64 i = 0; i < m_cloud_size; i++)
    if (m_labels_assoc[i] && m_control_points[m_labels_assoc[i] - 1].label_id == label)
      result.push_back(i);

  return result;
}

void RVizCloudAnnotationPoints::UpdateMainWeightPlane(const Uint64Set & touched_points,
                                                      BoolVector & touched)
{
  touched.resize(m_control_points.size(),false);

  for (Uint64Set::const_iterator iter = touched_points.begin(); iter != touched_points.end(); iter++)
  {
    const uint64 i = *iter;

    m_last_generated_tot_dists[i] = 0.0;
    m_labels_assoc[i] = 0;

    for (uint64 h = 0; h <= m_weight_steps_count; h++)
    {
      if (!m_weight_steps_planes[h])
        continue;

      const PointPlane & plane = *(m_weight_steps_planes[h]);
      const uint32 label_id = plane.GetLabel(i);
      const float weight = plane.GetTotDist(i);
      const uint32 prev_label = m_labels_assoc[i];
      const float prev_weight = m_last_generated_tot_dists[i];
      if (label_id != 0 && (weight < prev_weight || prev_label == 0))
      {
        if (m_labels_assoc[i])
          touched[m_labels_assoc[i] - 1] = true;
        if (label_id)
          touched[label_id - 1] = true;
        m_last_generated_tot_dists[i] = weight;
        m_labels_assoc[i] = label_id;
      }
    }
  }
}

void RVizCloudAnnotationPoints::RebuildMainWeightPlane(BoolVector & touched)
{
  m_labels_assoc.assign(m_cloud_size,0);
  m_last_generated_tot_dists.assign(m_cloud_size,0.0);
  touched.assign(m_control_points.size(),false);

  for (uint64 i = 0; i < m_cloud_size; i++)
    for (uint64 h = 0; h <= m_weight_steps_count; h++)
    {
      if (!m_weight_steps_planes[h])
        continue;

      const PointPlane & plane = *(m_weight_steps_planes[h]);
      const uint32 label_id = plane.GetLabel(i);
      const float weight = plane.GetTotDist(i);
      const uint32 prev_label = m_labels_assoc[i];
      const float prev_weight = m_last_generated_tot_dists[i];
      if (label_id != 0 && (weight < prev_weight || prev_label == 0))
      {
        if (m_labels_assoc[i])
          touched[m_labels_assoc[i] - 1] = true;
        if (label_id)
          touched[label_id - 1] = true;
        m_last_generated_tot_dists[i] = weight;
        m_labels_assoc[i] = label_id;
      }
    }
}

void RVizCloudAnnotationPoints::RegenerateLabelAssoc(BoolVector & touched)
{
  touched.assign(m_control_points.size(),false);

  Uint64Set touched_points;

  for (uint64 wi = 0; wi <= m_weight_steps_count; wi++)
  {
    if (m_control_points_for_weight[wi].empty() && !m_weight_steps_planes[wi])
      continue; // do not create this plane

    ExpandPointPlane(wi);
    PointPlane & plane = *(m_weight_steps_planes[wi]);
    plane.Clear();

    Uint64Vector seeds;
    const Uint64Vector & control_points = m_control_points_for_weight[wi];
    for (uint64 i = 0; i < control_points.size(); i++)
    {
      const uint64 cp_i = control_points[i];
      const ControlPoint & first = m_control_points[cp_i - 1];
      seeds.push_back(first.point_id);
      plane.SetSeed(first.point_id,cp_i);
    }

    plane.UpdateRegionGrowing(seeds,touched,touched_points);
  }

  RebuildMainWeightPlane(touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocAddedPlane(const uint64 added_index,
                                                           const uint32 added_weight,
                                                           BoolVector & touched,
                                                           Uint64Set & touched_points)
{
  touched.assign(m_control_points.size(),false);
  Uint64Vector seeds;

  touched[added_index - 1] = true; // we are adding it
  const uint64 point_id = m_control_points[added_index - 1].point_id;
  seeds.push_back(point_id);

  ExpandPointPlane(added_weight);

  PointPlane & plane = *(m_weight_steps_planes[added_weight]);
  plane.SetSeed(point_id,added_index);
  plane.UpdateRegionGrowing(seeds,touched,touched_points);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocAdded(const uint64 added_index,
                                                      const uint32 added_weight,
                                                      BoolVector & touched)
{
  Uint64Set touched_points;
  UpdateLabelAssocAddedPlane(added_index,added_weight,touched,touched_points);
  UpdateMainWeightPlane(touched_points,touched);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocDeleted(const uint64 removed_index,BoolVector & touched_labels)
{
  Uint64Vector removed_indices;
  removed_indices.push_back(removed_index);
  UpdateLabelAssocDeletedVector(removed_indices,touched_labels);
}

void RVizCloudAnnotationPoints::UpdateLabelAssocDeletedVector(const Uint64Vector & removed_indices,BoolVector & touched_labels)
{
  touched_labels.assign(m_control_points.size(),false);
  Uint64Set touched_points;

  for (uint64 i = 0; i < removed_indices.size(); i++)
  {
    const uint64 removed_index = removed_indices[i];
    touched_labels[removed_index - 1] = true; // we are removing it

    const uint32 removed_weight = m_control_points[removed_index - 1].weight_step_id;
    if (!m_weight_steps_planes[removed_weight])
      return;
    const uint64 point_id = m_control_points[removed_index - 1].point_id;

    PointPlane & plane = *(m_weight_steps_planes[removed_weight]);

    plane.RemoveLabel(removed_index,point_id,touched_labels,touched_points);
  }

  UpdateMainWeightPlane(touched_points,touched_labels);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::ClearLabel(const uint64 label)
{
  if (label == 0)
    return Uint64Vector();
  if (label > GetMaxLabel())
    return Uint64Vector();

  const Uint64Vector control_points_ids = m_control_points_for_label[label - 1];
  if (control_points_ids.empty())
    return Uint64Vector();

  BoolVector touched;
  UpdateLabelAssocDeletedVector(control_points_ids,touched);
  const Uint64Vector result = TouchedBoolVectorToExtLabel(touched);

  for (uint64 i = 0; i < control_points_ids.size(); i++)
    InternalSetControlPoint(m_control_points[control_points_ids[i] - 1].point_id,0,0);

  return result;
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::SetNameForLabel(const uint64 label,const std::string & name)
{
  ExpandLabelsUntil(label);
  m_ext_label_names[label - 1] = name;
  return Uint64Vector(1,label);
}

void RVizCloudAnnotationPoints::ExpandLabelsUntil(const uint64 label)
{
  if (label <= GetMaxLabel())
    return;

  m_ext_label_names.resize(label);
  m_control_points_for_label.resize(label);
}

void RVizCloudAnnotationPoints::ExpandPointPlane(const uint32 weight_id)
{
  PointPlane::Ptr & plane_ptr = m_weight_steps_planes[weight_id];
  if (!plane_ptr)
  {
    const float multiplier = (weight_id ? (float(m_weight_steps_count) / float(weight_id)) : NAN);
    plane_ptr = PointPlane::Ptr(new PointPlane(m_cloud_size,*m_point_neighborhood,multiplier));
  }
}

RVizCloudAnnotationPoints::uint64 RVizCloudAnnotationPoints::InternalSetControlPoint(const uint64 point_id,
                                                                                     const uint32 weight_step,
                                                                                     const uint32 label)
{
  ExpandLabelsUntil(label);

  const uint64 prev_control_point_id = m_control_points_assoc[point_id];
  if (prev_control_point_id != 0)
  {
    const uint32 prev_label = m_control_points[prev_control_point_id - 1].label_id;
    const uint32 prev_weight = m_control_points[prev_control_point_id - 1].weight_step_id;

    if (label == 0)
    {
      // remove control point
      m_control_points_assoc[point_id] = 0;
      m_control_points[prev_control_point_id - 1].Invalidate();
      m_erased_control_points.push_back(prev_control_point_id);
      EraseFromVector(m_control_points_for_label[prev_label - 1],prev_control_point_id);
      EraseFromVector(m_control_points_for_weight[prev_weight],prev_control_point_id);

      while (!m_control_points.empty() && m_control_points.back().Invalid())
      {
        EraseFromVector(m_erased_control_points,uint64(m_control_points.size()));
        m_control_points.pop_back();
      }

      return 0;
    }

    if (label == prev_label)
    {
      if (prev_weight != weight_step)
      {
        m_control_points[prev_control_point_id - 1].weight_step_id = weight_step;
        EraseFromVector(m_control_points_for_weight[prev_weight],prev_control_point_id);
        m_control_points_for_weight[weight_step].push_back(prev_control_point_id);
      }

      return prev_control_point_id;
    }

    // change label of control point
    EraseFromVector(m_control_points_for_label[prev_label - 1],prev_control_point_id);
    m_control_points[prev_control_point_id - 1].label_id = label;
    m_control_points_for_label[label - 1].push_back(prev_control_point_id);

    if (prev_weight != weight_step)
    {
      m_control_points[prev_control_point_id - 1].weight_step_id = weight_step;
      EraseFromVector(m_control_points_for_weight[prev_weight],prev_control_point_id);
      m_control_points_for_weight[weight_step].push_back(prev_control_point_id);
    }

    return prev_control_point_id;
  }

  // ok, create new control point
  uint64 new_id;
  if (m_erased_control_points.empty())
  {
    m_control_points.push_back(ControlPoint(point_id,weight_step,label));
    new_id = m_control_points.size();
  }
  else // if an erased control point is present, reuse it
  {
    new_id = m_erased_control_points.back();
    m_control_points[new_id - 1] = ControlPoint(point_id,weight_step,label);
    m_erased_control_points.pop_back();
  }
  m_control_points_for_label[label - 1].push_back(new_id);
  m_control_points_for_weight[weight_step].push_back(new_id);
  m_control_points_assoc[point_id] = new_id;

  return new_id;
}

template <typename T>
  void RVizCloudAnnotationPoints::EraseFromVector(std::vector<T> & vector,const T value)
{
  typename std::vector<T>::iterator iter = std::find(vector.begin(),vector.end(),value);
  if (iter != vector.end())
    vector.erase(iter);
}

RVizCloudAnnotationPoints::Uint64Vector RVizCloudAnnotationPoints::TouchedBoolVectorToExtLabel(const BoolVector & touched) const
{
  BoolVector touched_labels(m_control_points_for_label.size(),false);
  Uint64Vector result;

  for (uint64 i = 0; i < touched.size(); i++)
    if (touched[i])
      touched_labels[m_control_points[i].label_id - 1] = true;

  for (uint64 i = 0; i < touched_labels.size(); i++)
    if (touched_labels[i])
      result.push_back(i + 1);

  return result;
}
