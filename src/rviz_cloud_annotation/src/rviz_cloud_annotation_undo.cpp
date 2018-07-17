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

#include "rviz_cloud_annotation_undo.h"

#include <sstream>

typedef RVizCloudAnnotationUndo::SetControlPointAction SetControlPointAction;
typedef RVizCloudAnnotationUndo::Action Action;
typedef RVizCloudAnnotationUndo::SetNameForLabelAction SetNameForLabelAction;
typedef RVizCloudAnnotationUndo::ClearLabelAction ClearLabelAction;
typedef RVizCloudAnnotationUndo::RestoreLabelAction RestoreLabelAction;
typedef RVizCloudAnnotationUndo::ClearAction ClearAction;
typedef RVizCloudAnnotationUndo::RestoreAction RestoreAction;
typedef RVizCloudAnnotationUndo::SetControlPointVectorAction SetControlPointVectorAction;

#define QUEUE_SIZE_SANITY (10000)

RVizCloudAnnotationUndo::RVizCloudAnnotationUndo()
{
  m_undone_count = 0;
}

void RVizCloudAnnotationUndo::SetAnnotation(RVizCloudAnnotationPoints::Ptr annotation)
{
  m_annotation = annotation;
  Reset();
}

void RVizCloudAnnotationUndo::Reset()
{
  m_actions.clear();
  m_undone_count = 0;
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::SetControlPoint(const uint64 idx,
                                                                               const uint32 weight_step,
                                                                               const uint32 next_label)
{
  const ControlPointData prev_cp = m_annotation->GetControlPointForPoint(idx);
  if ((prev_cp.label_id == next_label) && (prev_cp.weight_step_id == weight_step))
    return Uint64Vector(); // nothing to do

  const Action::Ptr action(new SetControlPointAction(idx,prev_cp.label_id,prev_cp.weight_step_id,next_label,weight_step));
  PushAction(action);
  return action->Execute(*m_annotation);
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::SetControlPointVector(const Uint64Vector ids,
                                                                                     const uint32 weight_step,
                                                                                     const uint32 next_label)
{
  const Uint32Vector weight_steps(ids.size(),weight_step);
  const Uint64Vector next_labels(ids.size(),next_label);

  Uint32Vector prev_weight_steps(ids.size(),0);
  Uint64Vector prev_labels(ids.size(),0);

  for (uint64 i = 0; i < ids.size(); i++)
  {
    const ControlPointData prev_cp = m_annotation->GetControlPointForPoint(ids[i]);
    prev_weight_steps[i] = prev_cp.weight_step_id;
    prev_labels[i] = prev_cp.label_id;
  }

  const Action::Ptr action(new SetControlPointVectorAction(ids,prev_labels,prev_weight_steps,next_labels,weight_steps));
  PushAction(action);
  return action->Execute(*m_annotation);
}

void RVizCloudAnnotationUndo::PushAction(Action::Ptr action)
{
  for ( ; m_undone_count > 0; m_undone_count--)
    m_actions.pop_front(); // bring m_undone_count to 0 by popping all actions

  m_actions.push_front(action);
  while (m_actions.size() > QUEUE_SIZE_SANITY)
    m_actions.pop_back();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Undo()
{
  if (m_actions.size() == 0)
    return Uint64Vector(); // nothing to undo

  if (m_actions.size() == m_undone_count)
    return Uint64Vector(); // everything is undone

  const Action::Ptr to_be_undone = m_actions[m_undone_count];
  m_undone_count++;
  return to_be_undone->Inverse()->Execute(*m_annotation);
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Redo()
{
  if (m_undone_count == 0)
    return Uint64Vector(); // everything is done

  m_undone_count--;
  const Action::Ptr to_be_done = m_actions[m_undone_count];
  return to_be_done->Execute(*m_annotation);
}

bool RVizCloudAnnotationUndo::IsUndoEnabled() const
{
  return m_undone_count < m_actions.size();
}

bool RVizCloudAnnotationUndo::IsRedoEnabled() const
{
  return m_undone_count > 0;
}

std::string RVizCloudAnnotationUndo::GetUndoDescription() const
{
  if (!IsUndoEnabled())
    return "";
  return m_actions[m_undone_count]->GetDescription();
}

std::string RVizCloudAnnotationUndo::GetRedoDescription() const
{
  if (!IsRedoEnabled())
    return "";
  return m_actions[m_undone_count - 1]->GetDescription();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::Clear()
{
  ControlPointDataVector prev_control_points;
  StringVector names;
  const uint64 max_label = m_annotation->GetMaxLabel();
  names.resize(max_label);
  for (uint64 i = 0; i < max_label; i++)
  {
    names[i] = m_annotation->GetNameForLabel(i + 1);

    const ControlPointDataVector cp = m_annotation->GetControlPointList(i + 1);
    prev_control_points.insert(prev_control_points.end(),cp.begin(),cp.end());
  }

  const Action::Ptr action(new ClearAction(prev_control_points,names));
  PushAction(action);
  return m_annotation->Clear();
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::ClearLabel(const uint32 label)
{
  const ControlPointDataVector prev_control_points = m_annotation->GetControlPointList(label);
  if (prev_control_points.empty())
    return Uint64Vector();
  const Action::Ptr action(new ClearLabelAction(label,prev_control_points));
  PushAction(action);
  return m_annotation->ClearLabel(label);
}

RVizCloudAnnotationUndo::Uint64Vector RVizCloudAnnotationUndo::SetNameForLabel(const uint32 label,const std::string & new_name)
{
  const std::string prev_name = m_annotation->GetNameForLabel(label);
  if (prev_name == new_name)
    return Uint64Vector(); // nothing to do

  const Action::Ptr action(new SetNameForLabelAction(label,prev_name,new_name));
  PushAction(action);
  return action->Execute(*m_annotation);
}

// -----

RVizCloudAnnotationUndo::Uint64Vector SetControlPointAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.SetControlPoint(m_idx,m_next_weight_step,m_next_label);
}

Action::Ptr SetControlPointAction::Inverse() const
{
  return Action::Ptr(new SetControlPointAction(m_idx,m_next_label,m_next_weight_step,
                                               m_prev_label,m_prev_weight_step));
}

std::string SetControlPointAction::GetDescription() const
{
  std::ostringstream oss;
  if (m_next_label && !m_prev_label)
    oss << "Set " << m_next_label;
  else if (m_prev_label && !m_next_label)
    oss << "Del " << m_prev_label;
  else if (m_prev_label == m_next_label)
    oss << "Weight " << m_prev_label << " (" << m_prev_weight_step << " -> " << m_next_weight_step << ")";
  else
    oss << "Set " << m_prev_label << " -> " << m_next_label;
  return oss.str();
}

SetControlPointAction::SetControlPointAction(const uint64 idx,const uint32 prev_label,const uint32 prev_weight_step,
                                             const uint32 next_label, const uint32 next_weight_step)
{
  m_idx = idx;
  m_prev_label = prev_label;
  m_next_label = next_label;
  m_prev_weight_step = prev_weight_step;
  m_next_weight_step = next_weight_step;
}

// -----

RVizCloudAnnotationUndo::Uint64Vector SetControlPointVectorAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.SetControlPointVector(m_idxs,m_next_weight_steps,m_next_labels);
}

Action::Ptr SetControlPointVectorAction::Inverse() const
{
  return Action::Ptr(new SetControlPointVectorAction(m_idxs,m_next_labels,m_next_weight_steps,
                                                   m_prev_labels,m_prev_weight_steps));
}

std::string SetControlPointVectorAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Change pts";
  for (uint64 i = 0; i < m_idxs.size() && i < 3; i++)
    oss << " " << m_idxs[i];
  if (m_idxs.size() > 3)
    oss << "...";
  return oss.str();
}

SetControlPointVectorAction::SetControlPointVectorAction(const Uint64Vector & idxs,
                          const Uint64Vector & prev_labels,const Uint32Vector & prev_weight_steps,
                          const Uint64Vector & next_labels,const Uint32Vector & next_weight_steps)
{
  m_idxs = idxs;
  m_prev_labels = prev_labels;
  m_next_labels = next_labels;
  m_prev_weight_steps = prev_weight_steps;
  m_next_weight_steps = next_weight_steps;
}

// -----

SetNameForLabelAction::SetNameForLabelAction(const uint32 label,const std::string & prev_name,const std::string & new_name)
{
  m_label = label;
  m_prev_name = prev_name;
  m_new_name = new_name;
}

RVizCloudAnnotationUndo::Uint64Vector SetNameForLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  annotation.SetNameForLabel(m_label,m_new_name);
  Uint64Vector result;
  result.push_back(m_label);
  return result;
}

Action::Ptr SetNameForLabelAction::Inverse() const
{
  return Action::Ptr(new SetNameForLabelAction(m_label,m_new_name,m_prev_name));
}

std::string SetNameForLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Name " << m_label;
  return oss.str();
}

// -----

RVizCloudAnnotationUndo::Uint64Vector ClearLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.ClearLabel(m_label);
}

Action::Ptr ClearLabelAction::Inverse() const
{
  return Action::Ptr(new RestoreLabelAction(m_label,m_prev_control_points));
}

std::string ClearLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Clear " << m_label;
  return oss.str();
}

ClearLabelAction::ClearLabelAction(const uint32 label,const ControlPointDataVector & prev_control_points)
{
  m_label = label;
  m_prev_control_points = prev_control_points;
}

RVizCloudAnnotationUndo::Uint64Vector RestoreLabelAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.SetControlPointList(m_control_points,m_label);
}

Action::Ptr RestoreLabelAction::Inverse() const
{
  return Action::Ptr(new ClearLabelAction(m_label,m_control_points));
}

std::string RestoreLabelAction::GetDescription() const
{
  std::ostringstream oss;
  oss << "Restore " << m_label;
  return oss.str();
}

RestoreLabelAction::RestoreLabelAction(const uint32 label,const ControlPointDataVector & control_points)
{
  m_label = label;
  m_control_points = control_points;
}

// -----

RVizCloudAnnotationUndo::Uint64Vector ClearAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  return annotation.Clear();
}

Action::Ptr ClearAction::Inverse() const
{
  return Action::Ptr(new RestoreAction(m_prev_control_points,m_names));
}

std::string ClearAction::GetDescription() const
{
  return "Clear all";
}

ClearAction::ClearAction(const ControlPointDataVector & control_points,const StringVector & names)
{
  m_prev_control_points = control_points;
  m_names = names;
}

RVizCloudAnnotationUndo::Uint64Vector RestoreAction::Execute(RVizCloudAnnotationPoints & annotation) const
{
  const uint64 size = m_names.size();
  Uint64Vector all(size);
  for (uint64 i = 0; i < m_names.size(); i++)
  {
    annotation.SetNameForLabel(i + 1,m_names[i]);
    all[i] = i + 1;
  }

  annotation.SetControlPointList(m_control_points);

  return all;
}

Action::Ptr RestoreAction::Inverse() const
{
  return Action::Ptr(new ClearAction(m_control_points,m_names));
}

std::string RestoreAction::GetDescription() const
{
  return "Restore all";
}

RestoreAction::RestoreAction(const ControlPointDataVector & control_points,const StringVector & names)
{
  m_control_points = control_points;
  m_names = names;
}
