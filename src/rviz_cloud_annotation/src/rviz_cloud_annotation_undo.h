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

#ifndef RVIZ_CLOUD_ANNOTATION_UNDO_H
#define RVIZ_CLOUD_ANNOTATION_UNDO_H

#include "rviz_cloud_annotation_points.h"

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// STL
#include <deque>
#include <vector>
#include <stdint.h>
#include <string>

class RVizCloudAnnotationUndo
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<std::string> StringVector;
  typedef RVizCloudAnnotationPoints::CPData ControlPointData;
  typedef RVizCloudAnnotationPoints::CPDataVector ControlPointDataVector;

  RVizCloudAnnotationUndo();

  // interface
  class Action
  {
    public:
    typedef boost::shared_ptr<Action> Ptr;
    typedef boost::shared_ptr<const Action> ConstPtr;

    virtual ~Action() {}

    // returns a list of affected markers
    virtual Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const = 0;
    virtual Ptr Inverse() const = 0; // get the inverse action from the current one
    virtual std::string GetDescription() const = 0;
  };

  typedef std::deque<Action::Ptr> ActionPtrDeque;

  class SetControlPointAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    SetControlPointAction(const uint64 idx,const uint32 prev_label,const uint32 prev_weight_step,
                          const uint32 next_label,const uint32 next_weight_step);

    private:
    uint64 m_idx;
    uint32 m_prev_label;
    uint32 m_next_label;
    uint32 m_prev_weight_step;
    uint32 m_next_weight_step;
  };

  class SetControlPointVectorAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    SetControlPointVectorAction(const Uint64Vector & idxs,
                              const Uint64Vector & prev_labels,const Uint32Vector & prev_weight_steps,
                              const Uint64Vector & next_labels,const Uint32Vector & next_weight_steps);

    private:
    Uint64Vector m_idxs;
    Uint64Vector m_prev_labels;
    Uint32Vector m_prev_weight_steps;
    Uint64Vector m_next_labels;
    Uint32Vector m_next_weight_steps;
  };

  class SetNameForLabelAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    SetNameForLabelAction(const uint32 label,const std::string & prev_name,const std::string & new_name);

    private:
    std::string m_new_name;
    std::string m_prev_name;
    uint32 m_label;
  };

  class ClearLabelAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    ClearLabelAction(const uint32 label,const ControlPointDataVector & prev_control_points);

    private:
    ControlPointDataVector m_prev_control_points;
    uint32 m_label;
  };

  class RestoreLabelAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    RestoreLabelAction(const uint32 label,const ControlPointDataVector & control_points);

    private:
    ControlPointDataVector m_control_points;
    uint32 m_label;
  };

  class ClearAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    ClearAction(const ControlPointDataVector & control_points,const StringVector & names);

    private:
    ControlPointDataVector m_prev_control_points;
    StringVector m_names;
  };

  class RestoreAction: public Action
  {
    public:
    Uint64Vector Execute(RVizCloudAnnotationPoints & annotation) const;
    Action::Ptr Inverse() const;
    std::string GetDescription() const;

    RestoreAction(const ControlPointDataVector & control_points,const StringVector & names);

    private:
    ControlPointDataVector m_control_points;
    StringVector m_names;
  };

  Uint64Vector SetControlPoint(const uint64 idx,const uint32 weight_step,const uint32 next_label);
  Uint64Vector SetControlPointVector(const Uint64Vector ids,const uint32 weight_step,const uint32 next_label);
  Uint64Vector Clear();
  Uint64Vector ClearLabel(const uint32 label);
  Uint64Vector SetNameForLabel(const uint32 label,const std::string & name);

  Uint64Vector Undo();
  Uint64Vector Redo();

  bool IsUndoEnabled() const;
  bool IsRedoEnabled() const;
  std::string GetUndoDescription() const;
  std::string GetRedoDescription() const;

  RVizCloudAnnotationPoints::Ptr GetAnnotation() const {return m_annotation; }

  void SetAnnotation(RVizCloudAnnotationPoints::Ptr annotation);
  void Reset();

  private:
  void PushAction(Action::Ptr action);

  RVizCloudAnnotationPoints::Ptr m_annotation;

  ActionPtrDeque m_actions;
  uint64 m_undone_count;
};

#endif // RVIZ_CLOUD_ANNOTATION_UNDO_H
