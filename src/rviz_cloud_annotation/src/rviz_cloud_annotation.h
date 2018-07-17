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

#ifndef RVIZ_CLOUD_ANNOTATION_H
#define RVIZ_CLOUD_ANNOTATION_H

#define PARAM_NAME_UPDATE_TOPIC "update_topic"
#define PARAM_DEFAULT_UPDATE_TOPIC "/rviz_cloud_annotation/update_topic"

#define PARAM_NAME_NORMAL_SOURCE "normal_source"
#define PARAM_VALUE_NORMAL_SOURCE_CLOUD "cloud"               // from PARAM_NAME_CLOUD_FILENAME itself
#define PARAM_VALUE_NORMAL_SOURCE_OTHER_CLOUD "other_cloud:"  // example: "other_cloud:normal_cloud.pcd"
#define PARAM_DEFAULT_NORMAL_SOURCE PARAM_VALUE_NORMAL_SOURCE_CLOUD

#define PARAM_NAME_AUTOSAVE_TIME "autosave_time"
#define PARAM_DEFAULT_AUTOSAVE_TIME (0.0)  // seconds, < 1.0 to disable

#define PARAM_NAME_AUTOSAVE_TIMESTAMP "autosave_append_timestamp"
#define PARAM_DEFAULT_AUTOSAVE_TIMESTAMP (false)

#define PARAM_NAME_FRAME_ID "frame_id"
#define PARAM_DEFAULT_FRAME_ID "base_link"

#define PARAM_NAME_POINT_SIZE "point_size"
#define PARAM_DEFAULT_POINT_SIZE (0.05)

// size of the current label for each point
#define PARAM_NAME_LABEL_SIZE "label_size"
#define PARAM_DEFAULT_LABEL_SIZE (0.1)

// size of the current label for control points
#define PARAM_NAME_CONTROL_LABEL_SIZE "control_label_size"
#define PARAM_DEFAULT_CONTROL_LABEL_SIZE (0.1)

// shows labels in the back of the points as well
#define PARAM_NAME_SHOW_POINTS_BACK_LABELS "show_labels_back"
#define PARAM_DEFAULT_SHOW_POINTS_BACK_LABELS (true)

#define PARAM_NAME_CONTROL_POINT_VISUAL "control_point_visual"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_LINE "line"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE "sphere"
#define PARAM_VALUE_CONTROL_POINT_VISUAL_THREE_SPHERES "three_spheres"
#define PARAM_DEFAULT_CONTROL_POINT_VISUAL PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE

// [0..1] 0: size not affected by weight - 1: whole size affected
#define PARAM_NAME_CP_WEIGHT_SCALE_FRACTION "control_point_weight_scale_fraction"
#define PARAM_DEFAULT_CP_WEIGHT_SCALE_FRACTION (0.5)

#define PARAM_NAME_ZERO_WEIGHT_CP_SHOW "show_zero_weight_control_points"
#define PARAM_DEFAULT_ZERO_WEIGHT_CP_SHOW (true)

// from interface to backend
#define PARAM_NAME_SET_EDIT_MODE_TOPIC "rviz_cloud_annotation/set_edit_mode_topic"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC "/rviz_cloud_annotation/set_edit_mode"

// from backend to interface
#define PARAM_NAME_SET_EDIT_MODE_TOPIC2 "rviz_cloud_annotation/set_edit_mode_topic2"
#define PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2 "/rviz_cloud_annotation/set_edit_mode2"

#define PARAM_NAME_COLORS_COLS_PER_PAGE "rviz_cloud_annotation/color_columns_per_page"
#define PARAM_DEFAULT_COLOR_COLS_PER_PAGE (10)

#define PARAM_NAME_COLORS_ROWS_PER_PAGE "rviz_cloud_annotation/color_rows_per_page"
#define PARAM_DEFAULT_COLOR_ROWS_PER_PAGE (2)

#define PARAM_NAME_POINT_SIZE_CHANGE_MULT "rviz_cloud_annotation/point_change_size_multiplier"
#define PARAM_DEFAULT_POINT_SIZE_CHANGE_MULT (0.2)

#define PARAM_NAME_SET_CURRENT_LABEL_TOPIC "rviz_cloud_annotation/set_current_label_topic"
#define PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC "/rviz_cloud_annotation/set_current_label"

#define PARAM_NAME_CURRENT_LABEL_TOPIC "rviz_cloud_annotation/set_current_label_topic2"
#define PARAM_DEFAULT_CURRENT_LABEL_TOPIC "/rviz_cloud_annotation/set_current_label2"

#define PARAM_NAME_SAVE_TOPIC "rviz_cloud_annotation/save_topic"
#define PARAM_DEFAULT_SAVE_TOPIC "/rviz_cloud_annotation/save"

#define PARAM_NAME_RESTORE_TOPIC "rviz_cloud_annotation/restore_topic"
#define PARAM_DEFAULT_RESTORE_TOPIC "/rviz_cloud_annotation/restore"

#define PARAM_NAME_CLEAR_TOPIC "rviz_cloud_annotation/clear_topic"
#define PARAM_DEFAULT_CLEAR_TOPIC "/rviz_cloud_annotation/clear"

#define PARAM_NAME_NEW_TOPIC "rviz_cloud_annotation/new_topic"
#define PARAM_DEFAULT_NEW_TOPIC "/rviz_cloud_annotation/new"

// from RViz to backend
#define PARAM_NAME_SET_NAME_TOPIC "rviz_cloud_annotation/set_name_topic"
#define PARAM_DEFAULT_SET_NAME_TOPIC "/rviz_cloud_annotation/set_name"

// from backend to RViz
#define PARAM_NAME_SET_NAME_TOPIC2 "rviz_cloud_annotation/set_name_topic2"
#define PARAM_DEFAULT_SET_NAME_TOPIC2 "/rviz_cloud_annotation/set_name2"

// from backend to RViz
#define PARAM_NAME_POINT_COUNT_UPDATE_TOPIC "rviz_cloud_annotation/point_count_update"
#define PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC "/rviz_cloud_annotation/point_count_update"

#define PARAM_NAME_VIEW_LABEL_TOPIC "rviz_cloud_annotation/view_labels_topic"
#define PARAM_DEFAULT_VIEW_LABEL_TOPIC "/rviz_cloud_annotation/view_labels"

#define PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC "rviz_cloud_annotation/view_control_points_topic"
#define PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC "/rviz_cloud_annotation/view_control_points"

#define PARAM_NAME_VIEW_CLOUD_TOPIC "rviz_cloud_annotation/view_cloud_topic"
#define PARAM_DEFAULT_VIEW_CLOUD_TOPIC "/rviz_cloud_annotation/view_cloud"

#define PARAM_NAME_UNDO_TOPIC "rviz_cloud_annotation/undo_topic"
#define PARAM_DEFAULT_UNDO_TOPIC "/rviz_cloud_annotation/undo"

#define PARAM_NAME_REDO_TOPIC "rviz_cloud_annotation/redo_topic"
#define PARAM_DEFAULT_REDO_TOPIC "/rviz_cloud_annotation/redo"

#define PARAM_NAME_NEXT_TOPIC "rviz_cloud_annotation/next_topic"
#define PARAM_DEFAULT_NEXT_TOPIC "/rviz_cloud_annotation/next"

#define PARAM_NAME_PRE_TOPIC "rviz_cloud_annotation/pre_topic"
#define PARAM_DEFAULT_PRE_TOPIC "/rviz_cloud_annotation/pre"

#define PARAM_NAME_UNDO_REDO_STATE_TOPIC "rviz_cloud_annotation/undo_redo_state_topic"
#define PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC "/rviz_cloud_annotation/undo_redo_state"

#define PARAM_NAME_POINT_SIZE_CHANGE_TOPIC "rviz_cloud_annotation/point_size_change_topic"
#define PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC "/rviz_cloud_annotation/point_size_change"

#define PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC "rviz_cloud_annotation/point_weight_topic"
#define PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC "/rviz_cloud_annotation/point_weight"

#define PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC "rviz_cloud_annotation/point_max_weight_topic"
#define PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC "/rviz_cloud_annotation/point_max_weight"

#define PARAM_NAME_YAW_TOPIC "rviz_cloud_annotation/yaw_topic"
#define PARAM_DEFAULT_YAW_TOPIC "/rviz_cloud_annotation/yaw"

#define PARAM_NAME_OCCLUDED_TOPIC "rviz_cloud_annotation/occluded_topic"
#define PARAM_DEFAULT_OCCLUDED_TOPIC "/rviz_cloud_annotation/occluded"

#define PARAM_NAME_YAW_MAX_TOPIC "rviz_cloud_annotation/yaw_max_topic"
#define PARAM_DEFAULT_YAW_MAX_TOPIC "/rviz_cloud_annotation/yaw_max"

#define PARAM_NAME_YAW_MIN_TOPIC "rviz_cloud_annotation/yaw_min_topic"
#define PARAM_DEFAULT_YAW_MIN_TOPIC "/rviz_cloud_annotation/yaw_min"

#define PARAM_NAME_BIAS_TOPIC "rviz_cloud_annotation/bias_topic"
#define PARAM_DEFAULT_BIAS_TOPIC "/rviz_cloud_annotation/bias"

#define PARAM_NAME_BIAS_ZERO_TOPIC "rviz_cloud_annotation/bias_zero_topic"
#define PARAM_DEFAULT_BIAS_ZERO_TOPIC "/rviz_cloud_annotation/bias_zero"

#define PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC "rviz_cloud_annotation/goto_first_unused_topic"
#define PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC "/rviz_cloud_annotation/goto_first_unused"

#define PARAM_NAME_GOTO_LAST_UNUSED_TOPIC "rviz_cloud_annotation/goto_last_unused_topic"
#define PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC "/rviz_cloud_annotation/goto_last_unused"

#define PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC "rviz_cloud_annotation/goto_next_unused_topic"
#define PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC "/rviz_cloud_annotation/goto_next_unused"

#define PARAM_NAME_GOTO_FIRST_TOPIC "rviz_cloud_annotation/goto_first_topic"
#define PARAM_DEFAULT_GOTO_FIRST_TOPIC "/rviz_cloud_annotation/goto_first"

#define PARAM_NAME_RECT_SELECTION_TOPIC "rviz_cloud_annotation/rect_selection_topic"
#define PARAM_DEFAULT_RECT_SELECTION_TOPIC "/rviz_cloud_annotation/rect_selection"

#define PARAM_NAME_TOOL_TYPE_TOPIC "rviz_cloud_annotation/tool_type_topic"
#define PARAM_DEFAULT_TOOL_TYPE_TOPIC "/rviz_cloud_annotation/tool_type"

#define PARAM_NAME_ANNOTATION_TYPE_TOPIC "rviz_cloud_annotation/annotation_type_topic"
#define PARAM_DEFAULT_ANNOTATION_TYPE_TOPIC "/rviz_cloud_annotation/annotation_type"

#define PARAM_NAME_TOGGLE_NONE_TOPIC "rviz_cloud_annotation/toggle_none_topic"
#define PARAM_DEFAULT_TOGGLE_NONE_TOPIC "/rviz_cloud_annotation/toggle_none"

// parameters for smart labeling
// neighborhood graph distance
#define PARAM_NAME_NEIGH_SEARCH_DISTANCE "neighborhood_search_distance"  // DEPRECATED
#define PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE (0.0)

#define PARAM_NAME_NEIGH_SEARCH_TYPE "neigh_search_type"
#define PARAM_DEFAULT_NEIGH_SEARCH_TYPE (0)
#define PARAM_VALUE_NEIGH_SEARCH_FIXED_DISTANCE (0)  // neigh_search_params is the distance (float)
#define PARAM_VALUE_NEIGH_SEARCH_KNN_ATMOST (1)      // neigh_search_params is the maximum number of neighbors (integer)
#define PARAM_VALUE_NEIGH_SEARCH_KNN_ATLEAST (2)     // neigh_search_params is the minimum number of neighbors (integer)

// this is always a string, content depends on neigh_search_type
#define PARAM_NAME_NEIGH_SEARCH_PARAMS "neigh_search_params"
#define PARAM_DEFAULT_NEIGH_SEARCH_PARAMS ""

// max label size per control point
#define PARAM_NAME_MAX_DISTANCE "max_label_distance"
#define PARAM_DEFAULT_MAX_DISTANCE (0.0)

#define PARAM_NAME_COLOR_IMPORTANCE "color_importance"
#define PARAM_DEFAULT_COLOR_IMPORTANCE (0.0)

#define PARAM_NAME_POSITION_IMPORTANCE "position_importance"
#define PARAM_DEFAULT_POSITION_IMPORTANCE (1.0)

#define PARAM_NAME_NORMAL_IMPORTANCE "normal_importance"
#define PARAM_DEFAULT_NORMAL_IMPORTANCE (0.0)
// end parameters for smart labeling

// this number of weight steps, plus the step 0
#define PARAM_NAME_WEIGHT_STEPS "weight_steps"
#define PARAM_DEFAULT_WEIGHT_STEPS (100)

#define PARAM_NAME_YAW_MAX "yaw_max"
#define PARAM_DEFAULT_YAW_MAX (180)

#define PARAM_NAME_YAW_MIN "yaw_min"
#define PARAM_DEFAULT_YAW_MIN (-180)

#define EDIT_MODE_NONE (0)
#define EDIT_MODE_CONTROL_POINT (1)
#define EDIT_MODE_ERASER (2)
#define EDIT_MODE_COLOR_PICKER (3)
#define EDIT_MODE_MAX (4)

#define ANNOTATION_TYPE_BBOX (0)
#define ANNOTATION_TYPE_PLANE (1)
#define ANNOTATION_TYPE_KERB (2)
#define ANNOTATION_TYPE_LANE (3)

#define TOOL_TYPE_SINGLE_PICK (0)
#define TOOL_TYPE_DEEP_SQUARE (1)
#define TOOL_TYPE_SHALLOW_SQUARE (2)
#define TOOL_TYPE_SHALLOW_POLY (3)
#define TOOL_TYPE_MAX (4)

#define POINT_SIZE_CHANGE_BIGGER (1)
#define POINT_SIZE_CHANGE_SMALLER (-1)
#define POINT_SIZE_CHANGE_RESET (0)

#define LOG_FILE "LogFile"
#define LOG_FILE_DEFAULT "/LogFile/"

#define DATASET_FOLDER "cloud_file_folder"
#define ANNOTATION_FILE_FOLDER "annotation_file_folder"
#define ANNOTATION_CLOUD_FOLDER "annotation_cloud_folder"
#define LABEL_NAME_FOLDER "label_names_file_folder"
#define LINE_NAME_FOLDER "line_names_file_folder"
#define BBOX_NAME_FOLDER "bbox_names_file_folder"

#define DATASET_FOLDER_DEFAULT "/cloud_file_folder/"
#define ANNOTATION_FILE_FOLDER_DEFAULT "/annotation_file_folder/"
#define ANNOTATION_CLOUD_FOLDER_DEFAULT "/annotation_cloud_folder/"
#define LABEL_NAME_FOLDER_DEFAULT "/label_names_file_folder/"
#define LINE_NAME_FOLDER_DEFAULT "/line_names_file_folder/"
#define BBOX_NAME_FOLDER_DEFAULT "/bbox_names_file_folder/"

#define SAVE_BBOX_NAME "Bbox"
#define SAVE_BBOX_DEFAULT_NAME "Bbox.txt"

#endif  // RVIZ_CLOUD_ANNOTATION_H
