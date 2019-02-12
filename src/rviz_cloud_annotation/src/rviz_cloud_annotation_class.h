#ifndef RVIZ_CLOUD_ANNOTATION_CLASS_H
#define RVIZ_CLOUD_ANNOTATION_CLASS_H

#include <rviz_cloud_annotation/RectangleSelectionViewport.h>
#include <rviz_cloud_annotation/UndoRedoState.h>
#include "point_cloud_plane_curves_extract.h"
#include "point_neighborhood.h"
#include "point_neighborhood_search.h"
#include "rviz_cloud_annotation.h"
#include "rviz_cloud_annotation_params.h"
#include "rviz_cloud_annotation_points.h"
#include "rviz_cloud_annotation_undo.h"

// system
#include <sys/stat.h>
#include <sys/types.h>

// STL
#include <dirent.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
// ROS
#include <eigen_conversions/eigen_msg.h>
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64MultiArray.h>
#include <termios.h>
// Eigen
#include <Eigen/Dense>
// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/colors.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OpenCv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>

#define KEYDOWN(vk_code) ((GetAsyncKeyState(vk_code) & 0x8000) ? 1 : 0)
#define KEYUP(vk_code) ((GetAsyncKeyState(vk_code) & 0x8000) ? 0 : 1)

#define CLOUD_MARKER_NAME "cloud"
#define CONTROL_POINT_MARKER_PREFIX "control_points_"
#define LABEL_POINT_MARKER_PREFIX "label_points_"
#define BBOX_MARKER_NAME "BBOX"

class RVizCloudAnnotation //点云标注Main类
{
public:
  typedef visualization_msgs::InteractiveMarker InteractiveMarker;
  typedef visualization_msgs::Marker Marker;
  typedef interactive_markers::InteractiveMarkerServer InteractiveMarkerServer;
  typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
  typedef visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedback;
  typedef visualization_msgs::InteractiveMarkerFeedbackPtr InteractiveMarkerFeedbackPtr;
  typedef visualization_msgs::InteractiveMarkerFeedbackConstPtr InteractiveMarkerFeedbackConstPtr;

  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;

  typedef pcl::Normal PointNormal;
  typedef pcl::PointCloud<PointNormal> PointNormalCloud;

  typedef pcl::PointXYZI PointXYZI;
  typedef pcl::PointCloud<PointXYZI> PointXYZICloud;

  typedef pcl::PointXYZRGB PointXYZRGB;

  typedef pcl::PointCloud<PointXYZRGB> PointXYZRGBCloud;
  typedef pcl::PointXYZRGBL PointXYZRGBL;
  typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;

  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef RVizCloudAnnotationPoints::CPData ControlPointData;
  typedef RVizCloudAnnotationPoints::CPDataVector ControlPointDataVector;

  typedef uint64_t uint64;
  typedef int64_t int64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef uint8_t uint8;
  typedef unsigned int uint;
  typedef std::vector<uint32> Uint32Vector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<int64_t> Int64Vector;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<std::string> StringVector;
  typedef std::vector<Eigen::Vector3f> PointVector;

  struct Int32PolyTriangle
  {
    int32 x[3];
    int32 y[3];
    Int32PolyTriangle(int32 x1, int32 y1, int32 x2, int32 y2, int32 x3, int32 y3)
    {
      x[0] = x1;
      x[1] = x2;
      x[2] = x3;
      y[0] = y1;
      y[1] = y2;
      y[2] = y3;
    }
    int32 Contains(const int32 x, const int32 y) const;
  };
  typedef std::vector<Int32PolyTriangle> Int32PolyTriangleVector;

  enum ControlPointVisual
  {
    CONTROL_POINT_VISUAL_SPHERE,
    CONTROL_POINT_VISUAL_THREE_SPHERES,
    CONTROL_POINT_VISUAL_LINE,
  };

  RVizCloudAnnotation(ros::NodeHandle &nh);

  void LoadCloud(const std::string &filename, const std::string &normal_source, PointXYZRGBNormalCloud &cloud);
  void LoadCloud1(std::string &filename, const std::string &normal_source, PointXYZRGBNormalCloud &cloud);

  void onSave(const std_msgs::String &filename_msg)
  {
    Save(false);
  }
  void onAutosave(const ros::TimerEvent &event)
  {
    Save(true);
  }
  void Save(const bool is_autosave = false);

  std::string AppendTimestampBeforeExtension(const std::string &filename);

  void onRestore(const std_msgs::String &filename_msg)
  {
    std::string filename = filename_msg.data.empty() ? m_annotation_filename_out : filename_msg.data;
    Restore(filename);
  }

  void Restore(const std::string &filename);

  void onClear(const std_msgs::UInt32 &label_msg);

  void onClickOnCloud(const InteractiveMarkerFeedbackConstPtr &feedback_ptr);

  std::string GetClickType(const std::string &marker_name, uint64 &label_out) const;

  uint64 GetClickedPointId(const InteractiveMarkerFeedback &click_feedback, bool &ok);

  void onRectangleSelectionViewport(const rviz_cloud_annotation::RectangleSelectionViewport &msg);

  void VectorSelection(const Uint64Vector &ids);

  Uint64Vector RectangleSelectionToIds(const Eigen::Matrix4f proj_matrix, const Eigen::Affine3f camera_pose_inv,
                                       const PointXYZRGBNormalCloud &cloud, const uint32 start_x, const uint32 start_y,
                                       const uint32 width, const uint32 height, const Int32PolyTriangleVector tri_cond,
                                       const float point_size, const float focal_length, const bool is_deep_selection);

  void SetCurrentLabel(const uint64 label);

  void SetEditMode(const uint64 new_edit_mode);

  void onViewLabels(const std_msgs::Bool &msg)
  {
    m_view_labels = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
  }

  void onViewControlPoints(const std_msgs::Bool &msg)
  {
    m_view_control_points = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
  }

  void onViewCloud(const std_msgs::Bool &msg)
  {
    m_view_cloud = msg.data;

    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
  }

  void onSetCurrentLabel(const std_msgs::UInt32 &msg)
  {
    SetCurrentLabel(msg.data);
  }

  void onSetEditMode(const std_msgs::UInt32 &msg)
  {
    SetEditMode(msg.data);
  }

  void onSetAnnotationType(const std_msgs::UInt32 &type)
  {
    ANNOTATION_TYPE = type.data;

    ROS_INFO("rviz_cloud_annotation: annotation type: %u", type.data);
  }

  void onAutoPlane(const std_msgs::Empty &data)
  {
    ROS_INFO("rviz_cloud_annotation: auto plane");
    if (ANNOTATION_TYPE == PLANE)
    {
      for (int i = 0; i < _planeRings; i++)
      {
        generatePlane(mCurveId[i], true);
      }
    }
  }

  void onToggleNoneMode(const std_msgs::Empty &msg)
  {
    if ((m_edit_mode == EDIT_MODE_NONE) && (m_prev_edit_mode != EDIT_MODE_NONE))
    {
      SetEditMode(m_prev_edit_mode);
    }
    else if (m_edit_mode != EDIT_MODE_NONE)
    {
      SetEditMode(EDIT_MODE_NONE);
    }
  }

  //设置数据集路径
  void onSetName(const std_msgs::String &msg)
  {
    ROS_INFO("rviz_cloud_annotation: New File Path");
    m_dataset_files.clear();
    m_annotation_cloud_files.clear();
    m_annotation_file_files.clear();
    m_bbox_files.clear();
    m_label_files.clear();
    FILE_ID = 0;
    std::string param_stringA = msg.data + "/pcd/";
    std::string param_stringA1 = msg.data + "/image/";
    std::string param_stringB = msg.data + "/_pcd/";
    std::string param_stringC = msg.data + "/_annotation/";
    std::string param_stringD = msg.data + "/_bbox/";
    std::string param_stringE = msg.data + "/_label/";
    std::string param_stringF;
    mkdir(param_stringA.c_str(), S_IRWXU);
    mkdir(param_stringA1.c_str(), S_IRWXU);
    mkdir(param_stringB.c_str(), S_IRWXU);
    mkdir(param_stringC.c_str(), S_IRWXU);
    mkdir(param_stringD.c_str(), S_IRWXU);
    mkdir(param_stringE.c_str(), S_IRWXU);
    LoadDataSet(param_stringA, param_stringA1, param_stringB, param_stringC, param_stringD, param_stringE,
                param_stringF);
    InitNewCloud(*nh);
  }

  void onUndo(const std_msgs::Empty &);
  void onRedo(const std_msgs::Empty &);

  void onPointSizeChange(const std_msgs::Int32 &msg);

  void onControlPointWeightChange(const std_msgs::Int32 &msg);

  void onControlYawChange(const std_msgs::Int32 &msg);

  void onControlOccludedChange(const std_msgs::Int32 &msg);

  void onControlBiasChange(const std_msgs::Float32MultiArray &msg);

  void onGotoFirstUnused(const std_msgs::Empty &);
  void onGotoLastUnused(const std_msgs::Empty &);
  void onGotoFirst(const std_msgs::Empty &);
  void onGotoNextUnused(const std_msgs::Empty &);

  std::string label2Name(int label);

  void SendName();

  void SendBiasZero();

  void SendUndoRedoState();

  void SendPointCounts(const Uint64Vector &labels);

  Uint64Vector RangeUint64(const uint64 start, const uint64 end) const
  {
    Uint64Vector result(end - start);
    for (uint64 i = start; i < end; i++)
      result[i - start] = i;
    return result;
  }

  void SendCloudMarker(const bool apply);

  void ClearControlPointsMarker(const Uint64Vector &indices, const bool apply);

  void SendControlPointsMarker(const Uint64Vector &changed_labels, const bool apply);

  void SendControlPointMaxWeight();

  void SendYawMax();

  void SendYawMin();

  InteractiveMarker ControlPointsToMarker(const PointXYZRGBNormalCloud &cloud,
                                          const ControlPointDataVector &control_points, const uint64 label,
                                          const bool interactive);
  InteractiveMarker LabelsToMarker(const PointXYZRGBNormalCloud &cloud, const Uint64Vector &labels, const uint64 label,
                                   const bool interactive);

  InteractiveMarker CloudToMarker(const PointXYZRGBNormalCloud &cloud, const bool interactive);

  void colorize_point_cloud(double intensity, PointXYZRGB *sample);

private:
  ros::NodeHandle &m_nh;
  InteractiveMarkerServerPtr m_interactive_marker_server;
  PointXYZRGBNormalCloud::Ptr m_cloud;

  RVizCloudAnnotationPoints::ConstPtr m_annotation;
  RVizCloudAnnotationUndo m_undo_redo;

  KdTree::Ptr m_kdtree;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_toggle_none_sub;
  ros::Subscriber m_set_current_label_sub;

  ros::Publisher m_set_edit_mode_pub;
  ros::Publisher m_set_current_label_pub;

  ros::Subscriber m_set_name_sub;
  ros::Publisher m_set_name_pub;

  ros::Subscriber m_rect_selection_sub;

  ros::Subscriber m_save_sub;
  ros::Subscriber m_restore_sub;
  ros::Subscriber m_clear_sub;
  ros::Subscriber m_new_sub;

  ros::Subscriber m_next_sub;
  ros::Subscriber m_pre_sub;

  ros::Subscriber m_undo_sub;
  ros::Subscriber m_redo_sub;
  ros::Publisher m_undo_redo_state_pub;

  ros::Subscriber m_point_size_change_sub;

  ros::Subscriber m_goto_first_unused_sub;
  ros::Subscriber m_goto_last_unused_sub;
  ros::Subscriber m_goto_first_sub;
  ros::Subscriber m_goto_next_unused_sub;

  ros::Subscriber m_view_control_points_sub;
  ros::Subscriber m_view_cloud_sub;
  ros::Subscriber m_view_labels_sub;
  bool m_view_cloud;
  bool m_view_labels;
  bool m_view_control_points;

  ros::Publisher m_point_count_update_pub;

  ros::Subscriber m_control_points_weight_sub;
  ros::Publisher m_control_point_weight_max_weight_pub;

  ros::Subscriber m_bbox_occluded_sub;

  ros::Subscriber m_control_yaw_sub;
  ros::Publisher m_control_yaw_max_pub;
  ros::Publisher m_control_yaw_min_pub;

  ros::Publisher m_control_bias_zero_pub;
  ros::Subscriber m_control_bias_sub;

  uint32 m_control_point_weight_step;
  uint32 m_control_point_max_weight;

  ros::Subscriber m_on_set_annotation_type_sub;

  ros::Subscriber m_on_auto_plane_sub;

  ros::Publisher m_object_id_pub;

  int32 m_control_yaw_step;
  int32 m_control_yaw_min;
  int32 m_control_yaw_max;

  ros::Timer m_autosave_timer;
  bool m_autosave_append_timestamp;

  std::string m_frame_id;
  float m_point_size;
  float m_label_size;
  float m_control_label_size;

  float m_point_size_multiplier;
  float m_point_size_change_multiplier;

  bool m_show_points_back_labels;
  float m_cp_weight_scale_fraction;
  ControlPointVisual m_control_points_visual;
  bool m_show_zero_weight_control_points;

  uint64 m_current_label;
  uint64 m_edit_mode;
  uint64 m_prev_edit_mode;

  PointNeighborhood::ConstPtr m_point_neighborhood;

  std::string m_annotation_filename_in;
  std::string m_annotation_filename_out;
  std::string m_ann_cloud_filename_out;
  std::string m_label_names_filename_out;

  std::string m_bbox_save;

  // My defined data:
private:
  std::string m_dataset_folder;
  std::string m_image_folder;
  std::string m_annotation_cloud_folder;
  std::string m_annotation_file_folder;
  std::string m_bbox_folder;
  std::string m_line_folder;
  std::string m_label_folder;

  std::string m_log_file;

  StringVector m_dataset_files;
  StringVector m_image_files;
  StringVector m_annotation_cloud_files;
  StringVector m_annotation_file_files;
  StringVector m_bbox_files;
  StringVector m_line_files;
  StringVector m_label_files;

  int FILE_ID = 0;

  ros::Publisher bbox_marker_pub;
  ros::Publisher bbox_head_marker_pub;

  ros::Publisher kerb_marker_pub;

  ros::Publisher plane_marker_pub;

  ros::Publisher lane_marker_pub;

  const uint BBOX = 0u;
  const uint PLANE = 1u;
  const uint KERB = 2u;
  const uint LANE = 3u;

  // MarkerArray LineMarkers;

  uint ANNOTATION_TYPE = BBOX;

  int BBOX_ID = 0;

  int KERB_ID = 0;

  int LANE_ID = 0;

  int PLANE_ID = 0;

  float m_box_bias[BBOXNUMBER_LINEPOINTNUMBER][6] = {{0}};

  float BBOX_YAW = 0;

  int BBOX_YAW_ANGLE = 0;

  float KERB_YAW = 0;

  bool if_tilt = false;

  Uint64Vector m_points_to_abandon;

  Uint64Vector ids_in_bbox[BBOXNUMBER_LINEPOINTNUMBER];

  Uint64Vector ids_in_kerb[LINENUMBER];

  Uint64Vector ids_in_lane[LINENUMBER];

  Uint64Vector mCurveId[_planeRings];

  Uint64Vector ids_in_plane;

  Uint64Vector ids_out_plane;

  Uint64Vector ids_in_plane_flag;

  float BBOX_SET[BBOXNUMBER_LINEPOINTNUMBER][11] = {{0}};

  float BBOX_LABEL_SET[BBOXNUMBER_LINEPOINTNUMBER][10] = {{0}};

  float KERB_SET[LINENUMBER][BBOXNUMBER_LINEPOINTNUMBER][3] = {{{0}}};

  int KERB_SIZE[LINENUMBER] = {0};

  float LANE_SET[LINENUMBER][BBOXNUMBER_LINEPOINTNUMBER][3] = {{{0}}};

  int LANE_SIZE[LINENUMBER] = {0};

  float m_bbox_occluded[BBOXNUMBER_LINEPOINTNUMBER] = {0};

  Int64Vector m_label;

  Int64Vector m_plane_flag;

  std::string param_string2;

  ros::NodeHandle *nh;

  // My defined function:
public:
  void InitNewCloud(ros::NodeHandle &nh);

  void LoadDataSet(std::string A, std::string A1, std::string B, std::string C, std::string D, std::string E,
                   std::string F);

  void AddBbox(float A, float B, float B1, float B2, float B3, float B4, float C1, float C2, bool tilt);

  void onNew(const std_msgs::UInt32 &label_msg);

  void EmptyBboxToMarker(const int id);

  void EmptyLaneToMarker(const int id);

  void EmptyPlaneToMarker(const int id);

  void EmptyKerbToMarker(const int id);

  void FinalLabel(PointXYZRGBNormalCloud &cloud);

  bool InBbox(float x, float y, float z, int i);

  bool InKerb(float x, float y, float z, uint64 id, int kerb_id);

  bool InLane(float x, float y, float z, uint64 id, int lane_id);

  bool InPlane(float x, float y, float z, uint64 id);

  void BboxToMarker(const float shape[], const int id, bool if_tilt);

  RVizCloudAnnotation::Uint64Vector RecountIds(const Uint64Vector &ids);

  void generateLane(const PointXYZRGBNormalCloud &cloud);

  void generateDefaultPlane(PointXYZRGBNormalCloud &cloud);

  void generatePlane(const Uint64Vector &ids, bool editMode);

  // void generatePlane(const PointXYZRGBNormalCloud &cloud);

  void generateKerb(const PointXYZRGBNormalCloud &cloud);

  void generateBbox(const PointXYZRGBNormalCloud &cloud, bool if_tilt);

  void onNextObject(const std_msgs::Empty &)
  {
    std_msgs::Int32 msg;
    if (ANNOTATION_TYPE == BBOX)
    {
      BBOX_ID++;
      // ROS_INFO("rviz_cloud_annotation:current BBOX ID: %i", BBOX_ID);
      SendControlPointMaxWeight();
      SendYawMin();
      BBOX_YAW = 0;
      SendBiasZero();
      msg.data = BBOX_ID;
    }
    else if (ANNOTATION_TYPE == KERB)
    {
      KERB_ID++;
      // ROS_INFO("rviz_cloud_annotation:current KERB_ID: %i", KERB_ID);
      msg.data = KERB_ID;
    }
    else if (ANNOTATION_TYPE == LANE)
    {
      LANE_ID++;
      // ROS_INFO("rviz_cloud_annotation:current LANE_ID: %i", LANE_ID);
      msg.data = LANE_ID;
    }
    m_object_id_pub.publish(msg);
  }

  void gotoNextObject()
  {
    std_msgs::Int32 msg;
    if (ANNOTATION_TYPE == BBOX)
    {
      BBOX_ID++;
      // ROS_INFO("rviz_cloud_annotation:current BBOX ID: %i", BBOX_ID);
      SendControlPointMaxWeight();
      SendYawMin();
      BBOX_YAW = 0;
      SendBiasZero();
      msg.data = BBOX_ID;
    }
    else if (ANNOTATION_TYPE == KERB)
    {
      KERB_ID++;
      // ROS_INFO("rviz_cloud_annotation:current KERB_ID: %i", KERB_ID);
      msg.data = KERB_ID;
    }
    else if (ANNOTATION_TYPE == LANE)
    {
      LANE_ID++;
      // ROS_INFO("rviz_cloud_annotation:current LANE_ID: %i", LANE_ID);
      msg.data = LANE_ID;
    }
    m_object_id_pub.publish(msg);
  }

  void onPreObject(const std_msgs::Empty &)
  {
    std_msgs::Int32 msg;
    if (ANNOTATION_TYPE == BBOX)
    {
      if (BBOX_ID > 0)
      {
        BBOX_ID--;
        msg.data = BBOX_ID;
      }
      // ROS_INFO("rviz_cloud_annotation:current BBOX ID: %i", BBOX_ID);
    }
    else if (ANNOTATION_TYPE == KERB)
    {
      if (KERB_ID > 0)
      {
        KERB_ID--;
        msg.data = KERB_ID;
      }
      // ROS_INFO("rviz_cloud_annotation:current KERB_ID: %i", KERB_ID);
    }
    else if (ANNOTATION_TYPE == LANE)
    {
      if (LANE_ID > 0)
      {
        LANE_ID--;
        msg.data = LANE_ID;
      }
      // ROS_INFO("rviz_cloud_annotation:current LANE_ID: %i", LANE_ID);
    }
    m_object_id_pub.publish(msg);
  }

  float line(float v);
  float line(float x, float y, float k);

  void int2str(const int &int_temp, std::string &string_temp)
  {
    std::stringstream stream;
    stream << int_temp;
    string_temp = stream.str();
  }
  float m_sqrt(float x)
  {
    float half_x = 0.5 * x;
    int i = *((int *)&x);
    i = 0x5f3759df - (i >> 1);
    x = *((float *)&i);
    x = x * (1.5 - (half_x * x * x));
    return 1 / x;
  }
};

#endif // RVIZ_CLOUD_ANNOTATION_CLASS_H
