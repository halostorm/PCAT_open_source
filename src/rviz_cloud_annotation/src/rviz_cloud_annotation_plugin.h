#ifndef RVIZ_CLOUD_ANNOTATION_PLUGIN_H
#define RVIZ_CLOUD_ANNOTATION_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

#include <std_msgs/String.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/Empty.h>

#include <stdint.h>
#include <vector>

#include <rviz_cloud_annotation/UndoRedoState.h>

class QLabel;
class QPushButton;
class QButtonGroup;
class QLineEdit;
class QAction;
class QToolButton;
class QSlider;
class QMenu;

namespace pcl
{
class RGB;
}

namespace rviz_cloud_annotation
{

class QRVizCloudAnnotation : public rviz::Panel
{
  Q_OBJECT;

  typedef uint64_t uint64;
  typedef int64_t int64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<int64_t> Int64Vector;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<QPushButton *> PQPushButtonVector;

public:
  QRVizCloudAnnotation(QWidget *parent = NULL);
  virtual ~QRVizCloudAnnotation();

private Q_SLOTS:
  void onSetEditMode(int mode);
  void onSetToolType(int type);

  void onSetAnnotationType(uint32 annotation_type);

  void onLabelButtonSelected(int id);
  void onPlusLabel();
  void onMinusLabel();
  void onPageUp();
  void onPageDown();

  void onSave();
  void onRestore();
  void onClear();
  void onNew();
  void onClearCurrent();

  void onUndo();
  void onRedo();

  void onSendName();

  void onViewCloudToggled(const bool checked);
  void onViewControlPointsToggled(const bool checked);
  void onViewLabelsToggled(const bool checked);

  void onGotoFirstUnused();
  void onGotoLastUnused();
  void onGotoFirst();
  void onGotoNextUnused();

  void onSmallerPoints();
  void onBiggerPoints();
  void onResetPointsSize();

  void onControlYawSliderMoved(int new_value);
  void onControlYawSliderSet(int new_value);
  void onSetControlMaxYaw(const std_msgs::Int32 &msg);
  void onSetControlMinYaw(const std_msgs::Int32 &msg);
  void onYawInc();
  void onYawDec();

  void onBiasX1();
  void onBiasX2();
  void onBiasX3();
  void onBiasX4();
  void onBiasY1();
  void onBiasY2();
  void onBiasY3();
  void onBiasY4();
  void onBiasZ1();
  void onBiasZ2();
  void onBiasZ3();
  void onBiasZ4();

  void onSetBiasZero(const std_msgs::Empty &msg);

  void onSetObjectId(const std_msgs::Int32 &msg);
  void onChangeObjectId();

  void onControlPointWeightSliderMoved(int new_value);
  void onControlPointWeightSliderSet(int new_value);
  void onControlPointWeightInc();
  void onControlPointWeightDec();
  void onControlPointWeightMax();
  void onControlPointWeightMin();
  void onSetControlPointMaxWeight(const std_msgs::Int32 &msg);

private:
  void SetCurrentEditMode(const uint64 mode);

  void FillColorPageButtons();
  void FillPointCounts(uint type);
  void FillColorPageButtonStylesheet();

  void SetCurrentLabel(const uint64 label, const uint64 page);

  void onSetCurrentLabel(const std_msgs::UInt32 &label);
  void onSetEditMode2(const std_msgs::UInt32 &mode);
  void onPointCountUpdate(const std_msgs::UInt64MultiArray &counters);
  void onUndoRedoState(const rviz_cloud_annotation::UndoRedoState &msg);

  void onSetName(const std_msgs::String &name);

  uint64 GetPageForLabel(const uint64 label) const;
  uint64 GetLabelFromPageAndId(const uint64 page, const int id) const;
  uint64 GetFirstLabelForPage(const uint64 page) const;
  uint64 GetLastLabelForPage(const uint64 page) const;

  void SetUndoText(const std::string &text);
  void SetRedoText(const std::string &text);

  void onNextObject();

  void onPreObject();

  static void ColorToHex(const pcl::RGB &color, char hex[7]);

  uint64 m_current_edit_mode;

  int32 bbox_id = 0;
  int32 lane_id = 0;
  int32 kerb_id = 0;

  // 0 for the eraser
  uint64 m_current_label;
  uint64 m_current_page;

  ros::NodeHandle m_nh;
  ros::Publisher m_set_edit_mode_pub;
  ros::Publisher m_set_current_label_pub;

  ros::Publisher m_save_pub;
  ros::Publisher m_restore_pub;
  ros::Publisher m_clear_pub;
  ros::Publisher m_new_pub;

  ros::Publisher m_goto_first_unused_pub;
  ros::Publisher m_goto_last_unused_pub;
  ros::Publisher m_goto_first_pub;
  ros::Publisher m_goto_next_unused_pub;

  ros::Subscriber m_set_edit_mode_sub;
  ros::Subscriber m_set_current_label_sub;

  ros::Publisher m_set_name_pub;
  ros::Subscriber m_set_name_sub;

  ros::Publisher m_view_cloud_pub;
  ros::Publisher m_view_labels_pub;
  ros::Publisher m_view_control_points_pub;

  ros::Publisher m_next_pub;
  ros::Publisher m_pre_pub;

  ros::Publisher m_redo_pub;
  ros::Publisher m_undo_pub;
  ros::Subscriber m_undo_redo_state_sub;

  ros::Publisher m_point_size_change_pub;

  ros::Publisher m_control_points_weight_pub;
  ros::Subscriber m_control_point_max_weight_sub;

  ros::Publisher m_bias_pub;
  ros::Subscriber m_bias_zero_sub;

  ros::Publisher m_yaw_pub;
  ros::Subscriber m_yaw_max_sub;
  ros::Subscriber m_yaw_min_sub;

  ros::Publisher m_bbox_occluded_pub;

  ros::Publisher m_tool_type_pub;

  ros::Publisher m_annotation_type_pub;
  ros::Subscriber m_object_id_sub;

  QPushButton *m_edit_none_button;
  QPushButton *m_edit_control_point_button;
  QPushButton *m_edit_eraser_button;
  QPushButton *m_edit_color_picker_button;
  QButtonGroup *m_toolbar_group;

  QPushButton *m_line_button;
  QPushButton *m_plane_button;
  QPushButton *m_bbox_button;
  QButtonGroup *m_annotation_type_group;

  QPushButton *m_tool_single_button;
  QPushButton *m_tool_shallow_square_button;
  QPushButton *m_tool_deep_square_button;
  QPushButton *m_tool_shallow_poly_button;
  QButtonGroup *m_tooltype_group;

  QAction *m_prev_page_action;
  QAction *m_next_page_action;
  QAction *m_next_label_action;
  QAction *m_prev_label_action;

  QAction *m_prev_weight_action;
  QAction *m_next_weight_action;
  QAction *m_min_weight_action;
  QAction *m_max_weight_action;
  QMenu *m_weight_menu;

  QAction *m_X1_bias_action;
  QAction *m_X2_bias_action;
  QAction *m_X3_bias_action;
  QAction *m_X4_bias_action;
  QAction *m_Y1_bias_action;
  QAction *m_Y2_bias_action;
  QAction *m_Y3_bias_action;
  QAction *m_Y4_bias_action;
  QAction *m_Z1_bias_action;
  QAction *m_Z2_bias_action;
  QAction *m_Z3_bias_action;
  QAction *m_Z4_bias_action;
  QMenu *m_bias_menu;

  QAction *m_prev_yaw_action;
  QAction  *m_next_yaw_action;
  QAction  *m_max_yaw_action;
  QAction  *m_min_yaw_action;
  QMenu *m_yaw_menu;

  QAction *m_undo_action;
  QAction *m_redo_action;

  QLabel *m_current_page_label;

  QLabel *m_object_id;

  QLabel *m_current_control_point_weight_label;
  QSlider *m_current_control_point_weight_slider;
  uint32 m_control_point_weight_max;

  QLabel *m_current_yaw_label;
  QSlider *m_current_yaw_slider;
  int32 m_yaw_max;
  int32 m_yaw_min;

  PQPushButtonVector m_page_buttons;
  QButtonGroup *m_page_button_group;

  Uint64Vector m_point_counters;
  ros::Subscriber m_point_count_update_sub;

  QLineEdit *m_set_name_edit;

  uint64 m_color_cols_per_page;
  uint64 m_color_rows_per_page;

  FloatVector m_bias;
  const float bias_step = 0.05;

  const uint BBOX = 0u;
  const uint PLANE = 1u;
  const uint KERB = 2u;
  const uint LANE = 3u;

  uint ANNOTATION_TYPE = BBOX;
  
};

} // namespace rviz_cloud_annotation

#endif // RVIZ_CLOUD_ANNOTATION_PLUGIN_H
