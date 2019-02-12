#include "rviz_cloud_annotation_plugin.h"

namespace rviz_cloud_annotation
{
QRVizCloudAnnotation::QRVizCloudAnnotation(QWidget *parent) : rviz::Panel(parent), m_nh("~")
{
  /*
  time_t deadLine = 1546272000;
  char deadLineTime[64];
  strftime(deadLineTime, sizeof(deadLineTime), "%Y-%m-%d %H:%M:%S", localtime(&deadLine));

  time_t tt = time(NULL);
  time_t begin = tt;
  FirstUsedTime(begin);

  if (tt > deadLine || tt - begin > 3 * 24 * 3600)
  {
    const std::string msg = std::string("您好，试用期已经结束，请获取完整版！ ");
    QMessageBox::about(NULL, "提示", msg.c_str());
    return;
  }
  else
  {
    const std::string msg = std::string("您好，试用截止日期为: ") + std::string(deadLineTime);
    QMessageBox::about(NULL, "提示", msg.c_str());
  }
  */

  {
    std::string param_string;
    int temp_int;

    m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC, param_string, PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
    m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2, param_string, PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
    m_set_edit_mode_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetEditMode2, this);

    m_current_page = 0;

    m_nh.param<int>(PARAM_NAME_COLORS_COLS_PER_PAGE, temp_int, PARAM_DEFAULT_COLOR_COLS_PER_PAGE);
    if (temp_int > 0 && temp_int < 30)
      m_color_cols_per_page = temp_int;
    else
    {
      m_color_cols_per_page = PARAM_DEFAULT_COLOR_COLS_PER_PAGE;
      ROS_WARN("rviz_cloud_annotation_plugin: %s is out of sane range: %i, using default.",
               PARAM_NAME_COLORS_COLS_PER_PAGE, temp_int);
    }

    m_nh.param<int>(PARAM_NAME_COLORS_ROWS_PER_PAGE, temp_int, PARAM_DEFAULT_COLOR_ROWS_PER_PAGE);
    if (temp_int > 0 && temp_int < 10)
      m_color_rows_per_page = temp_int;
    else
    {
      m_color_cols_per_page = PARAM_DEFAULT_COLOR_ROWS_PER_PAGE;
      ROS_WARN("rviz_cloud_annotation_plugin: %s is out of sane range: %i, using default.",
               PARAM_NAME_COLORS_ROWS_PER_PAGE, temp_int);
    }

    m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC, param_string, PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
    m_set_current_label_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_CURRENT_LABEL_TOPIC, param_string, PARAM_DEFAULT_CURRENT_LABEL_TOPIC);
    m_set_current_label_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetCurrentLabel, this);

    m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC, param_string, PARAM_DEFAULT_SAVE_TOPIC);
    m_save_pub = m_nh.advertise<std_msgs::String>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC, param_string, PARAM_DEFAULT_RESTORE_TOPIC);
    m_restore_pub = m_nh.advertise<std_msgs::String>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC, param_string, PARAM_DEFAULT_CLEAR_TOPIC);
    m_clear_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_NEW_TOPIC, param_string, PARAM_DEFAULT_NEW_TOPIC);
    m_new_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC, param_string, PARAM_DEFAULT_SET_NAME_TOPIC);
    m_set_name_pub = m_nh.advertise<std_msgs::String>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC2, param_string, PARAM_DEFAULT_SET_NAME_TOPIC2);
    m_set_name_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetName, this);

    m_nh.param<std::string>(PARAM_NAME_POINT_COUNT_UPDATE_TOPIC, param_string, PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC);
    m_point_count_update_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onPointCountUpdate, this);

    m_nh.param<std::string>(PARAM_NAME_VIEW_CLOUD_TOPIC, param_string, PARAM_DEFAULT_VIEW_CLOUD_TOPIC);
    m_view_cloud_pub = m_nh.advertise<std_msgs::Bool>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC, param_string,
                            PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC);
    m_view_control_points_pub = m_nh.advertise<std_msgs::Bool>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_VIEW_LABEL_TOPIC, param_string, PARAM_DEFAULT_VIEW_LABEL_TOPIC);
    m_view_labels_pub = m_nh.advertise<std_msgs::Bool>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_UNDO_TOPIC, param_string, PARAM_DEFAULT_UNDO_TOPIC);
    m_undo_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_REDO_TOPIC, param_string, PARAM_DEFAULT_REDO_TOPIC);
    m_redo_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_NEXT_TOPIC, param_string, PARAM_DEFAULT_NEXT_TOPIC);
    m_next_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_PRE_TOPIC, param_string, PARAM_DEFAULT_PRE_TOPIC);
    m_pre_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_UNDO_REDO_STATE_TOPIC, param_string, PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC);
    m_undo_redo_state_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onUndoRedoState, this);

    m_nh.param<std::string>(PARAM_NAME_POINT_SIZE_CHANGE_TOPIC, param_string, PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC);
    m_point_size_change_pub = m_nh.advertise<std_msgs::Int32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_BIAS_TOPIC, param_string, PARAM_DEFAULT_BIAS_TOPIC);
    m_bias_pub = m_nh.advertise<std_msgs::Float32MultiArray>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_BIAS_ZERO_TOPIC, param_string, PARAM_DEFAULT_BIAS_ZERO_TOPIC);
    m_bias_zero_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetBiasZero, this);

    m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC, param_string,
                            PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC);
    m_control_points_weight_pub = m_nh.advertise<std_msgs::Int32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC, param_string,
                            PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC);
    m_control_point_max_weight_sub =
        m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetControlPointMaxWeight, this);

    m_nh.param<std::string>(PARAM_NAME_YAW_TOPIC, param_string, PARAM_DEFAULT_YAW_TOPIC);
    m_yaw_pub = m_nh.advertise<std_msgs::Int32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_OCCLUDED_TOPIC, param_string, PARAM_DEFAULT_OCCLUDED_TOPIC);
    m_bbox_occluded_pub = m_nh.advertise<std_msgs::Int32>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_YAW_MAX_TOPIC, param_string, PARAM_DEFAULT_YAW_MAX_TOPIC);
    m_yaw_max_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetControlMaxYaw, this);

    m_nh.param<std::string>(PARAM_NAME_YAW_MIN_TOPIC, param_string, PARAM_DEFAULT_YAW_MIN_TOPIC);
    m_yaw_min_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetControlMinYaw, this);

    m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC, param_string, PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC);
    m_goto_first_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_TOPIC, param_string, PARAM_DEFAULT_GOTO_FIRST_TOPIC);
    m_goto_first_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_GOTO_LAST_UNUSED_TOPIC, param_string, PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC);
    m_goto_last_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC, param_string, PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC);
    m_goto_next_unused_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_TOOL_TYPE_TOPIC, param_string, PARAM_DEFAULT_TOOL_TYPE_TOPIC);
    m_tool_type_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1, true);

    m_nh.param<std::string>(PARAM_NAME_ANNOTATION_TYPE_TOPIC, param_string, PARAM_DEFAULT_ANNOTATION_TYPE_TOPIC);
    m_annotation_type_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1, true);

    m_nh.param<std::string>(PARAM_NAME_OBJECT_ID_TOPIC, param_string, PARAM_DEFAULT_OBJECT_ID_TOPIC);
    m_object_id_sub = m_nh.subscribe(param_string, 1, &QRVizCloudAnnotation::onSetObjectId, this);

    image_transport::ImageTransport imageTrans(m_nh);
    m_nh.param<std::string>(PARAM_IMAGE_LABEL_TOPIC, param_string, PARAM_DEFAULT_IMAGE_LABEL_TOPIC);
    m_image_label_pub = imageTrans.advertise(param_string, 1);

    m_nh.param<std::string>(PARAM_AUTO_PLANE_TOPIC, param_string, PARAM_DEFAULT_AUTO_PLANE_TOPIC);
    m_auto_plane_pub = m_nh.advertise<std_msgs::Empty>(param_string, 1);
  }

  QBoxLayout *main_layout = new QBoxLayout(QBoxLayout::TopToBottom, this);

  {
    QMenuBar *menu_bar = new QMenuBar(this);
    menu_bar->setNativeMenuBar(false);
    main_layout->addWidget(menu_bar);

    QMenu *file_menu = menu_bar->addMenu("文件");

    QAction *new_action = new QAction("切换下一帧文件", menu_bar);
    new_action->setShortcut(QKeySequence("Ctrl+Shift+O"));
    file_menu->addAction(new_action);
    connect(new_action, &QAction::triggered, this, &QRVizCloudAnnotation::onNew, Qt::QueuedConnection);

    QAction *clear_action = new QAction("清除标记", menu_bar);
    file_menu->addAction(clear_action);
    connect(clear_action, &QAction::triggered, this, &QRVizCloudAnnotation::onClear, Qt::QueuedConnection);

    QAction *save_action = new QAction("保存", menu_bar);
    save_action->setShortcut(QKeySequence("Ctrl+Shift+K"));
    file_menu->addAction(save_action);
    connect(save_action, &QAction::triggered, this, &QRVizCloudAnnotation::onSave, Qt::QueuedConnection);

    QAction *restore_action = new QAction("另存为", menu_bar);
    file_menu->addAction(restore_action);
    connect(restore_action, &QAction::triggered, this, &QRVizCloudAnnotation::onRestore, Qt::QueuedConnection);

    QMenu *edit_menu = menu_bar->addMenu("编辑");

    m_undo_action = new QAction("取消", menu_bar);
    m_undo_action->setEnabled(false);
    m_undo_action->setShortcut(QKeySequence("U"));
    edit_menu->addAction(m_undo_action);
    connect(m_undo_action, &QAction::triggered, this, &QRVizCloudAnnotation::onUndo);

    m_redo_action = new QAction("恢复", menu_bar);
    m_redo_action->setEnabled(false);
    m_redo_action->setShortcut(QKeySequence("Shift+U"));
    edit_menu->addAction(m_redo_action);
    connect(m_redo_action, &QAction::triggered, this, &QRVizCloudAnnotation::onRedo);

    QMenu *view_menu = menu_bar->addMenu("视图");

    QAction *bigger_points_action = new QAction("增加点的尺寸", menu_bar);
    bigger_points_action->setShortcut(QKeySequence("Shift+O"));
    view_menu->addAction(bigger_points_action);
    connect(bigger_points_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiggerPoints);

    QAction *smaller_points_action = new QAction("减小点的尺寸", menu_bar);
    smaller_points_action->setShortcut(QKeySequence("O"));
    view_menu->addAction(smaller_points_action);
    connect(smaller_points_action, &QAction::triggered, this, &QRVizCloudAnnotation::onSmallerPoints);

    QAction *reset_points_size_action = new QAction("重置尺寸", menu_bar);
    reset_points_size_action->setShortcut(QKeySequence("Alt+O"));
    view_menu->addAction(reset_points_size_action);
    connect(reset_points_size_action, &QAction::triggered, this, &QRVizCloudAnnotation::onResetPointsSize);

    QMenu *label_menu = menu_bar->addMenu("标记");

    QAction *clear_current_action = new QAction("清除当前标记", menu_bar);
    label_menu->addAction(clear_current_action);
    clear_current_action->setShortcut(QKeySequence("Ctrl+Shift+C"));
    connect(clear_current_action, &QAction::triggered, this, &QRVizCloudAnnotation::onClearCurrent,
            Qt::QueuedConnection);

    label_menu->addSeparator();

    m_prev_label_action = new QAction("切换到上一颜色", menu_bar);
    label_menu->addAction(m_prev_label_action);
    m_prev_label_action->setShortcut(QKeySequence("Shift+C"));
    connect(m_prev_label_action, &QAction::triggered, this, &QRVizCloudAnnotation::onMinusLabel);

    m_next_label_action = new QAction("切换到下一颜色", menu_bar);
    label_menu->addAction(m_next_label_action);
    m_next_label_action->setShortcut(QKeySequence("Shift+Z"));
    connect(m_next_label_action, &QAction::triggered, this, &QRVizCloudAnnotation::onPlusLabel);

    label_menu->addSeparator();

    m_prev_page_action = new QAction("上一页标签", menu_bar);
    label_menu->addAction(m_prev_page_action);
    m_prev_page_action->setShortcut(QKeySequence("PgUp"));
    connect(m_prev_page_action, &QAction::triggered, this, &QRVizCloudAnnotation::onPageUp);

    m_next_page_action = new QAction("下一页标签", menu_bar);
    label_menu->addAction(m_next_page_action);
    m_next_page_action->setShortcut(QKeySequence("PgDown"));
    connect(m_next_page_action, &QAction::triggered, this, &QRVizCloudAnnotation::onPageDown);

    label_menu->addSeparator();

    QMenu *next_menu = menu_bar->addMenu("选择");

    QAction *next_object = new QAction("下一个物体", menu_bar);
    next_menu->addAction(next_object);
    next_object->setShortcut(QKeySequence("Shift+N"));
    connect(next_object, &QAction::triggered, this, &QRVizCloudAnnotation::onNextObject, Qt::QueuedConnection);

    QAction *pre_object = new QAction("上一个物体", menu_bar);
    next_menu->addAction(pre_object);
    pre_object->setShortcut(QKeySequence("Shift+P"));
    connect(pre_object, &QAction::triggered, this, &QRVizCloudAnnotation::onPreObject, Qt::QueuedConnection);

    // go to submenu
    {
      QMenu *goto_menu = label_menu->addMenu("跳转");

      QAction *goto_first_action = new QAction("第一个", menu_bar);
      goto_menu->addAction(goto_first_action);
      goto_first_action->setShortcut(QKeySequence("Home"));
      connect(goto_first_action, &QAction::triggered, this, &QRVizCloudAnnotation::onGotoFirst);

      QAction *goto_first_unused_action = new QAction("第一个空标签", menu_bar);
      goto_menu->addAction(goto_first_unused_action);
      connect(goto_first_unused_action, &QAction::triggered, this, &QRVizCloudAnnotation::onGotoFirstUnused);

      QAction *goto_next_unused_action = new QAction("下一个空标签", menu_bar);
      goto_menu->addAction(goto_next_unused_action);
      connect(goto_next_unused_action, &QAction::triggered, this, &QRVizCloudAnnotation::onGotoNextUnused);

      QAction *goto_last_unused_action = new QAction("最后一个空标签", menu_bar);
      goto_menu->addAction(goto_last_unused_action);
      connect(goto_last_unused_action, &QAction::triggered, this, &QRVizCloudAnnotation::onGotoLastUnused);
    }

    label_menu->addSeparator();

    m_auto_plane_action = new QAction("自动生成地面(仅16线)", menu_bar);
    label_menu->addAction(m_auto_plane_action);
    connect(m_auto_plane_action, &QAction::triggered, this, &QRVizCloudAnnotation::onAutoPlane);

    label_menu->addSeparator();

    // weight submenu
    {
      m_weight_menu = label_menu->addMenu("调节BBox遮挡系数");
      m_weight_menu->setEnabled(false);

      m_prev_weight_action = new QAction("减小", menu_bar);
      m_weight_menu->addAction(m_prev_weight_action);
      m_prev_weight_action->setShortcut(QKeySequence("G"));
      connect(m_prev_weight_action, &QAction::triggered, this, &QRVizCloudAnnotation::onControlPointWeightDec);

      m_next_weight_action = new QAction("增加", menu_bar);
      m_weight_menu->addAction(m_next_weight_action);
      connect(m_next_weight_action, &QAction::triggered, this, &QRVizCloudAnnotation::onControlPointWeightInc);
      m_next_weight_action->setShortcut(QKeySequence("T"));

      m_min_weight_action = new QAction("最小值", menu_bar);
      m_weight_menu->addAction(m_min_weight_action);
      connect(m_min_weight_action, &QAction::triggered, this, &QRVizCloudAnnotation::onControlPointWeightMin);

      m_max_weight_action = new QAction("最大值", menu_bar);
      m_weight_menu->addAction(m_max_weight_action);
      connect(m_max_weight_action, &QAction::triggered, this, &QRVizCloudAnnotation::onControlPointWeightMax);
    }

    label_menu->addSeparator();

    // weight submenu
    {
      m_bias_menu = label_menu->addMenu("调节Bbox大小");
      m_bias_menu->setEnabled(true);

      m_X1_bias_action = new QAction("X+增加", menu_bar);
      m_bias_menu->addAction(m_X1_bias_action);
      m_X1_bias_action->setShortcut(QKeySequence("A"));
      connect(m_X1_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasX1);

      m_X2_bias_action = new QAction("X+减小", menu_bar);
      m_bias_menu->addAction(m_X2_bias_action);
      m_X2_bias_action->setShortcut(QKeySequence("Shift+A"));
      connect(m_X2_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasX2);

      m_X3_bias_action = new QAction("X-增加", menu_bar);
      m_bias_menu->addAction(m_X3_bias_action);
      m_X3_bias_action->setShortcut(QKeySequence("D"));
      connect(m_X3_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasX3);

      m_X4_bias_action = new QAction("X-减小", menu_bar);
      m_bias_menu->addAction(m_X4_bias_action);
      m_X4_bias_action->setShortcut(QKeySequence("Shift+D"));
      connect(m_X4_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasX4);

      m_Y1_bias_action = new QAction("Y+增加", menu_bar);
      m_bias_menu->addAction(m_Y1_bias_action);
      m_Y1_bias_action->setShortcut(QKeySequence("W"));
      connect(m_Y1_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasY1);

      m_Y2_bias_action = new QAction("Y+减小", menu_bar);
      m_bias_menu->addAction(m_Y2_bias_action);
      m_Y2_bias_action->setShortcut(QKeySequence("Shift+W"));
      connect(m_Y2_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasY2);

      m_Y3_bias_action = new QAction("Y-增加", menu_bar);
      m_bias_menu->addAction(m_Y3_bias_action);
      m_Y3_bias_action->setShortcut(QKeySequence("S"));
      connect(m_Y3_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasY3);

      m_Y4_bias_action = new QAction("Y-减小", menu_bar);
      m_bias_menu->addAction(m_Y4_bias_action);
      m_Y4_bias_action->setShortcut(QKeySequence("Shift+S"));
      connect(m_Y4_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasY4);

      m_Z1_bias_action = new QAction("Z+增加", menu_bar);
      m_bias_menu->addAction(m_Z1_bias_action);
      m_Z1_bias_action->setShortcut(QKeySequence("Q"));
      connect(m_Z1_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasZ1);

      m_Z2_bias_action = new QAction("Z+减小", menu_bar);
      m_bias_menu->addAction(m_Z2_bias_action);
      m_Z2_bias_action->setShortcut(QKeySequence("Shift+Q"));
      connect(m_Z2_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasZ2);

      m_Z3_bias_action = new QAction("Z-增加", menu_bar);
      m_bias_menu->addAction(m_Z3_bias_action);
      m_Z3_bias_action->setShortcut(QKeySequence("E"));
      connect(m_Z3_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasZ3);

      m_Z4_bias_action = new QAction("Z-减小", menu_bar);
      m_bias_menu->addAction(m_Z4_bias_action);
      m_Z4_bias_action->setShortcut(QKeySequence("Shift+E"));
      connect(m_Z4_bias_action, &QAction::triggered, this, &QRVizCloudAnnotation::onBiasZ4);
    }

    label_menu->addSeparator();
    {
      m_yaw_menu = label_menu->addMenu("方位");
      m_yaw_menu->setEnabled(false);

      m_prev_yaw_action = new QAction("减小", menu_bar);
      m_yaw_menu->addAction(m_prev_yaw_action);
      m_prev_yaw_action->setShortcut(QKeySequence("R"));
      connect(m_prev_yaw_action, &QAction::triggered, this, &QRVizCloudAnnotation::onYawDec);

      m_next_yaw_action = new QAction("增加", menu_bar);
      m_yaw_menu->addAction(m_next_yaw_action);
      m_next_yaw_action->setShortcut(QKeySequence("F"));
      connect(m_next_yaw_action, &QAction::triggered, this, &QRVizCloudAnnotation::onYawInc);
    }
  }

  {
    QBoxLayout *annotation_type_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(annotation_type_layout);
    m_annotation_type_group = new QButtonGroup(this);

    QLabel *title1_label = new QLabel("标注类别：", this);
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::red);
    title1_label->setPalette(palette);
    annotation_type_layout->addWidget(title1_label);

    m_bbox_button = new QPushButton(" 障碍物 F1", this);
    m_bbox_button->setCheckable(true);
    m_bbox_button->setChecked(true);
    m_bbox_button->setShortcut(QKeySequence("F1"));
    annotation_type_layout->addWidget(m_bbox_button);
    m_annotation_type_group->addButton(m_bbox_button, ANNOTATION_TYPE_BBOX);

    m_plane_button = new QPushButton(" 地面 F2", this);
    m_plane_button->setCheckable(true);
    m_plane_button->setShortcut(QKeySequence("F2"));
    annotation_type_layout->addWidget(m_plane_button);
    m_annotation_type_group->addButton(m_plane_button, ANNOTATION_TYPE_PLANE);

    m_kerb_button = new QPushButton(" 路沿 F3", this);
    m_kerb_button->setCheckable(true);
    m_kerb_button->setShortcut(QKeySequence("F3"));
    annotation_type_layout->addWidget(m_kerb_button);
    m_annotation_type_group->addButton(m_kerb_button, ANNOTATION_TYPE_KERB);

    m_lane_button = new QPushButton(" 车道线 F4", this);
    m_lane_button->setCheckable(true);
    m_lane_button->setShortcut(QKeySequence("F4"));
    annotation_type_layout->addWidget(m_lane_button);
    m_annotation_type_group->addButton(m_lane_button, ANNOTATION_TYPE_LANE);

    void (QButtonGroup::*button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
    connect(m_annotation_type_group, button_clicked_function_pointer, this, &QRVizCloudAnnotation::onSetAnnotationType);
  }

  {
    QBoxLayout *toolbar_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(toolbar_layout);
    m_toolbar_group = new QButtonGroup(this);

    QLabel *title1_label = new QLabel("操作模式：", this);
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::red);
    title1_label->setPalette(palette);
    toolbar_layout->addWidget(title1_label);

    m_edit_none_button = new QPushButton("移动缩放视角 1", this);
    m_edit_none_button->setCheckable(true);
    m_edit_none_button->setChecked(true);
    m_edit_none_button->setShortcut(QKeySequence("1"));
    toolbar_layout->addWidget(m_edit_none_button);
    m_toolbar_group->addButton(m_edit_none_button, EDIT_MODE_NONE);

    m_edit_control_point_button = new QPushButton("标记点云 2", this);
    m_edit_control_point_button->setShortcut(QKeySequence("2"));
    m_edit_control_point_button->setCheckable(true);
    toolbar_layout->addWidget(m_edit_control_point_button);
    m_toolbar_group->addButton(m_edit_control_point_button, EDIT_MODE_CONTROL_POINT);

    m_edit_eraser_button = new QPushButton("删除点云 3", this);
    m_edit_eraser_button->setShortcut(QKeySequence("3"));
    m_edit_eraser_button->setCheckable(true);
    toolbar_layout->addWidget(m_edit_eraser_button);
    m_toolbar_group->addButton(m_edit_eraser_button, EDIT_MODE_ERASER);

    // m_edit_color_picker_button = new QPushButton("选择 4", this);
    // m_edit_color_picker_button->setShortcut(QKeySequence("4"));
    // m_edit_color_picker_button->setCheckable(false);
    // toolbar_layout->addWidget(m_edit_color_picker_button);
    // m_toolbar_group->addButton(m_edit_color_picker_button, EDIT_MODE_COLOR_PICKER);

    void (QButtonGroup::*button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
    connect(m_toolbar_group, button_clicked_function_pointer, this, &QRVizCloudAnnotation::onSetEditMode);
  }

  {
    QBoxLayout *tooltype_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(tooltype_layout);

    QLabel *title1_label = new QLabel("选择方式：", this);
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::red);
    title1_label->setPalette(palette);
    tooltype_layout->addWidget(title1_label);

    m_tooltype_group = new QButtonGroup(this);
    m_tool_single_button = new QPushButton("点 选择 4", this);
    m_tool_single_button->setCheckable(true);
    m_tool_single_button->setChecked(true);
    m_tool_single_button->setShortcut(QKeySequence("4"));
    tooltype_layout->addWidget(m_tool_single_button);
    m_tooltype_group->addButton(m_tool_single_button, TOOL_TYPE_SINGLE_PICK);

    // m_tool_shallow_square_button = new QPushButton("矩形_浅 6", this);
    // m_tool_shallow_square_button->setCheckable(true);
    // m_tool_shallow_square_button->setShortcut(QKeySequence("6"));
    // tooltype_layout->addWidget(m_tool_shallow_square_button);
    // m_tooltype_group->addButton(m_tool_shallow_square_button, TOOL_TYPE_SHALLOW_SQUARE);

    m_tool_deep_square_button = new QPushButton("矩形 选择 5", this);
    m_tool_deep_square_button->setCheckable(true);
    m_tool_deep_square_button->setShortcut(QKeySequence("5"));
    tooltype_layout->addWidget(m_tool_deep_square_button);
    m_tooltype_group->addButton(m_tool_deep_square_button, TOOL_TYPE_DEEP_SQUARE);

    m_tool_shallow_poly_button = new QPushButton("多边形 选择 6", this);
    m_tool_shallow_poly_button->setCheckable(true);
    m_tool_shallow_poly_button->setShortcut(QKeySequence("6"));
    tooltype_layout->addWidget(m_tool_shallow_poly_button);
    m_tooltype_group->addButton(m_tool_shallow_poly_button, TOOL_TYPE_SHALLOW_POLY);

    void (QButtonGroup::*button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
    connect(m_tooltype_group, button_clicked_function_pointer, this, &QRVizCloudAnnotation::onSetToolType);
  }

  {
    QBoxLayout *bbox_type = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(bbox_type);
    m_toolbar_group = new QButtonGroup(this);

    QLabel *title1_label = new QLabel("障碍物类别：[1-5]-小车   [6-10]-大车   [11-15]-行人   [16-20]-骑行", this);
    QPalette palette;
    palette.setColor(QPalette::Text, Qt::red);
    title1_label->setPalette(palette);
    bbox_type->addWidget(title1_label);
  }

  {
    QGridLayout *page_layout = new QGridLayout();
    main_layout->addLayout(page_layout);

    page_layout->setVerticalSpacing(1);
    page_layout->setHorizontalSpacing(1);

    m_page_button_group = new QButtonGroup(this);

    m_page_buttons.resize(m_color_cols_per_page * m_color_rows_per_page);
    for (uint64 y = 0; y < m_color_rows_per_page; y++)
      for (uint64 x = 0; x < m_color_cols_per_page; x++)
      {
        QPushButton *button = new QPushButton(this);
        const uint64 id = x + y * m_color_cols_per_page;
        m_page_buttons[id] = button;
        button->setCheckable(true);
        m_page_button_group->addButton(button, id + 1);
        button->setMinimumWidth(1);
        page_layout->addWidget(button, y, x);
      }

    // this is needed since buttonClicked is overloaded (we must specify which one)
    void (QButtonGroup::*button_clicked_function_pointer)(int) = &QButtonGroup::buttonClicked;
    connect(m_page_button_group, button_clicked_function_pointer, this, &QRVizCloudAnnotation::onLabelButtonSelected);
  }

  {
    QBoxLayout *page_shift_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(page_shift_layout);

    m_current_page_label = new QLabel("None", this);
    m_current_page_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    page_shift_layout->addWidget(m_current_page_label);

    m_object_id = new QLabel("第1个障碍物", this);
    m_object_id->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    page_shift_layout->addWidget(m_object_id);

    QLabel *view_label = new QLabel("显示:", this);
    view_label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    page_shift_layout->addWidget(view_label);

    QToolButton *view_cloud_button = new QToolButton(this);
    view_cloud_button->setText("点云");
    view_cloud_button->setCheckable(true);
    view_cloud_button->setChecked(true);
    view_cloud_button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    page_shift_layout->addWidget(view_cloud_button);
    connect(view_cloud_button, &QToolButton::toggled, this, &QRVizCloudAnnotation::onViewCloudToggled);

    QToolButton *view_labels_button = new QToolButton(this);
    view_labels_button->setText("标记");
    view_labels_button->setCheckable(true);
    view_labels_button->setChecked(true);
    view_labels_button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    page_shift_layout->addWidget(view_labels_button);
    connect(view_labels_button, &QToolButton::toggled, this, &QRVizCloudAnnotation::onViewLabelsToggled);

    QToolButton *view_control_points_button = new QToolButton(this);
    view_control_points_button->setText("索引");
    view_control_points_button->setCheckable(true);
    view_control_points_button->setChecked(true);
    view_control_points_button->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    page_shift_layout->addWidget(view_control_points_button);
    connect(view_control_points_button, &QToolButton::toggled, this, &QRVizCloudAnnotation::onViewControlPointsToggled);
  }

  {
    QBoxLayout *control_point_weight_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(control_point_weight_layout);

    QLabel *weight_label = new QLabel("遮挡系数(%):", this);
    weight_label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    control_point_weight_layout->addWidget(weight_label);

    m_current_control_point_weight_label = new QLabel("---/---", this);
    m_current_control_point_weight_label->setFixedWidth(m_current_control_point_weight_label->sizeHint().width());
    m_current_control_point_weight_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    m_current_control_point_weight_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    control_point_weight_layout->addWidget(m_current_control_point_weight_label);

    m_current_control_point_weight_slider = new QSlider(Qt::Horizontal, this);
    QSlider *&weight_slider = m_current_control_point_weight_slider;
    weight_slider->setFocusPolicy(Qt::NoFocus);
    weight_slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    weight_slider->setMinimum(0);
    weight_slider->setMaximum(100);
    weight_slider->setValue(0);
    weight_slider->setPageStep(1);
    weight_slider->setEnabled(false);
    weight_slider->setTracking(false);
    weight_slider->setSingleStep(1);
    weight_slider->setTickPosition(QSlider::NoTicks);
    control_point_weight_layout->addWidget(weight_slider);
    connect(weight_slider, &QSlider::sliderMoved, this, &QRVizCloudAnnotation::onControlPointWeightSliderMoved);
    connect(weight_slider, &QSlider::valueChanged, this, &QRVizCloudAnnotation::onControlPointWeightSliderSet);
    connect(weight_slider, &QSlider::valueChanged, this, &QRVizCloudAnnotation::onControlPointWeightSliderMoved);
  }

  {
    QBoxLayout *control_yaw_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(control_yaw_layout);

    QLabel *yaw_label = new QLabel("方位角(yaw):", this);
    yaw_label->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
    control_yaw_layout->addWidget(yaw_label);

    m_current_yaw_label = new QLabel("---/---", this);
    m_current_yaw_label->setFixedWidth(m_current_yaw_label->sizeHint().width());
    m_current_yaw_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    m_current_yaw_label->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
    control_yaw_layout->addWidget(m_current_yaw_label);

    m_current_yaw_slider = new QSlider(Qt::Horizontal, this);
    QSlider *&yaw_slider = m_current_yaw_slider;
    yaw_slider->setFocusPolicy(Qt::NoFocus);
    yaw_slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    yaw_slider->setMinimum(-180);
    yaw_slider->setMaximum(180);
    yaw_slider->setValue(0);
    yaw_slider->setPageStep(1);
    yaw_slider->setEnabled(false);
    yaw_slider->setTracking(false);
    yaw_slider->setSingleStep(5);
    yaw_slider->setTickPosition(QSlider::NoTicks);
    control_yaw_layout->addWidget(yaw_slider);
    connect(yaw_slider, &QSlider::sliderMoved, this, &QRVizCloudAnnotation::onControlYawSliderMoved);
    connect(yaw_slider, &QSlider::valueChanged, this, &QRVizCloudAnnotation::onControlYawSliderSet);
    connect(yaw_slider, &QSlider::valueChanged, this, &QRVizCloudAnnotation::onControlYawSliderMoved);
  }

  {
    QBoxLayout *set_name_layout = new QBoxLayout(QBoxLayout::LeftToRight);
    main_layout->addLayout(set_name_layout);

    QPushButton *set_name_button = new QPushButton("输入数据集路径: ", this);
    set_name_layout->addWidget(set_name_button);

    m_set_name_edit = new QLineEdit("", this);
    set_name_layout->addWidget(m_set_name_edit);
    connect(m_set_name_edit, SIGNAL(returnPressed()), set_name_button,
            SLOT(animateClick())); // won't work with qt5 syntax
    connect(set_name_button, &QPushButton::clicked, this, &QRVizCloudAnnotation::onSendName);
  }

  m_current_edit_mode = 1;
  SetCurrentEditMode(EDIT_MODE_NONE);

  m_current_page = 1;
  m_current_label = 0;
  SetCurrentLabel(1, 0);

  FillColorPageButtonStylesheet();
} // namespace rviz_cloud_annotation

QRVizCloudAnnotation::~QRVizCloudAnnotation()
{
}

void QRVizCloudAnnotation::onNextObject()
{
  m_next_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onPreObject()
{
  m_pre_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onUndo()
{
  m_undo_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onRedo()
{
  m_redo_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onUndoRedoState(const rviz_cloud_annotation::UndoRedoState &msg)
{
  m_undo_action->setEnabled(msg.undo_enabled);
  SetUndoText(msg.undo_description);
  m_redo_action->setEnabled(msg.redo_enabled);
  SetRedoText(msg.redo_description);
}

void QRVizCloudAnnotation::SetUndoText(const std::string &text)
{
  std::string t = "取消";
  // if (!text.empty())
  //    t += " (" + text + ")";
  m_undo_action->setText(t.c_str());
}

void QRVizCloudAnnotation::SetRedoText(const std::string &text)
{
  std::string t = "恢复";
  // if (!text.empty())
  //    t += " (" + text + ")";
  m_redo_action->setText(t.c_str());
}

void QRVizCloudAnnotation::onSetName(const std_msgs::String &name)
{
  const std::string n = name.data;
  m_set_name_edit->setText(QString::fromUtf8(n.c_str()));
}

void QRVizCloudAnnotation::onSendName()
{
  std_msgs::String msg;
  msg.data = m_set_name_edit->text().toUtf8().constData();
  m_set_name_pub.publish(msg);
}

void QRVizCloudAnnotation::onViewCloudToggled(const bool checked)
{
  std_msgs::Bool msg;
  msg.data = checked;
  m_view_cloud_pub.publish(msg);
}

void QRVizCloudAnnotation::onViewControlPointsToggled(const bool checked)
{
  std_msgs::Bool msg;
  msg.data = checked;
  m_view_control_points_pub.publish(msg);
}

void QRVizCloudAnnotation::onViewLabelsToggled(const bool checked)
{
  std_msgs::Bool msg;
  msg.data = checked;
  m_view_labels_pub.publish(msg);
}

void QRVizCloudAnnotation::onPointCountUpdate(const std_msgs::UInt64MultiArray &counters)
{
  const uint64 count = counters.data.size() / 2;
  for (uint64 i = 0; i < count; i++)
  {
    const uint64 index = counters.data[i * 2];
    const uint64 value = counters.data[i * 2 + 1];

    if (index > m_point_counters.size())
      m_point_counters.resize(index, 0);
    m_point_counters[index - 1] = value;
  }

  // FillPointCounts();
}

void QRVizCloudAnnotation::onControlPointWeightInc()
{
  const uint32 value = m_current_control_point_weight_slider->value();
  if (value >= m_control_point_weight_max)
    return;
  m_current_control_point_weight_slider->setValue(value + 1);
}

void QRVizCloudAnnotation::onControlPointWeightDec()
{
  const uint32 value = m_current_control_point_weight_slider->value();
  if (value == 0)
    return;
  m_current_control_point_weight_slider->setValue(value - 1);
}

void QRVizCloudAnnotation::onControlPointWeightMax()
{
  m_current_control_point_weight_slider->setValue(m_control_point_weight_max);
}

void QRVizCloudAnnotation::onControlPointWeightMin()
{
  m_current_control_point_weight_slider->setValue(0);
}

void QRVizCloudAnnotation::onControlPointWeightSliderMoved(int new_value)
{
  const std::string text =
      boost::lexical_cast<std::string>(new_value) + "/" + boost::lexical_cast<std::string>(m_control_point_weight_max);
  m_current_control_point_weight_label->setText(text.c_str());
}

void QRVizCloudAnnotation::onControlPointWeightSliderSet(int new_value)
{
  m_prev_weight_action->setEnabled(uint32(new_value) > 0);
  m_next_weight_action->setEnabled(uint32(new_value) < m_control_point_weight_max);
  m_max_weight_action->setEnabled(uint32(new_value) < m_control_point_weight_max);
  m_min_weight_action->setEnabled(uint32(new_value) > 0);

  std_msgs::Int32 msg;
  msg.data = new_value;
  // m_control_points_weight_pub.publish(msg);
  m_bbox_occluded_pub.publish(msg);
}

void QRVizCloudAnnotation::onYawInc()
{
  const int32 value = m_current_yaw_slider->value();
  if (value >= m_yaw_max)
    return;
  m_current_yaw_slider->setValue(value + 1);
}

void QRVizCloudAnnotation::onYawDec()
{
  const int32 value = m_current_yaw_slider->value();
  if (value <= m_yaw_min)
    return;
  m_current_yaw_slider->setValue(value - 1);
}

void QRVizCloudAnnotation::onControlYawSliderMoved(int new_value)
{
  const std::string text = boost::lexical_cast<std::string>(new_value) + "/" + "[" +
                           boost::lexical_cast<std::string>(m_yaw_min) + "," +
                           boost::lexical_cast<std::string>(m_yaw_max) + "]";
  m_current_yaw_label->setText(text.c_str());
  m_current_yaw_label->setFixedWidth(m_current_yaw_label->sizeHint().width());
}

void QRVizCloudAnnotation::onControlYawSliderSet(int new_value)
{
  m_prev_yaw_action->setEnabled(int32(new_value) > m_yaw_min);
  m_next_yaw_action->setEnabled(int32(new_value) < m_yaw_max);
  // m_max_yaw_action->setEnabled(uint32(new_value) < m_yaw_max);
  // m_min_yaw_action->setEnabled(uint32(new_value) > 0);

  std_msgs::Int32 msg;
  msg.data = new_value;

  m_yaw_pub.publish(msg);
}

void QRVizCloudAnnotation::onSetControlMaxYaw(const std_msgs::Int32 &msg)
{
  m_yaw_max = msg.data;

  // compute maximum length of label
  {
    const std::string number = boost::lexical_cast<std::string>(m_yaw_max);
    // const std::string number_zeros(number.size(), '0');
    // const std::string text = number_zeros + "/" + number_zeros;
    const std::string text = boost::lexical_cast<std::string>(0) + "/" + "[" +
                             boost::lexical_cast<std::string>(m_yaw_min) + "," +
                             boost::lexical_cast<std::string>(m_yaw_max) + "]";
    m_current_yaw_label->setText(text.c_str());
    m_current_yaw_label->setFixedWidth(m_current_yaw_label->sizeHint().width());
  }

  m_current_yaw_slider->setMaximum(m_yaw_max);

  m_current_yaw_slider->setEnabled(true);

  // onControlYawSliderMoved(m_yaw_min);

  m_yaw_menu->setEnabled(true);

  ROS_INFO("rviz_cloud_annotation_plugin: set max yaw to %i", (signed int)(m_yaw_max));
}

void QRVizCloudAnnotation::onSetControlMinYaw(const std_msgs::Int32 &msg)
{
  m_yaw_min = msg.data;

  // compute maximum length of label
  {
    const std::string number = boost::lexical_cast<std::string>(m_yaw_min);
    const std::string number_zeros(number.size(), '0');
    // const std::string text = number_zeros + "/" + number_zeros;
    const std::string text = boost::lexical_cast<std::string>(0) + "/" + "[" +
                             boost::lexical_cast<std::string>(m_yaw_min) + "," +
                             boost::lexical_cast<std::string>(m_yaw_max) + "]";
    m_current_yaw_label->setText(text.c_str());
    m_current_yaw_label->setFixedWidth(m_current_yaw_label->sizeHint().width());
  }

  m_current_yaw_slider->setMinimum(m_yaw_min);
  m_current_yaw_slider->setEnabled(true);

  onControlYawSliderMoved((m_yaw_min + m_yaw_max) / 2);

  m_yaw_menu->setEnabled(true);

  ROS_INFO("rviz_cloud_annotation_plugin: set min yaw to %i", (signed int)(m_yaw_min));
}

void QRVizCloudAnnotation::onSetControlPointMaxWeight(const std_msgs::Int32 &msg)
{
  m_control_point_weight_max = msg.data;

  // compute maximum length of label
  {
    const std::string number = boost::lexical_cast<std::string>(m_control_point_weight_max);
    const std::string number_zeros(number.size(), '0');
    const std::string text = number_zeros + "/" + number_zeros;
    m_current_control_point_weight_label->setText(text.c_str());
    m_current_control_point_weight_label->setFixedWidth(m_current_control_point_weight_label->sizeHint().width());
  }

  m_current_control_point_weight_slider->setMaximum(m_control_point_weight_max);
  m_current_control_point_weight_slider->setValue(0);
  m_current_control_point_weight_slider->setEnabled(true);
  onControlPointWeightSliderMoved(0);

  m_weight_menu->setEnabled(true);

  ROS_INFO("rviz_cloud_annotation_plugin: set max weight to %i", (unsigned int)(m_control_point_weight_max));

  showImageLabel("/home/halo/lidar_annotation/image/label.png");
}

void QRVizCloudAnnotation::onSetBiasZero(const std_msgs::Empty &msg)
{
  m_bias.push_back(0);
  m_bias.push_back(0);
  m_bias.push_back(0);
  m_bias.push_back(0);
  m_bias.push_back(0);
  m_bias.push_back(0);
}
void QRVizCloudAnnotation::onSetObjectId(const std_msgs::Int32 &msg)
{
  if (ANNOTATION_TYPE == BBOX)
  {
    bbox_id = msg.data;
    std::string m_str_ = "第" + std::to_string(bbox_id + 1) + "个障碍物";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == KERB)
  {
    kerb_id = msg.data;
    std::string m_str_ = "第" + std::to_string(kerb_id + 1) + "条路沿";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == LANE)
  {
    lane_id = msg.data;
    std::string m_str_ = "第" + std::to_string(lane_id + 1) + "条车道线";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == PLANE)
  {
    std::string m_str_ = " ";
    m_object_id->setText(m_str_.c_str());
  }
}

void QRVizCloudAnnotation::onChangeObjectId()
{
  if (ANNOTATION_TYPE == BBOX)
  {
    std::string m_str_ = "第" + std::to_string(bbox_id + 1) + "个障碍物";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == KERB)
  {
    std::string m_str_ = "第" + std::to_string(kerb_id + 1) + "条路沿";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == LANE)
  {
    std::string m_str_ = "第" + std::to_string(lane_id + 1) + "条车道线";
    m_object_id->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == PLANE)
  {
    std::string m_str_ = " ";
    m_object_id->setText(m_str_.c_str());
  }
}

void QRVizCloudAnnotation::ColorToHex(const pcl::RGB &color, char color_hex[7])
{
  static const char HEX[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
  color_hex[0] = HEX[color.r / 16];
  color_hex[1] = HEX[color.r % 16];
  color_hex[2] = HEX[color.g / 16];
  color_hex[3] = HEX[color.g % 16];
  color_hex[4] = HEX[color.b / 16];
  color_hex[5] = HEX[color.b % 16];
  color_hex[6] = 0;
}

void QRVizCloudAnnotation::FillColorPageButtonStylesheet()
{
  std::ostringstream stylesheet;
  stylesheet << "QPushButton[ColorPageButton=\"true\"] {\n";
  stylesheet << "  min-width: 1em;\n";
  stylesheet << "  padding: 2px;\n";
  stylesheet << "  margin: 1px;\n";
  stylesheet << "}\n\n";

  for (uint64 i = 0; i < 256; i++)
  {
    stylesheet << "QPushButton[ColorPageButtonColorId=\"" << i << "\"] {\n";

    const pcl::RGB color = pcl::GlasbeyLUT::at(i);

    const uint64 luminance = color.r / 3 + color.g / 2 + color.b / 6; // approximate
    const char *text_color = "black";
    if (luminance < 80)
      text_color = "white";

    char color_hex[7];
    ColorToHex(color, color_hex);
    stylesheet << "  color: " << text_color << ";\n";
    stylesheet << "  background-color: #" << color_hex << ";\n";
    stylesheet << "}\n\n";

    stylesheet << "QPushButton[ColorPageButton=\"true\"][ColorPageButtonColorId=\"" << i << "\"]:checked {\n";
    stylesheet << "  color: black;\n";
    stylesheet << "}\n\n";
  }

  const std::string str = stylesheet.str();
  setStyleSheet(str.c_str());
}

void QRVizCloudAnnotation::FillPointCounts(uint type)
{
  const uint64 page_size = m_color_cols_per_page * m_color_rows_per_page;
  for (uint64 i = 0; i < page_size; i++)
  {
    const uint64 label = i + m_current_page * page_size + 1;

    const uint64 count = (label > m_point_counters.size()) ? 0 : m_point_counters[label - 1];
    const std::string str = boost::lexical_cast<std::string>(count);
    // m_page_buttons[i]->setText(str.c_str());

    //映射为kitti label
    if (type == BBOX)
    {
      uint64 m_label_ = (label - 1) % (page_size);
      m_label_ = m_label_ / 5;
      std::string m_str_;
      switch (m_label_)
      {
      case 0:
        m_str_ = "小车";
        break;
      case 1:
        m_str_ = "大车";
        break;
      case 2:
        m_str_ = "行人";
        break;
      case 3:
        m_str_ = "骑行";
        break;
      }
      m_page_buttons[i]->setText(m_str_.c_str());
    }
    else if (type == LANE)
    {
      std::string m_str_ = "车道";
      m_page_buttons[i]->setText(m_str_.c_str());
    }
    else if (type == KERB)
    {
      std::string m_str_ = "路沿";
      m_page_buttons[i]->setText(m_str_.c_str());
    }
    else if (type == PLANE)
    {
      std::string m_str_ = "地面";
      m_page_buttons[i]->setText(m_str_.c_str());
    }
  }
}

void QRVizCloudAnnotation::FillColorPageButtons()
{
  const uint64 size = m_page_buttons.size();

  for (uint64 i = 0; i < size; i++)
  {
    m_page_buttons[i]->setProperty("ColorPageButton", true);
    m_page_buttons[i]->setProperty("ColorPageButtonColorId", int((i + m_current_page * size) % 256));
    style()->unpolish(m_page_buttons[i]);
    style()->polish(m_page_buttons[i]);
    update();
  }
}

void QRVizCloudAnnotation::onSetCurrentLabel(const std_msgs::UInt32 &label)
{
  const uint64 new_label = label.data;
  if (new_label == m_current_label)
    return;

  SetCurrentLabel(new_label, GetPageForLabel(new_label));
}

void QRVizCloudAnnotation::onSetEditMode2(const std_msgs::UInt32 &mode)
{
  SetCurrentEditMode(mode.data);
}

void QRVizCloudAnnotation::onSetEditMode(int edit_mode)
{
  SetCurrentEditMode(edit_mode);
}

void QRVizCloudAnnotation::onSetToolType(int tool_type)
{
  std_msgs::UInt32 msg;
  msg.data = tool_type;
  m_tool_type_pub.publish(msg);
}

void QRVizCloudAnnotation::onSetAnnotationType(uint32 annotation_type)
{
  std_msgs::UInt32 msg;
  msg.data = annotation_type;
  m_annotation_type_pub.publish(msg);
  ANNOTATION_TYPE = annotation_type;
  /*
  if (ANNOTATION_TYPE == BBOX)
  {
    m_tool_shallow_poly_button->setChecked(true);
    m_tool_deep_square_button->setChecked(false);
    m_tool_single_button->setChecked(false);
    m_tool_shallow_square_button->setChecked(false);

    m_tool_shallow_poly_button->setCheckable(true);
    m_tool_deep_square_button->setCheckable(true);
    m_tool_single_button->setCheckable(false);
    m_tool_shallow_square_button->setCheckable(true);
  }
  else if (ANNOTATION_TYPE == PLANE)
  {
    m_tool_shallow_poly_button->setChecked(true);
    m_tool_deep_square_button->setChecked(false);
    m_tool_single_button->setChecked(false);
    m_tool_shallow_square_button->setChecked(false);

    m_tool_shallow_poly_button->setCheckable(true);
    m_tool_deep_square_button->setCheckable(true);
    m_tool_single_button->setCheckable(false);
    m_tool_shallow_square_button->setCheckable(true);
  }
  else if (ANNOTATION_TYPE == KERB)
  {
    m_tool_shallow_poly_button->setChecked(false);
    m_tool_deep_square_button->setChecked(false);
    m_tool_single_button->setChecked(true);
    m_tool_shallow_square_button->setChecked(false);

    m_tool_shallow_poly_button->setCheckable(false);
    m_tool_deep_square_button->setCheckable(false);
    m_tool_single_button->setCheckable(true);
    m_tool_shallow_square_button->setCheckable(false);
  }
  else if (ANNOTATION_TYPE == LANE)
  {
    m_tool_shallow_poly_button->setChecked(false);
    m_tool_deep_square_button->setChecked(false);
    m_tool_single_button->setChecked(true);
    m_tool_shallow_square_button->setChecked(false);
    m_tool_shallow_poly_button->setCheckable(false);
    m_tool_deep_square_button->setCheckable(false);
    m_tool_single_button->setCheckable(true);
    m_tool_shallow_square_button->setCheckable(false);
  }
  */
  SetCurrentLabel(m_current_label, GetPageForLabel(m_current_label));
  onChangeObjectId();
}

void QRVizCloudAnnotation::SetCurrentEditMode(const uint64 mode)
{
  if (m_current_edit_mode == mode)
    return;

  m_current_edit_mode = mode;

  std_msgs::UInt32 edit_mode_out;
  edit_mode_out.data = mode;
  m_set_edit_mode_pub.publish(edit_mode_out);

  switch (mode)
  {
  case EDIT_MODE_NONE:
    if (!m_edit_none_button->isChecked())
      m_edit_none_button->setChecked(true);
    break;
  case EDIT_MODE_CONTROL_POINT:
    if (!m_edit_control_point_button->isChecked())
      m_edit_control_point_button->setChecked(true);
    break;
  case EDIT_MODE_ERASER:
    if (!m_edit_eraser_button->isChecked())
      m_edit_eraser_button->setChecked(true);
    break;
  case EDIT_MODE_COLOR_PICKER:
    if (!m_edit_color_picker_button->isChecked())
      m_edit_color_picker_button->setChecked(true);
    break;
  default:
    break;
  }
}

void QRVizCloudAnnotation::onLabelButtonSelected(int id)
{
  SetCurrentLabel(GetLabelFromPageAndId(m_current_page, id), m_current_page);
}

void QRVizCloudAnnotation::onPlusLabel()
{
  SetCurrentLabel(m_current_label + 1, GetPageForLabel(m_current_label + 1));
}

void QRVizCloudAnnotation::onMinusLabel()
{
  if (m_current_label > 1)
    SetCurrentLabel(m_current_label - 1, GetPageForLabel(m_current_label - 1));
}

void QRVizCloudAnnotation::onAutoPlane()
{
  m_auto_plane_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onPageDown()
{
  SetCurrentLabel(GetFirstLabelForPage(m_current_page + 1), m_current_page + 1);
}

void QRVizCloudAnnotation::onPageUp()
{
  if (m_current_page > 0)
    SetCurrentLabel(GetFirstLabelForPage(m_current_page - 1), m_current_page - 1);
}

void QRVizCloudAnnotation::onGotoFirstUnused()
{
  m_goto_first_unused_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onGotoLastUnused()
{
  m_goto_last_unused_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onGotoFirst()
{
  m_goto_first_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onGotoNextUnused()
{
  m_goto_next_unused_pub.publish(std_msgs::Empty());
}

void QRVizCloudAnnotation::onBiggerPoints()
{
  std_msgs::Int32 msg;
  msg.data = POINT_SIZE_CHANGE_BIGGER;
  m_point_size_change_pub.publish(msg);
}

void QRVizCloudAnnotation::onSmallerPoints()
{
  std_msgs::Int32 msg;
  msg.data = POINT_SIZE_CHANGE_SMALLER;
  m_point_size_change_pub.publish(msg);
}

void QRVizCloudAnnotation::onResetPointsSize()
{
  std_msgs::Int32 msg;
  msg.data = POINT_SIZE_CHANGE_RESET;
  m_point_size_change_pub.publish(msg);
}

void QRVizCloudAnnotation::onSave()
{
  std_msgs::String filename; // NYI: select filename from GUI
  m_save_pub.publish(filename);
}

void QRVizCloudAnnotation::onRestore()
{
  const QMessageBox::StandardButton result = QMessageBox::question(this, "Restore", "确定恢复到上一次保存的状态?\n"
                                                                                    "当前进度将被清除!",
                                                                   QMessageBox::Yes | QMessageBox::No);
  if (result == QMessageBox::Yes)
  {
    std_msgs::String filename; // NYI: select filename from GUI
    m_restore_pub.publish(filename);
  }
}

void QRVizCloudAnnotation::onClear()
{
  const QMessageBox::StandardButton result = QMessageBox::question(this, "New", "确定清除所有标记?\n"
                                                                                "所有标记信息都将清除！",
                                                                   QMessageBox::Yes | QMessageBox::No);
  if (result == QMessageBox::Yes)
  {
    std_msgs::UInt32 label;
    label.data = 0;
    m_clear_pub.publish(label);
  }
}

void QRVizCloudAnnotation::onNew()
{
  const QMessageBox::StandardButton result = QMessageBox::question(this, "New", "是否确定打开新文件?\n"
                                                                                "当前标记状态将被保存！",
                                                                   QMessageBox::No | QMessageBox::Yes);
  if (result == QMessageBox::Yes)
  {
    std_msgs::UInt32 label;
    label.data = 0;
    m_new_pub.publish(label);

    bbox_id = 0;
    lane_id = 0;
    kerb_id = 0;
    const std_msgs::Int32 msg;
    onSetObjectId(msg);
  }
}

void QRVizCloudAnnotation::onClearCurrent()
{
  const std::string msg =
      std::string("是否确定清除当前标记 ") + boost::lexical_cast<std::string>(m_current_label) + "?";
  const QMessageBox::StandardButton result =
      QMessageBox::question(this, "Clear", msg.c_str(), QMessageBox::Yes | QMessageBox::No);
  if (result == QMessageBox::Yes)
  {
    std_msgs::UInt32 label;
    label.data = m_current_label;
    m_clear_pub.publish(label);
  }
}

QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetPageForLabel(const uint64 label) const
{
  if (label == 0)
    return 0; // should never happen

  const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
  return ((label - 1) / size);
}

QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetLabelFromPageAndId(const uint64 page, const int id) const
{
  if (id == 0)
    return 0;
  const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
  return (page * size) + id;
}

QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetFirstLabelForPage(const uint64 page) const
{
  const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
  return (page * size) + 1;
}

QRVizCloudAnnotation::uint64 QRVizCloudAnnotation::GetLastLabelForPage(const uint64 page) const
{
  const uint64 size = m_color_cols_per_page * m_color_rows_per_page;
  return (page + 1) * size;
}

void QRVizCloudAnnotation::SetCurrentLabel(const uint64 label, const uint64 page)
{
  const uint64 page_size = m_color_cols_per_page * m_color_rows_per_page;

  if (m_current_page != page)
  {
    m_current_page = page;
    FillColorPageButtons();
    FillPointCounts(ANNOTATION_TYPE);
  }

  if (label != m_current_label)
  {
    m_current_label = label;

    const uint64 current_id = (m_current_label - 1) % page_size;
    if (!m_page_buttons[current_id]->isChecked())
      m_page_buttons[current_id]->setChecked(true);

    std_msgs::UInt32 msg;
    msg.data = m_current_label;
    m_set_current_label_pub.publish(msg);
  }
  if (ANNOTATION_TYPE == BBOX)
  {
    std::ostringstream text;
    text << std::setw(3) << m_current_label << " ";
    text << std::setw(3) << GetFirstLabelForPage(m_current_page) << "-";
    text << std::setw(3) << GetLastLabelForPage(m_current_page);
    const std::string str = text.str();
    m_current_page_label->setText(str.c_str());

    //映射为kitti label
    uint64 m_label_ = m_current_label % (page_size + 1) - 1;
    m_label_ = m_label_ / 5;
    std::string m_str_;
    switch (m_label_)
    {
    case 0:
      m_str_ = "当前标记：小型车";
      break;
    case 1:
      m_str_ = "当前标记：大型车";
      break;
    case 2:
      m_str_ = "当前标记：行人";
      break;
    case 3:
      m_str_ = "当前标记：骑行者";
      break;
    }
    m_current_page_label->setText(m_str_.c_str());
  }
  else if (ANNOTATION_TYPE == KERB)
  {
    const std::string str = "当前标记：路沿";
    m_current_page_label->setText(str.c_str());
  }
  else if (ANNOTATION_TYPE == PLANE)
  {
    const std::string str = "当前标记：地面";
    m_current_page_label->setText(str.c_str());
  }
  else if (ANNOTATION_TYPE == LANE)
  {
    const std::string str = "当前标记：车道线";
    m_current_page_label->setText(str.c_str());
  }

  m_prev_page_action->setEnabled(m_current_page > 0);
  m_prev_label_action->setEnabled(m_current_label > 1);
}

void QRVizCloudAnnotation::onBiasX1()
{
  m_bias[0] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasX2()
{
  m_bias[0] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasX3()
{
  m_bias[1] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasX4()
{
  m_bias[1] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasY1()
{
  m_bias[2] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasY2()
{
  m_bias[2] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasY3()
{
  m_bias[3] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasY4()
{
  m_bias[3] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasZ1()
{
  m_bias[4] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasZ2()
{
  m_bias[4] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasZ3()
{
  m_bias[5] += bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}
void QRVizCloudAnnotation::onBiasZ4()
{
  m_bias[5] -= bias_step;
  std_msgs::Float32MultiArray bias;
  bias.data = m_bias;
  m_bias_pub.publish(bias);
}

void QRVizCloudAnnotation::showImageLabel(std::string imagePath)
{
  cv::Mat image = cv::imread(imagePath.c_str(), CV_LOAD_IMAGE_COLOR);
  if (image.empty())
  {
    std::cout << "open image error" << std::endl;
  }
  else
  {
    std::cout << "open image ok" << std::endl;
  }
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  m_image_label_pub.publish(msg);
}

} // namespace rviz_cloud_annotation

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::QRVizCloudAnnotation, rviz::Panel);
