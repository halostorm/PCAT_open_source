#include "rviz_cloud_annotation_class.h"

RVizCloudAnnotation::RVizCloudAnnotation(ros::NodeHandle &nh1) : m_nh(nh1) {
  nh = &nh1;
  std::string param_stringA;
  std::string param_stringA1;
  std::string param_stringB;
  std::string param_stringC;
  std::string param_stringD;
  std::string param_stringE;
  std::string param_stringF;
  std::string param_stringO;

  m_nh.param<std::string>(PARAM_NAME_NORMAL_SOURCE, param_string2,
                          PARAM_DEFAULT_NORMAL_SOURCE); // 点云类型

  m_nh.param<std::string>(DATASET_FOLDER, param_stringA,
                          DATASET_FOLDER_DEFAULT); // 加载点云数据集路径

  m_dataset_folder = param_stringA;

  m_nh.param<std::string>(IMAGE_FOLDER, param_stringA1,
                          IMAGE_FOLDER_DEFAULT); // 加载图片数据集路径

  m_image_folder = param_stringA1;

  m_nh.param<std::string>(ANNOTATION_CLOUD_FOLDER, param_stringB,
                          ANNOTATION_CLOUD_FOLDER_DEFAULT); // 加载点云输出路径

  m_annotation_cloud_folder = param_stringB;

  m_nh.param<std::string>(ANNOTATION_FILE_FOLDER, param_stringC,
                          ANNOTATION_FILE_FOLDER_DEFAULT); // 加载标注记录路径

  m_annotation_file_folder = param_stringC;

  m_nh.param<std::string>(BBOX_NAME_FOLDER, param_stringD,
                          BBOX_NAME_FOLDER_DEFAULT); // 加载BBOX输出路径

  m_bbox_folder = param_stringD;

  m_nh.param<std::string>(LABEL_NAME_FOLDER, param_stringE,
                          LABEL_NAME_FOLDER_DEFAULT); // 加载标签输出路径

  m_label_folder = param_stringE;

  m_nh.param<std::string>(LINE_NAME_FOLDER, param_stringF,
                          LINE_NAME_FOLDER_DEFAULT); // 加载Line输出路径

  m_line_folder = param_stringF;

  m_nh.param<std::string>(LOG_FILE, param_stringO,
                          LOG_FILE_DEFAULT); // 加载标注日志路径

  m_log_file = param_stringO;

  ROS_INFO("rviz_cloud_annotation: log file: %s", m_log_file.c_str());
  //根据以上路径创建文件夹
  mkdir(param_stringA.c_str(), S_IRWXU);
  mkdir(param_stringA1.c_str(), S_IRWXU);
  mkdir(param_stringB.c_str(), S_IRWXU);
  mkdir(param_stringC.c_str(), S_IRWXU);
  mkdir(param_stringD.c_str(), S_IRWXU);
  mkdir(param_stringE.c_str(), S_IRWXU);

  //加载所有待标注数据集
  LoadDataSet(param_stringA, param_stringA1, param_stringB, param_stringC,
              param_stringD, param_stringE, param_stringF);
  //初始化一帧点云，并开启标记
  InitNewCloud(*nh);
  ROS_INFO("rviz_cloud_annotation: return");
}

//加载数据集及路径信息
void RVizCloudAnnotation::LoadDataSet(std::string A, std::string A1,
                                      std::string B, std::string C,
                                      std::string D, std::string E,
                                      std::string F) {
  StringVector label_files;
  std::ifstream ifile;
  ifile.open(m_log_file.c_str());

  std::string s;
  while (std::getline(ifile, s)) {
    label_files.push_back(s);
    // ROS_INFO("rviz_cloud_annotation: label_files: %s", s.c_str());
  }

  PointCloudFilesTool *pcft;
  StringVector m_pcd_files;
  pcft->getFilesList(A, m_pcd_files);
  for (int k = 0; k < m_pcd_files.size(); k++) {
    bool ifNew = true;
    std::string FILE_NAME = m_pcd_files[k];

    std::string name = A + FILE_NAME;

    for (int i = 0; i < label_files.size(); i++) {
      if (name.compare(label_files[i]) == 0) {
        ifNew = false;
        break;
      }
    }
    if (ifNew) {
      m_dataset_files.push_back(name.c_str());

      std::string index = FILE_NAME.substr(0, FILE_NAME.length() - 4);

      name = A1 + index + std::string(".png");

      m_image_files.push_back(name.c_str());

      name = B + index + std::string(".pcd");
      // ROS_INFO("rviz_cloud_annotation: Annotation Clouds: %s", name.c_str());
      m_annotation_cloud_files.push_back(name.c_str());

      name = C + index + std::string(".ann");
      // ROS_INFO("rviz_cloud_annotation: Annotation Files: %s", name.c_str());
      m_annotation_file_files.push_back(name.c_str());

      name = D + index + std::string(".txt");
      // ROS_INFO("rviz_cloud_annotation: BBox: %s", name.c_str());
      m_bbox_files.push_back(name.c_str());

      name = E + index + std::string(".txt");
      // ROS_INFO("rviz_cloud_annotation: Label: %s", name.c_str());
      m_label_files.push_back(name.c_str());
    }
  }
}

//打开新的一帧
void RVizCloudAnnotation::onNew(const std_msgs::UInt32 &label_msg) {
  //保存
  Save(false);
  //清除旧Markers
  for (int i = 0; i <= BBOX_ID; i++) {
    if (BBOX_SET[i][10] > 1) {
      EmptyBboxToMarker(i);
      ROS_INFO("rviz_cloud_annotation: Delete BBOX ID: %i", i);
    }
  }
  for (int i = 0; i <= KERB_ID; i++) {
    EmptyKerbToMarker(i);
  }
  for (int i = 0; i <= LANE_ID; i++) {
    EmptyLaneToMarker(i);
  }
  EmptyPlaneToMarker(PLANE_ID);

  if (FILE_ID < m_dataset_files.size() - 1) {
    FILE_ID++;
    //加载新的一帧并开启标注
    InitNewCloud(*nh);
  }
}

//初始化新的一帧点云，并开启标注
void RVizCloudAnnotation::InitNewCloud(ros::NodeHandle &nh) {
  //初始化Markers参数
  BBOX_ID = 0;
  KERB_ID = 0;
  LANE_ID = 0;
  PLANE_ID = 0;

  BBOX_YAW = 0;
  BBOX_YAW_ANGLE = 0;

  if_tilt = false;

  m_label.clear();

  for (int i = 0; i < BBOXNUMBER_LINEPOINTNUMBER; i++) {
    ids_in_bbox[i].clear();
    m_bbox_occluded[i] = 0;
    for (int j = 0; j < 11; j++) {
      if (j < 6) {
        m_box_bias[i][j] = 0;
      }
      BBOX_SET[i][j] = 0;
      if (j < 10) {
        BBOX_LABEL_SET[i][j] = 0;
      }
    }
  }
  //初始化Kerb
  for (int k = 0; k < LINENUMBER; k++) {
    for (int i = 0; i < KERB_SIZE[k]; i++) {
      KERB_SET[k][i][0] = 0;
      KERB_SET[k][i][1] = 0;
      KERB_SET[k][i][2] = 0;
      KERB_SET[k][i][3] = 0;
    }
    ids_in_kerb[k].clear();
    KERB_SIZE[k] = 0;
  }
  //初始化Lane
  for (int k = 0; k < LINENUMBER; k++) {
    for (int i = 0; i < LANE_SIZE[k]; i++) {
      LANE_SET[k][i][0] = 0;
      LANE_SET[k][i][1] = 0;
      LANE_SET[k][i][2] = 0;
      LANE_SET[k][i][3] = 0;
    }
    ids_in_lane[k].clear();
    LANE_SIZE[k] = 0;
  }
  //初始化Plane
  ids_in_plane.clear();
  ids_in_plane_flag.clear();

  //初始化系统参数
  std::string param_string;
  double param_double;
  int param_int;

  m_nh.param<std::string>(PARAM_NAME_UPDATE_TOPIC, param_string,
                          PARAM_DEFAULT_UPDATE_TOPIC);
  m_interactive_marker_server =
      InteractiveMarkerServerPtr(new InteractiveMarkerServer(param_string));

  m_cloud = PointXYZRGBNormalCloud::Ptr(new PointXYZRGBNormalCloud);
  if (!m_dataset_files.empty()) {
    ROS_INFO("rviz_cloud_annotation: files exist:");
    try {
      std::string file = m_dataset_files[FILE_ID];
      std::string file1 = "$(env HOME)/Path_Call.txt";
      ROS_INFO("rviz_cloud_annotation: file_now: %s", file.c_str());
      LoadCloud(file, param_string2, *m_cloud); //加载点云
    } catch (const std::string &msg) {
      ROS_FATAL("rviz_cloud_annotation: %s", msg.c_str());
      std::exit(1);
    }

    m_kdtree = KdTree::Ptr(new KdTree);
    m_kdtree->setInputCloud(m_cloud);

    {
      PointNeighborhood::Conf conf;

      m_nh.param<int>(PARAM_NAME_NEIGH_SEARCH_TYPE, param_int,
                      PARAM_DEFAULT_NEIGH_SEARCH_TYPE);
      m_nh.param<std::string>(PARAM_NAME_NEIGH_SEARCH_PARAMS, param_string,
                              PARAM_DEFAULT_NEIGH_SEARCH_PARAMS);
      if (param_int == PARAM_DEFAULT_NEIGH_SEARCH_TYPE &&
          param_string == PARAM_DEFAULT_NEIGH_SEARCH_PARAMS) {
        ROS_INFO("rviz_cloud_annotation: parameter %s is at default value, "
                 "using %s instead.",
                 PARAM_NAME_NEIGH_SEARCH_PARAMS,
                 PARAM_NAME_NEIGH_SEARCH_DISTANCE);
        m_nh.param<double>(PARAM_NAME_NEIGH_SEARCH_DISTANCE, param_double,
                           PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE);
        param_string = boost::lexical_cast<std::string>(param_double);
      }

      try {
        conf.searcher =
            PointNeighborhoodSearch::CreateFromString(param_int, param_string);
      } catch (const PointNeighborhoodSearch::ParserException &ex) {
        ROS_ERROR("rviz_cloud_annotation: could not configure point "
                  "neighborhood search from ROS param: %s",
                  ex.message.c_str());
        conf.searcher = PointNeighborhoodSearch::CreateFromString(
            PARAM_DEFAULT_NEIGH_SEARCH_TYPE,
            boost::lexical_cast<std::string>(
                PARAM_DEFAULT_NEIGH_SEARCH_DISTANCE));
      }

      m_nh.param<double>(PARAM_NAME_COLOR_IMPORTANCE, param_double,
                         PARAM_DEFAULT_COLOR_IMPORTANCE);
      conf.color_importance = param_double;

      m_nh.param<double>(PARAM_NAME_NORMAL_IMPORTANCE, param_double,
                         PARAM_DEFAULT_NORMAL_IMPORTANCE);
      conf.normal_importance = param_double;

      m_nh.param<double>(PARAM_NAME_POSITION_IMPORTANCE, param_double,
                         PARAM_DEFAULT_POSITION_IMPORTANCE);
      conf.position_importance = param_double;

      m_nh.param<double>(PARAM_NAME_MAX_DISTANCE, param_double,
                         PARAM_DEFAULT_MAX_DISTANCE);
      conf.max_distance = param_double;

      ROS_INFO("rviz_cloud_annotation: building point neighborhood...");
      m_point_neighborhood =
          PointNeighborhood::ConstPtr(new PointNeighborhood(m_cloud, conf));
      ROS_INFO("rviz_cloud_annotation: done.");
    }
  }

  m_nh.param<int>(PARAM_NAME_WEIGHT_STEPS, param_int,
                  PARAM_DEFAULT_WEIGHT_STEPS);
  m_control_point_max_weight = std::max<uint32>(1, param_int);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_WEIGHT_TOPIC, param_string,
                          PARAM_DEFAULT_CONTROL_POINT_WEIGHT_TOPIC);
  m_control_points_weight_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onControlPointWeightChange, this);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_MAX_WEIGHT_TOPIC,
                          param_string,
                          PARAM_DEFAULT_CONTROL_POINT_MAX_WEIGHT_TOPIC);
  m_control_point_weight_max_weight_pub =
      nh.advertise<std_msgs::Int32>(param_string, 1, true);

  m_nh.param<int>(PARAM_NAME_YAW_MAX, param_int, PARAM_DEFAULT_YAW_MAX);
  m_control_yaw_max = std::max<int32>(1, param_int);

  m_nh.param<int>(PARAM_NAME_YAW_MIN, param_int, PARAM_DEFAULT_YAW_MIN);
  m_control_yaw_min = std::min<int32>(1, param_int);

  RVizCloudAnnotationPoints::Ptr default_annotation =
      RVizCloudAnnotationPoints::Ptr(new RVizCloudAnnotationPoints(
          m_cloud->size(), m_control_point_max_weight, m_point_neighborhood));
  m_annotation = default_annotation;
  m_undo_redo.SetAnnotation(default_annotation);

  std::string frame_id;
  m_nh.param<std::string>(PARAM_NAME_FRAME_ID, frame_id,
                          PARAM_DEFAULT_FRAME_ID);

  std::string index;
  int2str(FILE_ID, index);
  m_frame_id = frame_id;

  ROS_INFO("rviz_cloud_annotation: %s", m_frame_id.c_str());

  m_nh.param<double>(PARAM_NAME_POINT_SIZE, param_double,
                     PARAM_DEFAULT_POINT_SIZE);
  m_point_size = param_double;

  m_nh.param<double>(PARAM_NAME_LABEL_SIZE, param_double,
                     PARAM_DEFAULT_LABEL_SIZE);
  m_label_size = param_double;

  m_nh.param<double>(PARAM_NAME_CONTROL_LABEL_SIZE, param_double,
                     PARAM_DEFAULT_CONTROL_LABEL_SIZE);
  m_control_label_size = param_double;

  m_nh.param<bool>(PARAM_NAME_SHOW_POINTS_BACK_LABELS,
                   m_show_points_back_labels,
                   PARAM_DEFAULT_SHOW_POINTS_BACK_LABELS);

  m_nh.param<std::string>(PARAM_NAME_CONTROL_POINT_VISUAL, param_string,
                          PARAM_DEFAULT_CONTROL_POINT_VISUAL);
  if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_LINE)
    m_control_points_visual = CONTROL_POINT_VISUAL_LINE;
  else if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_SPHERE)
    m_control_points_visual = CONTROL_POINT_VISUAL_SPHERE;
  else if (param_string == PARAM_VALUE_CONTROL_POINT_VISUAL_THREE_SPHERES)
    m_control_points_visual = CONTROL_POINT_VISUAL_THREE_SPHERES;
  else {
    ROS_ERROR("rviz_cloud_annotation: invalid value for parameter %s: %s",
              PARAM_NAME_CONTROL_POINT_VISUAL, param_string.c_str());
    m_control_points_visual = CONTROL_POINT_VISUAL_SPHERE;
  }

  m_nh.param<double>(PARAM_NAME_CP_WEIGHT_SCALE_FRACTION, param_double,
                     PARAM_DEFAULT_CP_WEIGHT_SCALE_FRACTION);
  m_cp_weight_scale_fraction =
      std::min<float>(1.0, std::max(0.0, param_double));

  m_nh.param<bool>(PARAM_NAME_ZERO_WEIGHT_CP_SHOW,
                   m_show_zero_weight_control_points,
                   PARAM_DEFAULT_ZERO_WEIGHT_CP_SHOW);

  m_nh.param<std::string>(PARAM_NAME_RECT_SELECTION_TOPIC, param_string,
                          PARAM_DEFAULT_RECT_SELECTION_TOPIC);
  m_rect_selection_sub =
      m_nh.subscribe(param_string, 1,
                     &RVizCloudAnnotation::onRectangleSelectionViewport, this);

  m_nh.param<std::string>(PARAM_NAME_SAVE_TOPIC, param_string,
                          PARAM_DEFAULT_SAVE_TOPIC);
  m_save_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onSave, this);

  m_nh.param<std::string>(PARAM_NAME_RESTORE_TOPIC, param_string,
                          PARAM_DEFAULT_RESTORE_TOPIC);
  m_restore_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onRestore, this);

  m_nh.param<std::string>(PARAM_NAME_CLEAR_TOPIC, param_string,
                          PARAM_DEFAULT_CLEAR_TOPIC);
  m_clear_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onClear, this);

  m_nh.param<std::string>(PARAM_NAME_NEW_TOPIC, param_string,
                          PARAM_DEFAULT_NEW_TOPIC);
  m_new_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onNew, this);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC, param_string,
                          PARAM_DEFAULT_SET_EDIT_MODE_TOPIC);
  m_set_edit_mode_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onSetEditMode, this);

  m_nh.param<std::string>(PARAM_NAME_TOGGLE_NONE_TOPIC, param_string,
                          PARAM_DEFAULT_TOGGLE_NONE_TOPIC);
  m_toggle_none_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onToggleNoneMode, this);

  m_nh.param<std::string>(PARAM_NAME_SET_CURRENT_LABEL_TOPIC, param_string,
                          PARAM_DEFAULT_SET_CURRENT_LABEL_TOPIC);
  m_set_current_label_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onSetCurrentLabel, this);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2, param_string,
                          PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
  m_set_edit_mode_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

  m_nh.param<std::string>(PARAM_NAME_CURRENT_LABEL_TOPIC, param_string,
                          PARAM_DEFAULT_CURRENT_LABEL_TOPIC);
  m_set_current_label_pub = m_nh.advertise<std_msgs::UInt32>(param_string, 1);

  m_nh.param<std::string>(PARAM_NAME_POINT_COUNT_UPDATE_TOPIC, param_string,
                          PARAM_DEFAULT_POINT_COUNT_UPDATE_TOPIC);
  m_point_count_update_pub =
      m_nh.advertise<std_msgs::UInt64MultiArray>(param_string, 1, true);

  m_nh.param<std::string>(SAVE_BBOX_NAME, m_bbox_save, SAVE_BBOX_DEFAULT_NAME);

  m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC, param_string,
                          PARAM_DEFAULT_SET_NAME_TOPIC);
  m_set_name_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onSetName, this);

  bbox_marker_pub =
      m_nh.advertise<visualization_msgs::Marker>("bbox_marker", 1, true);

  bbox_head_marker_pub =
      m_nh.advertise<visualization_msgs::Marker>("bbox_head_marker", 1, true);

  lane_marker_pub =
      m_nh.advertise<visualization_msgs::Marker>("lane_marker", 1, true);

  kerb_marker_pub =
      m_nh.advertise<visualization_msgs::Marker>("kerb_marker", 1, true);

  plane_marker_pub =
      m_nh.advertise<visualization_msgs::Marker>("plane_marker", 1, true);

  m_nh.param<std::string>(PARAM_NAME_SET_NAME_TOPIC2, param_string,
                          PARAM_DEFAULT_SET_NAME_TOPIC2);
  m_set_name_pub = m_nh.advertise<std_msgs::String>(param_string, 1, true);

  m_nh.param<std::string>(PARAM_NAME_VIEW_CONTROL_POINTS_TOPIC, param_string,
                          PARAM_DEFAULT_VIEW_CONTROL_POINTS_TOPIC);
  m_view_control_points_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onViewControlPoints, this);

  m_nh.param<std::string>(PARAM_NAME_VIEW_CLOUD_TOPIC, param_string,
                          PARAM_DEFAULT_VIEW_CLOUD_TOPIC);
  m_view_cloud_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onViewCloud, this);

  m_nh.param<std::string>(PARAM_NAME_VIEW_LABEL_TOPIC, param_string,
                          PARAM_DEFAULT_VIEW_LABEL_TOPIC);
  m_view_labels_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onViewLabels, this);

  m_nh.param<std::string>(PARAM_NAME_UNDO_REDO_STATE_TOPIC, param_string,
                          PARAM_DEFAULT_UNDO_REDO_STATE_TOPIC);
  m_undo_redo_state_pub = m_nh.advertise<rviz_cloud_annotation::UndoRedoState>(
      param_string, 1, true);

  m_nh.param<std::string>(PARAM_NAME_UNDO_TOPIC, param_string,
                          PARAM_DEFAULT_UNDO_TOPIC);
  m_undo_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onUndo, this);

  m_nh.param<std::string>(PARAM_NAME_REDO_TOPIC, param_string,
                          PARAM_DEFAULT_REDO_TOPIC);
  m_redo_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onRedo, this);

  m_nh.param<std::string>(PARAM_NAME_NEXT_TOPIC, param_string,
                          PARAM_DEFAULT_NEXT_TOPIC);
  m_next_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onNextObject, this);

  m_nh.param<std::string>(PARAM_NAME_PRE_TOPIC, param_string,
                          PARAM_DEFAULT_PRE_TOPIC);
  m_pre_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onPreObject, this);

  m_nh.param<std::string>(PARAM_NAME_POINT_SIZE_CHANGE_TOPIC, param_string,
                          PARAM_DEFAULT_POINT_SIZE_CHANGE_TOPIC);
  m_point_size_change_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onPointSizeChange, this);

  m_nh.param<float>(PARAM_NAME_POINT_SIZE_CHANGE_MULT,
                    m_point_size_change_multiplier,
                    PARAM_DEFAULT_POINT_SIZE_CHANGE_MULT);

  m_nh.param<std::string>(PARAM_NAME_YAW_TOPIC, param_string,
                          PARAM_DEFAULT_YAW_TOPIC);
  m_control_yaw_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onControlYawChange, this);

  m_nh.param<std::string>(PARAM_NAME_OCCLUDED_TOPIC, param_string,
                          PARAM_DEFAULT_OCCLUDED_TOPIC);
  m_bbox_occluded_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onControlOccludedChange, this);

  m_nh.param<std::string>(PARAM_NAME_YAW_MAX_TOPIC, param_string,
                          PARAM_DEFAULT_YAW_MAX_TOPIC);
  m_control_yaw_max_pub = nh.advertise<std_msgs::Int32>(param_string, 1, true);

  m_nh.param<std::string>(PARAM_NAME_YAW_MIN_TOPIC, param_string,
                          PARAM_DEFAULT_YAW_MIN_TOPIC);
  m_control_yaw_min_pub = nh.advertise<std_msgs::Int32>(param_string, 1, true);

  m_nh.param<std::string>(PARAM_NAME_BIAS_TOPIC, param_string,
                          PARAM_DEFAULT_BIAS_TOPIC);
  m_control_bias_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onControlBiasChange, this);

  m_nh.param<std::string>(PARAM_NAME_BIAS_ZERO_TOPIC, param_string,
                          PARAM_DEFAULT_BIAS_ZERO_TOPIC);
  m_control_bias_zero_pub =
      nh.advertise<std_msgs::Empty>(param_string, 1, true);

  m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_UNUSED_TOPIC, param_string,
                          PARAM_DEFAULT_GOTO_FIRST_UNUSED_TOPIC);
  m_goto_first_unused_sub = nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onGotoFirstUnused, this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_LAST_UNUSED_TOPIC, param_string,
                          PARAM_DEFAULT_GOTO_LAST_UNUSED_TOPIC);
  m_goto_last_unused_sub = nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onGotoLastUnused, this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_FIRST_TOPIC, param_string,
                          PARAM_DEFAULT_GOTO_FIRST_TOPIC);
  m_goto_first_sub =
      nh.subscribe(param_string, 1, &RVizCloudAnnotation::onGotoFirst, this);

  m_nh.param<std::string>(PARAM_NAME_GOTO_NEXT_UNUSED_TOPIC, param_string,
                          PARAM_DEFAULT_GOTO_NEXT_UNUSED_TOPIC);
  m_goto_next_unused_sub = nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onGotoNextUnused, this);

  m_nh.param<std::string>(PARAM_NAME_ANNOTATION_TYPE_TOPIC, param_string,
                          PARAM_DEFAULT_ANNOTATION_TYPE_TOPIC);
  m_on_set_annotation_type_sub = m_nh.subscribe(
      param_string, 1, &RVizCloudAnnotation::onSetAnnotationType, this);

  m_nh.param<std::string>(PARAM_NAME_OBJECT_ID_TOPIC, param_string,
                          PARAM_DEFAULT_OBJECT_ID_TOPIC);
  m_object_id_pub = nh.advertise<std_msgs::Int32>(param_string, 1, true);

  m_nh.param<std::string>(PARAM_AUTO_PLANE_TOPIC, param_string,
                          PARAM_DEFAULT_AUTO_PLANE_TOPIC);
  m_on_auto_plane_sub =
      m_nh.subscribe(param_string, 1, &RVizCloudAnnotation::onAutoPlane, this);

  m_nh.param<double>(PARAM_NAME_AUTOSAVE_TIME, param_double,
                     PARAM_DEFAULT_AUTOSAVE_TIME);
  if (param_double >= 1.0) {
    m_autosave_timer =
        m_nh.createTimer(ros::Duration(param_double),
                         &RVizCloudAnnotation::onAutosave, this, false, false);
    ROS_INFO("rviz_cloud_annotation: autosave every %f seconds.",
             float(param_double));
  }
  m_nh.param<bool>(PARAM_NAME_AUTOSAVE_TIMESTAMP, m_autosave_append_timestamp,
                   PARAM_DEFAULT_AUTOSAVE_TIMESTAMP);

  m_current_label = 1;
  m_edit_mode = EDIT_MODE_NONE;
  m_prev_edit_mode = EDIT_MODE_NONE;

  m_point_size_multiplier = 1.0;

  m_control_point_weight_step = m_control_point_max_weight;

  m_control_yaw_step = m_control_yaw_max;

  m_view_cloud = m_view_labels = m_view_control_points = true;

  SendCloudMarker(true);
  SendControlPointMaxWeight();
  SendYawMax();
  SendYawMin();
  SendBiasZero();
  // Restore(m_annotation_filename_in); //恢复标记
  m_autosave_timer.start();
}

//加载一帧点云

void RVizCloudAnnotation::LoadCloud(const std::string &filename,
                                    const std::string &normal_source,
                                    PointXYZRGBNormalCloud &cloud) {
  std::string pcd_type;
  m_nh.param<std::string>(PARAM_NAME_PCD_TYPE, pcd_type,
                          PARAM_DEFAULT_PCD_TYPE);
  ROS_INFO("rviz_cloud_annotation: expected pcd format: %s", pcd_type.c_str());
  cloud.clear();

  pcl::PCLPointCloud2 cloud2;
  if (pcl::io::loadPCDFile(filename, cloud2)) // read pcd file
  {
    throw std::string(std::string("could not load cloud: ") + filename);
  }
  if (pcd_type == "XYZI") {
    PointXYZICloud cloud_in;
    pcl::fromPCLPointCloud2(cloud2, cloud_in); // input pointCloud as XYZI
    // transform XYZI to XYZRGB
    PointXYZRGBCloud xyz_rgb_cloud;
    for (int64 i = 0; i < cloud_in.size(); i++) {
      PointXYZRGB point;
      point.x = cloud_in[i].x;
      point.y = cloud_in[i].y;
      point.z = cloud_in[i].z;
      // float radius = m_sqrt(point.x * point.x + point.y * point.y);
      if (true) // radius < DISTANCE_LIMMIT && point.z < HEIGHT_LIMMIT)
      {
        colorize_point_cloud(cloud_in[i].intensity,
                             &point); // Transform intensity to RGB
        xyz_rgb_cloud.push_back(point);
      }
    }
    pcl::copyPointCloud(xyz_rgb_cloud,
                        cloud); // Transform to XYZRGBNormal finally
  } else if (pcd_type == "XYZRGB") {
    PointXYZRGBCloud cloud_in;
    pcl::fromPCLPointCloud2(cloud2, cloud_in); // input pointCloud as XYZRGB
    pcl::copyPointCloud(cloud_in, cloud); // Transform to XYZRGBNormal finally
  } else if (pcd_type == "XYZ") {
    PointXYZCloud cloud_in;
    pcl::fromPCLPointCloud2(cloud2, cloud_in); // input pointCloud as XYZ
    pcl::copyPointCloud(cloud_in, cloud); // Transform to XYZRGBNormal finally
  }

  ROS_INFO("rviz_cloud_annotation: cloud size: %ld", cloud.size());

  for (uint64 i = 0; i < cloud.size(); i++) {
    ids_in_plane_flag.push_back(0);
  }

  // code for plane annotation, should be removed if you don't need plane
  // annotation for decreasing running time.
  // generateDefaultPlane(cloud);
  //
}

//对点云进行 强度 --> RGB转换
void RVizCloudAnnotation::colorize_point_cloud(double intensity,
                                               PointXYZRGB *sample) {
  // if (intensity > 1) {
  //   intensity = intensity / 255;
  // }
  // intensity = intensity * 255;
  int r, g, b;
  double intensity_range = 255;
  double wavelength;
  double min_wavelength = 470;
  if (intensity <= intensity_range)
    wavelength =
        intensity / intensity_range * (780 - min_wavelength) + min_wavelength;
  else
    wavelength = 780;
  if ((wavelength >= 380) && (wavelength < 440)) {
    r = (-(wavelength - 440) / (440 - 380)) * 255;
    g = 0;
    b = 255;
  } else if ((wavelength >= 440) && (wavelength < 490)) {
    r = 0;
    g = ((wavelength - 440) / (490 - 440)) * 255;
    b = 255;
  } else if ((wavelength >= 490) && (wavelength < 510)) {
    r = 0;
    g = 255;
    b = (-(wavelength - 510) / (510 - 490)) * 255;
  } else if ((wavelength >= 510) && (wavelength < 580)) {
    r = ((wavelength - 510) / (580 - 510)) * 255;
    g = 255;
    b = 0;
  } else if ((wavelength >= 580) && (wavelength < 645)) {
    r = 255;
    g = (-(wavelength - 645) / (645 - 580)) * 255;
    b = 0;
  } else if ((wavelength >= 645) && (wavelength < 781)) {
    r = 255;
    g = 0;
    b = 0;
  } else {
    r = 0;
    g = 0;
    b = 0;
  }
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  sample->rgb = *reinterpret_cast<float *>(&rgb);
}

//取消上一次操作
void RVizCloudAnnotation::onUndo(const std_msgs::Empty &) {
  if (!m_undo_redo.IsUndoEnabled())
    return;

  ROS_INFO("rviz_cloud_annotation: Undo.");
  const Uint64Vector changed = m_undo_redo.Undo();
  SendControlPointsMarker(changed, true);
  SendPointCounts(changed);
  SendName();
  SendUndoRedoState();
}

//恢复上一次操作
void RVizCloudAnnotation::onRedo(const std_msgs::Empty &) {
  if (!m_undo_redo.IsRedoEnabled())
    return;

  ROS_INFO("rviz_cloud_annotation: Redo.");
  const Uint64Vector changed = m_undo_redo.Redo();
  SendControlPointsMarker(changed, true);
  SendPointCounts(changed);
  SendName();
  SendUndoRedoState();
}

//清除当前标记的物体 或 清除当前帧所有标记
void RVizCloudAnnotation::onClear(const std_msgs::UInt32 &label_msg) {
  const uint64 old_max_label = m_annotation->GetNextLabel();

  const uint64 clear_label = label_msg.data;
  if (clear_label >= old_max_label)
    return;

  if (clear_label != 0) {
    const Uint64Vector changed = m_undo_redo.ClearLabel(clear_label);
    SendControlPointsMarker(changed, true);
    SendPointCounts(changed);
    SendName();
    SendUndoRedoState();
    if (ANNOTATION_TYPE == BBOX) {
      EmptyBboxToMarker(BBOX_ID);
      ids_in_bbox[BBOX_ID].clear();
    } else if (ANNOTATION_TYPE == KERB) {
      for (int i = 0; i < KERB_SIZE[KERB_ID]; i++) {
        KERB_SET[KERB_ID][i][0] = 0;
        KERB_SET[KERB_ID][i][1] = 0;
        KERB_SET[KERB_ID][i][2] = 0;
        KERB_SET[KERB_ID][i][3] = 0;
      }
      ids_in_kerb[KERB_ID].clear();
      KERB_SIZE[KERB_ID] = 0;
      EmptyKerbToMarker(KERB_ID);
    } else if (ANNOTATION_TYPE == LANE) {
      for (int i = 0; i < LANE_SIZE[LANE_ID]; i++) {
        LANE_SET[LANE_ID][i][0] = 0;
        LANE_SET[LANE_ID][i][1] = 0;
        LANE_SET[LANE_ID][i][2] = 0;
        LANE_SET[LANE_ID][i][3] = 0;
      }
      ids_in_lane[LANE_ID].clear();
      LANE_SIZE[LANE_ID] = 0;
      EmptyLaneToMarker(LANE_ID);
    } else if (ANNOTATION_TYPE == PLANE) {
      ids_in_plane.clear();
      ids_in_plane_flag.clear();
      EmptyPlaneToMarker(PLANE_ID);
    }

    return;
  }

  const Uint64Vector changed = m_undo_redo.Clear();
  SendPointCounts(changed);
  SendControlPointsMarker(changed, true);
  SendName();
  SendUndoRedoState();

  for (int i = 0; i <= BBOX_ID; i++) {
    if (BBOX_SET[i][10] > 1) // >1 则不为空BBOX
    {
      EmptyBboxToMarker(i);
    }
    ids_in_bbox[i].clear();
  }
  for (int k = 0; k <= KERB_ID; k++) {
    for (int i = 0; i < KERB_SIZE[k]; i++) {
      KERB_SET[k][i][0] = 0;
      KERB_SET[k][i][1] = 0;
      KERB_SET[k][i][2] = 0;
      KERB_SET[k][i][3] = 0;
    }
    ids_in_kerb[k].clear();
    KERB_SIZE[k] = 0;
    EmptyKerbToMarker(k);
  }
  for (int k = 0; k <= LANE_ID; k++) {
    for (int i = 0; i < LANE_SIZE[k]; i++) {
      LANE_SET[k][i][0] = 0;
      LANE_SET[k][i][1] = 0;
      LANE_SET[k][i][2] = 0;
      LANE_SET[k][i][3] = 0;
    }
    ids_in_lane[k].clear();
    LANE_SIZE[k] = 0;
    EmptyLaneToMarker(k);
  }

  ids_in_plane.clear();
  ids_in_plane_flag.clear();
  EmptyPlaneToMarker(PLANE_ID);
}

//根据标志记录文件恢复当前帧的标注
void RVizCloudAnnotation::Restore(const std::string &filename) {
  std::ifstream ifile(filename.c_str());
  if (!ifile) {
    ROS_ERROR("rviz_cloud_annotation: could not open file: %s",
              filename.c_str());
    return;
  }

  ROS_INFO("rviz_cloud_annotation: loading file: %s", filename.c_str());

  RVizCloudAnnotationPoints::Ptr maybe_new_annotation;
  try {
    maybe_new_annotation = RVizCloudAnnotationPoints::Deserialize(
        ifile, m_control_point_max_weight, m_point_neighborhood);
  } catch (const RVizCloudAnnotationPoints::IOE &e) {
    ROS_ERROR("rviz_cloud_annotation: could not load file %s, reason: %s.",
              filename.c_str(), e.description.c_str());
    return;
  }

  if (maybe_new_annotation->GetCloudSize() != m_annotation->GetCloudSize()) {
    const uint new_size = maybe_new_annotation->GetCloudSize();
    const uint old_size = m_annotation->GetCloudSize();
    ROS_ERROR("rviz_cloud_annotation: file was created for a cloud of size %u, "
              "but it is %u. Load operation aborted.",
              new_size, old_size);
    return;
  }

  ClearControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), false);
  m_annotation = maybe_new_annotation;
  m_undo_redo.SetAnnotation(maybe_new_annotation);
  SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
  SendName();
  SendPointCounts(RangeUint64(1, m_annotation->GetNextLabel()));
  SendUndoRedoState();

  ROS_INFO("rviz_cloud_annotation: file loaded.");
}

//保存当前帧的标记信息的Label
void RVizCloudAnnotation::Save(const bool is_autosave) {
  if (is_autosave)
    ROS_INFO("rviz_cloud_annotation: auto-saving.");

  if (!m_annotation_file_files.empty()) {
    std::string filename = m_annotation_file_files[FILE_ID];

    if (is_autosave && m_autosave_append_timestamp) {
      filename = AppendTimestampBeforeExtension(filename);
    }

    std::ofstream ofile(filename.c_str(), std::ios::trunc);
    if (!ofile) {
      ROS_ERROR("rviz_cloud_annotation: could not create file: %s",
                filename.c_str());
      return;
    }

    ROS_INFO("rviz_cloud_annotation: saving file: %s", filename.c_str());

    try {
      m_annotation->Serialize(ofile); //输出序列化
    } catch (const RVizCloudAnnotationPoints::IOE &e) {
      ROS_ERROR("rviz_cloud_annotation: could not save file %s, reason: %s.",
                filename.c_str(), e.description.c_str());
    }
    ROS_INFO("rviz_cloud_annotation: done.");
  }

  if (!m_annotation_cloud_files.empty()) {
    //输出点云 .PCD格式
    std::string cloud_filename = m_annotation_cloud_files[FILE_ID];
    if (is_autosave && m_autosave_append_timestamp)
      cloud_filename = AppendTimestampBeforeExtension(cloud_filename);

    ROS_INFO("rviz_cloud_annotation: saving cloud: %s", cloud_filename.c_str());
    {
      PointXYZRGBLCloud cloud_out;
      pcl::copyPointCloud(*m_cloud, cloud_out);
      if (pcl::io::savePCDFileBinary(cloud_filename, cloud_out))
        ROS_ERROR("rviz_cloud_annotation: could not save labeled cloud.");
    }
    ROS_INFO("rviz_cloud_annotation: done.");

    FinalLabel(*m_cloud);

    ExportLane(*m_cloud);

    PointXYZRGBLCloud cloud_out1;
    pcl::copyPointCloud(*m_cloud, cloud_out1);
    std::string save_label_name = m_label_files[FILE_ID];

    std::ofstream ofile1;
    ofile1.open(save_label_name.c_str(), std::ios::trunc);

    ROS_INFO("rviz_cloud_annotation: filename: %s", save_label_name.c_str());

    for (int i = 0; i < cloud_out1.size(); i++) {
      std::string m_str_ = label2Name(m_label[i]);

      ofile1 << i << "\t" << cloud_out1[i].x << "\t" << cloud_out1[i].y << "\t"
             << cloud_out1[i].z << "\t" << m_str_ << "\n";
    }
    ofile1.close();
    m_label.clear();
  }

  if (!m_annotation_cloud_files.empty()) {
    std::string save_bbox_name = m_bbox_files[FILE_ID];

    std::ofstream ofile2;
    ofile2.open(save_bbox_name.c_str(), std::ios::trunc);

    ROS_INFO("rviz_cloud_annotation: bbox name: %s", save_bbox_name.c_str());
    for (int i = 0; i <= BBOX_ID; i++) {
      if (BBOX_SET[i][10] > 1) {
        ROS_INFO("rviz_cloud_annotation: bbox id: %d", i);
        ofile2 << label2Name((int)(BBOX_LABEL_SET[i][0])) << "\t";
        for (int j = 1; j < 10; j++) {
          ofile2 << BBOX_LABEL_SET[i][j] << "\t";
        }
        ofile2 << "\n";
      }
    }
    ofile2.close();
  }
  if (!m_dataset_files.empty()) {
    std::ofstream ofile3;
    ofile3.open(m_log_file.c_str(), std::ios::app);
    ofile3 << m_dataset_files[FILE_ID].c_str() << "\n";
  }
}

//点的 标签序号 --> Name
std::string RVizCloudAnnotation::label2Name(int label) {
  std::string m_str_ = "No";
  if (label > 1000) {
    return m_str_;
  }
  if (label == -1) {
    m_str_ = "Kerb";
    return m_str_;
  } else if (label == -2) {
    m_str_ = "Lane";
    return m_str_;
  } else if (label == -3) {
    m_str_ = "Ground";
    return m_str_;
  }

  int64 m_label_ = (label - 1) % 20;
  m_label_ = m_label_ / 5;

  switch (m_label_) {
  case 0:
    m_str_ = "Car";
    break;
  case 1:
    m_str_ = "Cart";
    break;
  case 2:
    m_str_ = "Pedestrian";
    break;
  case 3:
    m_str_ = "Cyclist";
    break;
  }
  return m_str_;
}

//设置操作模式
void RVizCloudAnnotation::SetEditMode(const uint64 new_edit_mode) {
  if (m_edit_mode == new_edit_mode)
    return;

  const char *info_string;

  bool send_cloud = false;
  switch (new_edit_mode) {
  case EDIT_MODE_NONE:
    if (m_edit_mode != EDIT_MODE_NONE)
      send_cloud = true;
    info_string = "Move";
    break;
  case EDIT_MODE_CONTROL_POINT:
    if (m_edit_mode == EDIT_MODE_NONE)
      send_cloud = true;
    info_string = "Point";
    break;
  case EDIT_MODE_ERASER:
    if (m_edit_mode == EDIT_MODE_NONE)
      send_cloud = true;
    info_string = "Delete";
    break;
  case EDIT_MODE_COLOR_PICKER:
    if (m_edit_mode == EDIT_MODE_NONE)
      send_cloud = true;
    info_string = "Pick Color";
    break;
  default:
    ROS_ERROR("rviz_cloud_annotation: unsupported edit mode %u received.",
              (unsigned int)(new_edit_mode));
    return;
  }

  ROS_INFO("rviz_cloud_annotation: edit mode is now: %s", info_string);

  if ((m_edit_mode == EDIT_MODE_NONE) || (new_edit_mode == EDIT_MODE_NONE))
    m_prev_edit_mode = m_edit_mode; // there must be at least one EDIT_MODE_NONE
                                    // between current and prev
  m_edit_mode = new_edit_mode;

  std_msgs::UInt32 msg;
  msg.data = m_edit_mode;
  m_set_edit_mode_pub.publish(msg);

  if (send_cloud) {
    SendCloudMarker(false);
    SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
  }
}

//清除点击点的Marker
void RVizCloudAnnotation::ClearControlPointsMarker(const Uint64Vector &indices,
                                                   const bool apply) {
  const uint64 changed_size = indices.size();
  const ControlPointDataVector control_points_empty;
  const Uint64Vector labels_empty;

  for (uint64 i = 0; i < changed_size; i++) {
    const uint64 label = indices[i];
    m_interactive_marker_server->insert(
        ControlPointsToMarker(*m_cloud, control_points_empty, label,
                              (m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));
    m_interactive_marker_server->insert(
        LabelsToMarker(*m_cloud, labels_empty, label,
                       (m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));
  }

  m_interactive_marker_server->insert(
      LabelsToMarker(*m_cloud, labels_empty, 0,
                     (m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

//被选择点增加Marker
RVizCloudAnnotation::InteractiveMarker
RVizCloudAnnotation::ControlPointsToMarker(
    const PointXYZRGBNormalCloud &cloud,
    const ControlPointDataVector &control_points, const uint64 label,
    const bool interactive) {
  const uint64 control_size = control_points.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = std::string(CONTROL_POINT_MARKER_PREFIX) +
                boost::lexical_cast<std::string>(label);
  marker.description = "";

  const pcl::RGB color = pcl::GlasbeyLUT::at((label - 1) % 256);

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  const bool use_spheres =
      m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE ||
      m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES;

  const float control_label_size =
      use_spheres ? m_control_label_size / 2.0 : m_control_label_size;
  const uint64 cp_marker_count =
      m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE
          ? 1
          : m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES
                ? 3
                : m_control_points_visual == CONTROL_POINT_VISUAL_LINE ? 2 : 2;

  Marker cloud_marker;
  cloud_marker.type =
      use_spheres ? int(Marker::SPHERE_LIST) : int(Marker::LINE_LIST);
  cloud_marker.scale.x = control_label_size;
  cloud_marker.scale.y = use_spheres ? control_label_size : 0.0;
  cloud_marker.scale.z = use_spheres ? control_label_size : 0.0;
  cloud_marker.color.r = float(color.r) / 255.0;
  cloud_marker.color.g = float(color.g) / 255.0;
  cloud_marker.color.b = float(color.b) / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(control_size * cp_marker_count);
  uint64 out_marker_count = 0;
  for (uint64 i = 0; i < control_size; i++) {
    const PointXYZRGBNormal &pt = cloud[control_points[i].point_id];

    const uint32 weight_step_id = control_points[i].weight_step_id;
    if (weight_step_id == 0 && !m_show_zero_weight_control_points)
      continue;

    const float weight_rel =
        float(weight_step_id) / float(m_control_point_max_weight);
    const float weight_scale = (1.0 - m_cp_weight_scale_fraction) +
                               weight_rel * m_cp_weight_scale_fraction;

    cloud_marker.points[out_marker_count].x = pt.x;
    cloud_marker.points[out_marker_count].y = pt.y;
    cloud_marker.points[out_marker_count].z = pt.z;
    out_marker_count++;

    if (cp_marker_count > 1) {
      cloud_marker.points[out_marker_count].x =
          pt.x + pt.normal_x * control_label_size * weight_scale;
      cloud_marker.points[out_marker_count].y =
          pt.y + pt.normal_y * control_label_size * weight_scale;
      cloud_marker.points[out_marker_count].z =
          pt.z + pt.normal_z * control_label_size * weight_scale;
      out_marker_count++;
    }

    if (cp_marker_count > 2) {
      cloud_marker.points[out_marker_count].x =
          pt.x + pt.normal_x * control_label_size * weight_scale / 2.0;
      cloud_marker.points[out_marker_count].y =
          pt.y + pt.normal_y * control_label_size * weight_scale / 2.0;
      cloud_marker.points[out_marker_count].z =
          pt.z + pt.normal_z * control_label_size * weight_scale / 2.0;
      out_marker_count++;
    }
  }
  cloud_marker.points.resize(out_marker_count);

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode =
      interactive ? int32(visualization_msgs::InteractiveMarkerControl::BUTTON)
                  : int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_control_points)
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

//发布点击点的Marker
void RVizCloudAnnotation::SendControlPointsMarker(
    const Uint64Vector &changed_labels, const bool apply) {
  const uint64 changed_size = changed_labels.size();
  for (uint64 i = 0; i < changed_size; i++) {
    const uint64 label = changed_labels[i];
    const bool isabove = label >= m_annotation->GetNextLabel();

    const ControlPointDataVector control_points =
        isabove ? ControlPointDataVector()
                : m_annotation->GetControlPointList(label);
    m_interactive_marker_server->insert(
        ControlPointsToMarker(*m_cloud, control_points, label,
                              (m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));

    const Uint64Vector label_points =
        isabove ? Uint64Vector() : m_annotation->GetLabelPointList(label);
    m_interactive_marker_server->insert(
        LabelsToMarker(*m_cloud, label_points, label,
                       (m_edit_mode != EDIT_MODE_NONE)),
        boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));
  }

  const Uint64Vector label_points = m_annotation->GetLabelPointList(0);
  m_interactive_marker_server->insert(
      LabelsToMarker(*m_cloud, label_points, 0,
                     (m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

//监听器： 点击点云
void RVizCloudAnnotation::onClickOnCloud(
    const InteractiveMarkerFeedbackConstPtr &feedback_ptr) //点选并处理
{
  if (m_edit_mode == EDIT_MODE_NONE) {
    ROS_WARN(
        "rviz_cloud_annotation: received stray click while not in edit mode.");
    return;
  }

  const InteractiveMarkerFeedback &feedback = *feedback_ptr;
  uint8 type = feedback.event_type;

  if (type != InteractiveMarkerFeedback::BUTTON_CLICK)
    return; // not a click

  ROS_INFO("rviz_cloud_annotation: click event (marker: %s).",
           feedback_ptr->marker_name.c_str());

  if (!feedback.mouse_point_valid)
    return; // invalid point

  bool ok;
  const uint64 idx = GetClickedPointId(feedback, ok);

  if (!ok)
    return;

  if (m_edit_mode == EDIT_MODE_CONTROL_POINT) {
    ROS_INFO("rviz_cloud_annotation: setting label %u to point %u",
             (unsigned int)(m_current_label), (unsigned int)(idx));
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(
        idx, m_control_point_weight_step, m_current_label);
    SendControlPointsMarker(changed_labels, true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();

    if (ANNOTATION_TYPE == BBOX) {
      ids_in_bbox[BBOX_ID].push_back(idx);
      ROS_INFO("ids_in_bbox size %i", int(ids_in_bbox[BBOX_ID].size()));
      generateBbox(*m_cloud, if_tilt);
    } else if (ANNOTATION_TYPE == KERB) {
      ids_in_kerb[KERB_ID].push_back(idx);
      generateKerb(*m_cloud);
    } else if (ANNOTATION_TYPE == LANE) {
      ids_in_lane[LANE_ID].push_back(idx);
      generateLane(*m_cloud);
    } else if (ANNOTATION_TYPE == PLANE) {
      ids_in_plane_flag[idx] = -3;
    }
  } else if (m_edit_mode == EDIT_MODE_ERASER) {
    ROS_INFO("rviz_cloud_annotation: eraser: erasing label from point %u",
             (unsigned int)(idx));
    const Uint64Vector changed_labels = m_undo_redo.SetControlPoint(idx, 0, 0);
    SendControlPointsMarker(changed_labels, true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();

    if (ANNOTATION_TYPE == BBOX) {
      if (ids_in_bbox[BBOX_ID].size() > 0) {
        ids_in_bbox[BBOX_ID].pop_back();
        generateBbox(*m_cloud, if_tilt);
        ROS_INFO("ids_in_bbox size %i", int(ids_in_bbox[BBOX_ID].size()));
      }
    } else if (ANNOTATION_TYPE == KERB) {
      if (ids_in_kerb[KERB_ID].size() > 0) {
        ids_in_kerb[KERB_ID].pop_back();
        generateKerb(*m_cloud);
      }
    } else if (ANNOTATION_TYPE == LANE) {
      if (ids_in_lane[LANE_ID].size() > 0) {
        ids_in_lane[LANE_ID].pop_back();
        generateLane(*m_cloud);
      }
    } else if (ANNOTATION_TYPE == PLANE) {
      ids_in_plane_flag[idx] = 0;
    }
  } else if (m_edit_mode == EDIT_MODE_COLOR_PICKER) {
    const uint64 label = m_annotation->GetLabelForPoint(idx);
    if (label == 0)
      ROS_WARN(
          "rviz_cloud_annotation: color picker: point %u has no label yet.",
          uint(idx));
    else {
      SetCurrentLabel(label);
      SetEditMode(EDIT_MODE_CONTROL_POINT);
    }
  }
}

//被选择点集增加Marker
RVizCloudAnnotation::InteractiveMarker RVizCloudAnnotation::LabelsToMarker(
    const PointXYZRGBNormalCloud &cloud, const Uint64Vector &labels,
    const uint64 label, const bool interactive) {
  const uint64 labels_size = labels.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = std::string(LABEL_POINT_MARKER_PREFIX) +
                boost::lexical_cast<std::string>(label);
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  pcl::RGB color;
  if (label == 0)
    color.r = color.g = color.b = 0;
  else
    color = pcl::GlasbeyLUT::at((label - 1) % 256);

  const float label_size =
      m_point_size_multiplier * (m_view_cloud ? m_label_size : m_point_size);
  const float normal_mult = m_view_cloud ? (m_label_size / 2.0) : 0.0;

  const bool use_spheres =
      m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE ||
      m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES;

  Marker cloud_marker;
  cloud_marker.type =
      use_spheres ? int(Marker::SPHERE_LIST) : int(Marker::LINE_LIST);
  cloud_marker.scale.x = label_size;
  cloud_marker.scale.y = label_size;
  cloud_marker.scale.z = label_size;
  cloud_marker.color.r = color.r / 255.0;
  cloud_marker.color.g = color.g / 255.0;
  cloud_marker.color.b = color.b / 255.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(labels_size);
  for (uint64 i = 0; i < labels_size; i++) {
    const PointXYZRGBNormal &pt = cloud[labels[i]];

    cloud_marker.points[i].x = pt.x + pt.normal_x * normal_mult;
    cloud_marker.points[i].y = pt.y + pt.normal_y * normal_mult;
    cloud_marker.points[i].z = pt.z + pt.normal_z * normal_mult;
  }

  if (m_view_cloud && m_show_points_back_labels) {
    cloud_marker.points.resize(labels_size * 2);

    for (uint64 i = 0; i < labels_size; i++) {
      const PointXYZRGBNormal &pt = cloud[labels[i]];

      cloud_marker.points[labels_size + i].x =
          pt.x - pt.normal_x * normal_mult; // point on the back
      cloud_marker.points[labels_size + i].y = pt.y - pt.normal_y * normal_mult;
      cloud_marker.points[labels_size + i].z = pt.z - pt.normal_z * normal_mult;
    }
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode =
      interactive ? int32(visualization_msgs::InteractiveMarkerControl::BUTTON)
                  : int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_labels && (!m_view_cloud || label != 0))
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

//点云增加Marker
RVizCloudAnnotation::InteractiveMarker
RVizCloudAnnotation::CloudToMarker(const PointXYZRGBNormalCloud &cloud,
                                   const bool interactive) {
  const uint64 cloud_size = cloud.size();

  InteractiveMarker marker;
  marker.header.frame_id = m_frame_id;
  marker.name = CLOUD_MARKER_NAME;
  marker.description = "";

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  const bool use_spheres =
      m_control_points_visual == CONTROL_POINT_VISUAL_SPHERE ||
      m_control_points_visual == CONTROL_POINT_VISUAL_THREE_SPHERES;
  Marker cloud_marker;
  cloud_marker.type =
      use_spheres ? int(Marker::SPHERE_LIST) : int(Marker::LINE_LIST);
  cloud_marker.scale.x = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.y = m_point_size_multiplier * m_point_size;
  cloud_marker.scale.z = m_point_size_multiplier * m_point_size;
  cloud_marker.color.r = 1.0;
  cloud_marker.color.g = 1.0;
  cloud_marker.color.b = 1.0;
  cloud_marker.color.a = 1.0;

  cloud_marker.points.resize(cloud_size);
  cloud_marker.colors.resize(cloud_size);
  for (uint64 i = 0; i < cloud_size; i++) {
    const PointXYZRGBNormal &pt = cloud[i];

    cloud_marker.points[i].x = pt.x;
    cloud_marker.points[i].y = pt.y;
    cloud_marker.points[i].z = pt.z;

    cloud_marker.colors[i].r = pt.r / 255.0;
    cloud_marker.colors[i].g = pt.g / 255.0;
    cloud_marker.colors[i].b = pt.b / 255.0;
    cloud_marker.colors[i].a = 1.0;
  }

  visualization_msgs::InteractiveMarkerControl points_control;
  points_control.always_visible = true;
  points_control.interaction_mode =
      interactive ? int32(visualization_msgs::InteractiveMarkerControl::BUTTON)
                  : int32(visualization_msgs::InteractiveMarkerControl::NONE);
  if (m_view_cloud)
    points_control.markers.push_back(cloud_marker);
  marker.controls.push_back(points_control);

  return marker;
}

//发布点云Marker
void RVizCloudAnnotation::SendCloudMarker(const bool apply) {
  m_interactive_marker_server->insert(
      CloudToMarker(*m_cloud, (m_edit_mode != EDIT_MODE_NONE)),
      boost::bind(&RVizCloudAnnotation::onClickOnCloud, this, _1));

  if (apply)
    m_interactive_marker_server->applyChanges();
}

void RVizCloudAnnotation::SetCurrentLabel(const uint64 label) {
  if (m_current_label == label)
    return;

  m_current_label = label;
  ROS_INFO("rviz_cloud_annotation: label is now: %u",
           (unsigned int)(m_current_label));
  SendName();
  SendUndoRedoState();

  std_msgs::UInt32 msg;
  msg.data = label;
  m_set_current_label_pub.publish(msg);

  gotoNextObject();
}

//获取点击事件类型
std::string RVizCloudAnnotation::GetClickType(const std::string &marker_name,
                                              uint64 &label_out) const {
  label_out = 0;
  if (marker_name == CLOUD_MARKER_NAME)
    return CLOUD_MARKER_NAME;

  std::string result;
  std::string num_str;
  if (marker_name.substr(0, std::strlen(CONTROL_POINT_MARKER_PREFIX)) ==
      CONTROL_POINT_MARKER_PREFIX) {
    result = CONTROL_POINT_MARKER_PREFIX;
    num_str = marker_name.substr(std::strlen(CONTROL_POINT_MARKER_PREFIX));
  } else if (marker_name.substr(0, std::strlen(LABEL_POINT_MARKER_PREFIX)) ==
             LABEL_POINT_MARKER_PREFIX) {
    result = LABEL_POINT_MARKER_PREFIX;
    num_str = marker_name.substr(std::strlen(LABEL_POINT_MARKER_PREFIX));
  } else {
    ROS_ERROR(
        "rviz_cloud_annotation: click: unsupported marker name in message: %s",
        marker_name.c_str());
    return "";
  }

  try {
    label_out = boost::lexical_cast<uint64>(num_str);
  } catch (const boost::bad_lexical_cast &) {
    ROS_ERROR("rviz_cloud_annotation: click: could not convert %s to number.",
              num_str.c_str());
    return "";
  }
  return result;
}

//获取点击点在点云中的ID
RVizCloudAnnotation::uint64 RVizCloudAnnotation::GetClickedPointId(
    const InteractiveMarkerFeedback &click_feedback, bool &ok) {
  if (true) {
    ok = false;
    const std::string marker_name = click_feedback.marker_name;

    uint64 label;
    std::string click_type = GetClickType(marker_name, label);

    if (click_type == CLOUD_MARKER_NAME ||
        click_type == LABEL_POINT_MARKER_PREFIX) {
      ROS_INFO("rviz_cloud_annotation: clicked on cloud point.");

      PointXYZRGBNormal click_pt;
      click_pt.x = click_feedback.mouse_point.x;
      click_pt.y = click_feedback.mouse_point.y;
      click_pt.z = click_feedback.mouse_point.z;

      ROS_INFO("rviz_cloud_annotation: click at: %f %f %f", float(click_pt.x),
               float(click_pt.y), float(click_pt.z));

      std::vector<int> idxs(1);
      std::vector<float> dsts(1);

      if (m_kdtree->nearestKSearch(click_pt, 1, idxs, dsts) <= 0) {
        ROS_WARN("rviz_cloud_annotation: point was clicked, but no nearest "
                 "cloud point found.");
        return 0;
      }
      const uint64 idx = idxs[0];
      const float dst = std::sqrt(dsts[0]);

      ROS_INFO("rviz_cloud_annotation: clicked on point: %u (accuracy: %f)",
               (unsigned int)(idx), float(dst));

      ok = true;
      return idx;
    }

    if (click_type == CONTROL_POINT_MARKER_PREFIX) {
      if (label == 0 || label >= m_annotation->GetNextLabel()) {
        ROS_ERROR(
            "rviz_cloud_annotation: click: invalid control point label %u",
            (unsigned int)(label));
        return 0;
      }

      const ControlPointDataVector control_points =
          m_annotation->GetControlPointList(label);
      if (control_points.empty()) {
        ROS_ERROR(
            "rviz_cloud_annotation: click: control point label %u is empty!",
            (unsigned int)(label));
        return 0;
      }

      const Eigen::Vector3f click_pt(click_feedback.mouse_point.x,
                                     click_feedback.mouse_point.y,
                                     click_feedback.mouse_point.z);

      uint64 nearest_idx;
      float nearest_sqr_dist;
      for (uint64 i = 0; i < control_points.size(); i++) {
        const PointXYZRGBNormal &pt = (*m_cloud)[control_points[i].point_id];
        const Eigen::Vector3f ept(pt.x, pt.y, pt.z);
        const Eigen::Vector3f en(pt.normal_x, pt.normal_y, pt.normal_z);
        const Eigen::Vector3f shift_pt = ept + en * m_control_label_size / 2.0;
        if (i == 0 || (shift_pt - click_pt).squaredNorm() < nearest_sqr_dist) {
          nearest_idx = i;
          nearest_sqr_dist = (shift_pt - click_pt).squaredNorm();
        }
      }

      const uint64 idx = control_points[nearest_idx].point_id;

      ROS_INFO(
          "rviz_cloud_annotation: clicked on control point: %lu (accuracy: %f)",
          idx, float(std::sqrt(nearest_sqr_dist)));

      ok = true;
      return idx;
    }
  }
  return 0;
}

//发布 取消/恢复 消息
void RVizCloudAnnotation::SendUndoRedoState() {
  rviz_cloud_annotation::UndoRedoState msg;
  msg.redo_enabled = m_undo_redo.IsRedoEnabled();
  msg.undo_enabled = m_undo_redo.IsUndoEnabled();
  msg.redo_description = m_undo_redo.GetRedoDescription();
  msg.undo_description = m_undo_redo.GetUndoDescription();
  m_undo_redo_state_pub.publish(msg);
}

//发布当前标记Name
void RVizCloudAnnotation::SendName() {
  std::string name = m_annotation->GetNameForLabel(m_current_label);
  std_msgs::String msg;
  msg.data = name;
  m_set_name_pub.publish(msg);
}

//发布当前标记的点的数量
void RVizCloudAnnotation::SendPointCounts(const Uint64Vector &labels) {
  const uint64 labels_size = labels.size();
  std_msgs::UInt64MultiArray msg;
  for (uint64 i = 0; i < labels_size; i++) {
    const uint64 label = labels[i];
    const uint64 count = m_annotation->GetLabelPointCount(label);
    msg.data.push_back(label);
    msg.data.push_back(count);
  }
  m_point_count_update_pub.publish(msg);
}

//发布BBOX默认最大遮挡系数
void RVizCloudAnnotation::SendControlPointMaxWeight() {
  std_msgs::Int32 msg;
  msg.data = m_control_point_max_weight;
  m_control_point_weight_max_weight_pub.publish(msg);
}
//发布BBOX默认最大方位角
void RVizCloudAnnotation::SendYawMax() {
  std_msgs::Int32 msg;
  msg.data = m_control_yaw_max;
  m_control_yaw_max_pub.publish(msg);
}
//发布BBOX默认最小方位角
void RVizCloudAnnotation::SendYawMin() {
  std_msgs::Int32 msg;
  msg.data = m_control_yaw_min;
  m_control_yaw_min_pub.publish(msg);
}
//发布BBOX默认Size偏移为0
void RVizCloudAnnotation::SendBiasZero() {
  std_msgs::Empty msg;
  m_control_bias_zero_pub.publish(msg);
}

// BBOX遮挡系数监听器
void RVizCloudAnnotation::onControlPointWeightChange(
    const std_msgs::Int32 &msg) {
  const uint32 new_weight = msg.data;
  if (new_weight > m_control_point_max_weight) {
    // ROS_ERROR("rviz_cloud_annotation: could not set weight to %i, maximum is
    // %i", (unsigned int)(new_weight),
    //(unsigned int)(m_control_point_weight_step));
    return;
  }

  m_control_point_weight_step = new_weight;
  // ROS_INFO("rviz_cloud_annotation: control point weight is now: %i",
  // (unsigned int)(m_control_point_weight_step));
}

// BBOX Size偏移监听器
void RVizCloudAnnotation::onControlBiasChange(
    const std_msgs::Float32MultiArray &msg) {
  FloatVector bias = msg.data;
  // ROS_INFO("rviz_cloud_annotation: bias: %f %f %f %f %f %f ", bias[0],
  // bias[1], bias[2], bias[3], bias[4], bias[5]);
  // ROS_INFO("rviz_cloud_annotation: ANNOTATION_TYPE %u", ANNOTATION_TYPE);
  if (ANNOTATION_TYPE == BBOX) {
    m_box_bias[BBOX_ID][0] = bias[0];
    m_box_bias[BBOX_ID][1] = bias[1];
    m_box_bias[BBOX_ID][2] = bias[2];
    m_box_bias[BBOX_ID][3] = bias[3];
    m_box_bias[BBOX_ID][4] = bias[4];
    m_box_bias[BBOX_ID][5] = bias[5];
    generateBbox(*m_cloud, if_tilt);
  }
}

// BBOX遮挡系数监听器
void RVizCloudAnnotation::onControlOccludedChange(const std_msgs::Int32 &msg) {
  const int new_occluded = msg.data;
  m_bbox_occluded[BBOX_ID] = (float)(new_occluded) / 100.0;
  ROS_INFO("rviz_cloud_annotation: current occluded: %f",
           m_bbox_occluded[BBOX_ID]);
}

// BBOX方位角监听器
void RVizCloudAnnotation::onControlYawChange(const std_msgs::Int32 &msg) {
  const int32 new_yaw = msg.data;
  if (new_yaw > m_control_yaw_max) {
    ROS_ERROR("rviz_cloud_annotation: could not set yaw to %i, maximum is %i",
              (signed int)(new_yaw), (signed int)(m_control_yaw_step));
    return;
  }

  m_control_yaw_step = new_yaw;
  BBOX_YAW_ANGLE = new_yaw;
  BBOX_YAW = BBOX_YAW_ANGLE / 180.0 * _M_PI;
  if (abs(BBOX_YAW_ANGLE) == 90 || abs(BBOX_YAW_ANGLE) == 0 ||
      abs(BBOX_YAW_ANGLE) == 180) {
    if_tilt = false;
  } else {
    if_tilt = true;
  }
  if (ANNOTATION_TYPE == BBOX) {
    generateBbox(*m_cloud, if_tilt);
  }
}

//点大小监听器
void RVizCloudAnnotation::onPointSizeChange(const std_msgs::Int32 &msg) {
  switch (msg.data) {
  case POINT_SIZE_CHANGE_BIGGER:
    if (m_point_size_multiplier > 1e5)
      return;
    m_point_size_multiplier *= (1.0 + m_point_size_change_multiplier);
    break;
  case POINT_SIZE_CHANGE_SMALLER:
    if (m_point_size_multiplier < 1e-5)
      return;
    m_point_size_multiplier /= (1.0 + m_point_size_change_multiplier);
    break;
  case POINT_SIZE_CHANGE_RESET:
    if (m_point_size_multiplier == 1.0)
      return;
    m_point_size_multiplier = 1.0;
    break;
  default:
    return;
  }

  ROS_INFO("rviz_cloud_annotation: point size multiplier is now: %f",
           float(m_point_size_multiplier));
  SendCloudMarker(false);
  SendControlPointsMarker(RangeUint64(1, m_annotation->GetNextLabel()), true);
}

//监听器：跳转至 第一个没使用的颜色
void RVizCloudAnnotation::onGotoFirstUnused(const std_msgs::Empty &) {
  ROS_INFO("rviz_cloud_annotation: go to first unused label.");
  uint64 i;
  for (i = 1; (m_annotation->GetLabelPointCount(i) != 0) && (i < UINT64_MAX);
       i++) {
  }
  SetCurrentLabel(i);
}

//监听器：跳转至 最后一个没使用的颜色
void RVizCloudAnnotation::onGotoLastUnused(const std_msgs::Empty &) {
  ROS_INFO("rviz_cloud_annotation: go to last unused label.");
  uint64 i = m_annotation->GetMaxLabel() + 1;
  for (; (m_annotation->GetLabelPointCount(i) == 0) && (i > 0); i--) {
  }
  SetCurrentLabel(i + 1);
}

//监听器：跳转至 第一个颜色
void RVizCloudAnnotation::onGotoFirst(const std_msgs::Empty &) {
  ROS_INFO("rviz_cloud_annotation: go to first label.");
  SetCurrentLabel(1);
}
//监听器：跳转至 下一个颜色
void RVizCloudAnnotation::onGotoNextUnused(const std_msgs::Empty &) {
  ROS_INFO("rviz_cloud_annotation: go to next unused label.");
  uint64 i = m_current_label + 1;
  for (; (m_annotation->GetLabelPointCount(i) != 0) && (i < UINT64_MAX); i++) {
  }
  SetCurrentLabel(i);
}

//监听器：框选 / 多边形选
void RVizCloudAnnotation::onRectangleSelectionViewport(
    const rviz_cloud_annotation::RectangleSelectionViewport &msg) {
  ROS_INFO("rviz_cloud_annotation: rectangle selection event received.");
  const bool is_deep_selection = msg.is_deep_selection;

  Eigen::Matrix4f projection_matrix;
  for (uint32 y = 0; y < 4; y++)
    for (uint32 x = 0; x < 4; x++)
      projection_matrix(y, x) = msg.projection_matrix[x + y * 4];

  const uint32 start_x = msg.start_x;
  const uint32 start_y = msg.start_y;
  const uint32 end_x = msg.end_x;
  const uint32 end_y = msg.end_y;
  const uint32 viewport_height = msg.viewport_height;
  const uint32 viewport_width = msg.viewport_width;
  const uint32 width = end_x - start_x;
  const uint32 height = end_y - start_y;

  const float focal_length = msg.focal_length;
  const float point_size = m_point_size_multiplier * m_point_size;

  Eigen::Affine3f camera_pose;
  {
    Eigen::Affine3d camera_pose_d;
    tf::poseMsgToEigen(msg.camera_pose, camera_pose_d);
    camera_pose = camera_pose_d.cast<float>();
  }
  const Eigen::Affine3f camera_pose_inv = camera_pose.inverse();

  Eigen::Affine3f scale_matrix = Eigen::Affine3f::Identity();
  scale_matrix(0, 0) = viewport_width / 2.0;
  scale_matrix(1, 1) = viewport_height / -2.0; // y axis must be inverted
  scale_matrix(1, 3) =
      viewport_height; // y is now negative, so move back to positive

  Eigen::Affine3f translation_matrix = Eigen::Affine3f::Identity();
  translation_matrix.translation().x() = 1.0;
  translation_matrix.translation().y() = 1.0;

  const Eigen::Matrix4f prod_matrix =
      (scale_matrix * translation_matrix * projection_matrix * camera_pose_inv)
          .matrix();

  Int32PolyTriangleVector tri_cond;
  if (!msg.polyline_x.empty()) {
    if (msg.polyline_x.size() < 3 ||
        msg.polyline_x.size() != msg.polyline_y.size()) {
      ROS_WARN("rviz_cloud_annotation: rectangle selection received invalid "
               "polyline: ignored.");
      return;
    }

    for (uint64 i = 2; i < msg.polyline_x.size(); i++) {
      Int32PolyTriangle tri(msg.polyline_x[0], msg.polyline_y[0],
                            msg.polyline_x[i - 1], msg.polyline_y[i - 1],
                            msg.polyline_x[i], msg.polyline_y[i]);
      tri_cond.push_back(tri);
    }
  }

  const Uint64Vector ids = RectangleSelectionToIds(
      prod_matrix, camera_pose_inv, *m_cloud, start_x, start_y, width, height,
      tri_cond, point_size, focal_length, is_deep_selection);
  ROS_INFO("rviz_cloud_annotation: rectangle selection selected %i points.",
           int(ids.size()));
  VectorSelection(ids);
}

//寻找多边形内部的点
RVizCloudAnnotation::int32
RVizCloudAnnotation::Int32PolyTriangle::Contains(const int32 px,
                                                 const int32 py) const {
  bool clockwise = false;

  for (uint64 i = 0; i < 3; i++) {
    const uint64 i1 = (i + 1) % 3;
    const uint64 i2 = (i + 2) % 3;
    const Eigen::Vector2i normal = Eigen::Vector2i(-y[i1] + y[i], x[i1] - x[i]);
    if (normal == Eigen::Vector2i::Zero())
      return 1; // degenerate

    const int32 dot1 = Eigen::Vector2i(x[i2] - x[i], y[i2] - y[i]).dot(normal);
    if (i == 0)
      clockwise = (dot1 < 0);

    const int32 dotp1 = Eigen::Vector2i(px - x[i], py - y[i]).dot(normal);
    if (!dot1)
      return 1; // degenerate
    if (!dotp1) {
      if (i != 1 && (clockwise == (i == 0)))
        return 1; // point on first edge is outside
    }

    if ((dotp1 > 0) != (dot1 > 0))
      return 1; // outside the triangle
  }

  return -1;
}

//统计 多边形/框选 选中的点
RVizCloudAnnotation::Uint64Vector RVizCloudAnnotation::RectangleSelectionToIds(
    const Eigen::Matrix4f prod_matrix, const Eigen::Affine3f camera_pose_inv,
    const PointXYZRGBNormalCloud &cloud, const uint32 start_x,
    const uint32 start_y, const uint32 width, const uint32 height,
    const Int32PolyTriangleVector tri_cond, const float point_size,
    const float focal_length, const bool is_deep_selection) {
  Uint64Vector virtual_id_image(width * height, 0);
  FloatVector virtual_depth_image(width * height, 0.0);

  BoolVector virtual_image_mask(width * height,
                                tri_cond.empty()); // if empty, all is true
  if (!tri_cond.empty()) {
    for (uint32 y = 0; y < height; y++)
      for (uint32 x = 0; x < width; x++) {
        uint32 found = 0;
        for (uint64 i = 0; i < tri_cond.size(); i++) {
          const Int32PolyTriangle &tri = tri_cond[i];
          const int32 contains = tri.Contains(x + start_x, y + start_y);
          if (contains <= 0)
            found += 1; // inside
        }

        virtual_image_mask[y * width + x] = (found % 2);
      }
  }

  const uint64 cloud_size = cloud.size();
  BoolVector selected_points(cloud_size, false);
  for (uint64 i = 0; i < cloud_size; i++) {
    const PointXYZRGBNormal &ppt = cloud[i];
    const Eigen::Vector3f ept(ppt.x, ppt.y, ppt.z);
    const Eigen::Vector4f thpt = prod_matrix * ept.homogeneous();
    if (thpt.w() < 1e-5)
      continue;
    const Eigen::Vector3f tpt = thpt.head<3>() / thpt.w();
    const Eigen::Vector2i itpt =
        tpt.head<2>().cast<int>() - Eigen::Vector2i(start_x, start_y);
    const float depth = -(camera_pose_inv * ept).z();
    if (depth < 0.0)
      continue; // behind the observer

    // if (is_deep_selection)
    if (true) {
      if (itpt.x() < 0 || itpt.y() < 0 || itpt.x() >= int(width) ||
          itpt.y() >= int(height))
        continue;
      if (!virtual_image_mask[itpt.x() + itpt.y() * width])
        continue;
      selected_points[i] = true; //确定选择的点
      continue;
    }

    // if not deep, then we must compute point size for occlusions
    const float size_px = (point_size / 4.0) * focal_length / depth;
    const int32 window_px = std::max<int32>(0, size_px);
    if (itpt.x() < -window_px || itpt.y() < -window_px ||
        itpt.x() >= int(width) + window_px ||
        itpt.y() >= int(height) + window_px)
      continue;

    if (window_px == 0) {
      if (itpt.x() < 0 || itpt.y() < 0 || itpt.x() >= int(width) ||
          itpt.y() >= int(height))
        continue;
      const uint64 di = itpt.x() + itpt.y() * width;
      if (!virtual_image_mask[di])
        continue;

      if (virtual_id_image[di] == 0 || virtual_depth_image[di] > depth) {
        virtual_id_image[di] = i + 1;
        virtual_depth_image[di] = depth;
      }
      continue;
    }

    for (int32 dy = -window_px; dy <= window_px; dy++)
      for (int32 dx = -window_px; dx <= window_px; dx++) {
        const Eigen::Vector2i ditpt = itpt + Eigen::Vector2i(dx, dy);
        if (ditpt.x() < 0 || ditpt.y() < 0 || ditpt.x() >= int(width) ||
            ditpt.y() >= int(height))
          continue;
        const uint64 di = ditpt.x() + ditpt.y() * width;
        if (!virtual_image_mask[di])
          continue;

        if (virtual_id_image[di] == 0 || virtual_depth_image[di] > depth) {
          virtual_id_image[di] = i + 1;
          virtual_depth_image[di] = depth;
        }
      }
  }

  if (!is_deep_selection) {
    for (uint64 i = 0; i < virtual_id_image.size(); i++)
      if (virtual_id_image[i])
        selected_points[virtual_id_image[i] - 1] = true;
  }

  Uint64Vector ids;
  for (uint64 i = 0; i < cloud_size; i++)
    if (selected_points[i])
      ids.push_back(i);

  return ids;
}

//删除后重新计算选中的点数
RVizCloudAnnotation::Uint64Vector
RVizCloudAnnotation::RecountIds(const Uint64Vector &ids) {
  Uint64Vector n_ids;

  uint64 i = 0;
  uint64 j = 0;
  while (i < ids.size() && j < ids_in_bbox[BBOX_ID].size()) {
    if (ids_in_bbox[BBOX_ID][j] < ids[i]) {
      j++;
    } else if (ids_in_bbox[BBOX_ID][j] > ids[i]) {
      i++;
    } else {
      ids_in_bbox[BBOX_ID][j] = -1;
      i++;
      j++;
    }
  }

  for (int k = 0; k < ids_in_bbox[BBOX_ID].size(); k++) {
    if (ids_in_bbox[BBOX_ID][k] != -1) {
      n_ids.push_back(ids_in_bbox[BBOX_ID][k]);
    }
  }
  return n_ids;
}

//对选择的点做处理，并生成：BBOX，地面，车道线，路沿
void RVizCloudAnnotation::VectorSelection(const Uint64Vector &ids) {
  if (m_edit_mode == EDIT_MODE_CONTROL_POINT) {
    if (ANNOTATION_TYPE == BBOX) {
      const Uint64Vector changed_labels =
          m_undo_redo.SetControlPointVector(ids, 0, m_current_label);
      ROS_INFO("rviz_cloud_annotation: m_current_label %lu", m_current_label);
      SendControlPointsMarker(changed_labels, true);
      SendPointCounts(changed_labels);
      SendUndoRedoState();
      ROS_INFO("rviz_cloud_annotation: ANNOTATION_TYPE %u", ANNOTATION_TYPE);

      ids_in_bbox[BBOX_ID] = ids;
      generateBbox(*m_cloud, if_tilt);
    } else if (ANNOTATION_TYPE == PLANE) {
      generatePlane(ids, true);
    }

    ROS_INFO("rviz_cloud_annotation: selection set %i points.",
             int(ids.size()));
  } else if (m_edit_mode == EDIT_MODE_ERASER) {
    if (ANNOTATION_TYPE == BBOX) {
      uint64 nowsize = ids_in_bbox[BBOX_ID].size() - ids.size();

      const Uint64Vector changed_labels =
          m_undo_redo.SetControlPointVector(ids, 0, 0);
      SendControlPointsMarker(changed_labels, true);
      SendPointCounts(changed_labels);
      SendUndoRedoState();

      Uint64Vector n_ids = RecountIds(ids);
      ids_in_bbox[BBOX_ID] = n_ids;
      generateBbox(*m_cloud, if_tilt);
    } else if (ANNOTATION_TYPE == PLANE) {
      generatePlane(ids, false);
    }
  } else {
    ROS_WARN("rviz_cloud_annotation: invalid action %i for selection.",
             int(m_edit_mode));
  }
}

//增加时间戳
std::string RVizCloudAnnotation::AppendTimestampBeforeExtension(
    const std::string &filename) {
  std::string datetime = boost::posix_time::to_simple_string(
      boost::posix_time::second_clock::local_time());
  std::replace(datetime.begin(), datetime.end(), ' ', '_');
  std::replace(datetime.begin(), datetime.end(), ':', '-');

  const std::size_t last_dot = filename.rfind('.');
  const std::size_t last_slash = filename.rfind("/");
  if (last_dot == std::string::npos)
    return filename + datetime;
  if (last_slash != std::string::npos && last_slash > last_dot)
    return filename + datetime;

  return filename.substr(0, last_dot) + datetime + filename.substr(last_dot);
}

//生成地面 并发布　Marker
void RVizCloudAnnotation::generatePlane(const Uint64Vector &ids,
                                        bool editMode) {
  if (editMode == true) // Add
  {
    const Uint64Vector changed_labels =
        m_undo_redo.SetControlPointVector(ids, 0, 26);
    SendControlPointsMarker(changed_labels, true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();

    for (int i = 0; i < ids.size(); i++) {
      ids_in_plane_flag[ids[i]] = -3;
    }
    ids_in_plane.insert(ids_in_plane.end(), ids.begin(), ids.end());
  } else // Delete
  {
    const Uint64Vector changed_labels =
        m_undo_redo.SetControlPointVector(ids, 0, 0);
    SendControlPointsMarker(changed_labels, true);
    SendPointCounts(changed_labels);
    SendUndoRedoState();
    for (int i = 0; i < ids.size(); i++) {
      ids_in_plane_flag[ids[i]] = 0;
    }
    for (int k = 0; k < ids_in_plane.size(); k++) {
      for (int m = 0; m < ids.size(); m++) {
        if (ids_in_plane[k] == ids[m]) {
          ids_in_plane[k] = 0;
          break;
        }
      }
    }
  }
}

//生成默认的地面
void RVizCloudAnnotation::generateDefaultPlane(PointXYZRGBNormalCloud &cloud) {
  PointCloudPlaneCurvesExtract *pcpce = new PointCloudPlaneCurvesExtract();
  pcpce->SearchCurves(cloud);
  for (int i = 0; i < _planeRings; i++) {
    mCurveId[i].clear();
    mCurveId[i].insert(mCurveId[i].end(), pcpce->mSizeCurvesId[i].begin(),
                       pcpce->mSizeCurvesId[i].end());
  }
}
// //生成 地面 Marker
// void RVizCloudAnnotation::generatePlane(const PointXYZRGBNormalCloud &cloud)
// {
//   Marker marker;

//   marker.header.frame_id = m_frame_id;
//   marker.header.stamp = ros::Time::now();

//   marker.id = PLANE_ID;

//   marker.action = visualization_msgs::Marker::MODIFY;

//   marker.type = visualization_msgs::Marker::POINTS;

//   marker.scale.x = 0.1;

//   marker.color.r = 0.8;
//   marker.color.g = 0.8;
//   marker.color.b = 0.8;
//   marker.color.a = 0.8;

//   ROS_INFO("rviz_cloud_annotation: pLANE_SIZE[LANE_ID]: %ld",
//   (ids_in_plane.size() - 1));
//   if (ids_in_plane.size() < 3)
//   {
//     return;
//   }

//   for (uint64 i = 0; i < ids_in_plane.size(); i++)
//   {
//     if (ids_in_plane[i] != 0)
//     {
//       // ROS_INFO("rviz_cloud_annotation: ids_in_plane[i]: %ld",
//       ids_in_plane[i]);
//       const PointXYZRGBNormal &ppt = cloud[ids_in_plane[i]];

//       const Eigen::Vector3f ept(ppt.x, ppt.y, ppt.z);
//       geometry_msgs::Point P;
//       P.x = ept.x();
//       P.y = ept.y();
//       P.z = ept.z();
//       marker.points.push_back(P);
//     }
//   }
//   // ROS_INFO("rviz_cloud_annotation: True 1");
//   plane_marker_pub.publish(marker);
//   ROS_INFO("rviz_cloud_annotation: publish plane");
// }

//发布路沿Marker
void RVizCloudAnnotation::generateKerb(const PointXYZRGBNormalCloud &cloud) {
  Marker marker;

  marker.header.frame_id = m_frame_id;
  marker.header.stamp = ros::Time::now();

  // marker.ns = "my_kerb-lists";

  marker.id = KERB_ID;

  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::POINTS;

  marker.scale.x = 0.5;

  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0.5;

  ROS_INFO("rviz_cloud_annotation: Kerb_Edge: %ld",
           (ids_in_kerb[KERB_ID].size() - 1));
  if (ids_in_kerb[KERB_ID].size() < 2) {
    return;
  }

  for (uint64 i = 0; i < ids_in_kerb[KERB_ID].size(); i++) {
    const PointXYZRGBNormal &ppt = cloud[ids_in_kerb[KERB_ID][i]];
    const Eigen::Vector3f ept(ppt.x, ppt.y, ppt.z);

    geometry_msgs::Point P;
    P.x = ept.x();
    P.y = ept.y();
    P.z = ept.z() - 0.01;

    KERB_SET[KERB_ID][i][0] = ept.x();
    KERB_SET[KERB_ID][i][1] = ept.y();
    KERB_SET[KERB_ID][i][2] = ept.z();
    KERB_SET[KERB_ID][i][3] = ids_in_kerb[KERB_ID][i]; // point id

    marker.points.push_back(P);
  }
  KERB_SIZE[KERB_ID] = ids_in_kerb[KERB_ID].size();

  kerb_marker_pub.publish(marker);
}

//发布车道线 Marker
void RVizCloudAnnotation::generateLane(const PointXYZRGBNormalCloud &cloud) {
  Marker marker;

  marker.header.frame_id = m_frame_id;
  marker.header.stamp = ros::Time::now();

  marker.id = LANE_ID;

  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::POINTS;

  marker.scale.x = 0.5;

  marker.color.r = 0;
  marker.color.g = 1;
  marker.color.b = 0;
  marker.color.a = 0.5;

  ROS_INFO("rviz_cloud_annotation: Lane_Edge: %ld",
           (ids_in_lane[LANE_ID].size() - 1));
  if (ids_in_lane[LANE_ID].size() < 2) {
    return;
  }

  for (uint64 i = 0; i < ids_in_lane[LANE_ID].size(); i++) {
    const PointXYZRGBNormal &ppt = cloud[ids_in_lane[LANE_ID][i]];
    const Eigen::Vector3f ept(ppt.x, ppt.y, ppt.z);

    geometry_msgs::Point P;
    P.x = ept.x();
    P.y = ept.y();
    P.z = ept.z() - 0.01;

    LANE_SET[LANE_ID][i][0] = ept.x();
    LANE_SET[LANE_ID][i][1] = ept.y();
    LANE_SET[LANE_ID][i][2] = ept.z();
    LANE_SET[LANE_ID][i][3] = ids_in_lane[LANE_ID][i]; // point id

    marker.points.push_back(P);
  }
  LANE_SIZE[LANE_ID] = ids_in_lane[LANE_ID].size();

  lane_marker_pub.publish(marker);
}

//生成 BBOX信息
void RVizCloudAnnotation::generateBbox(const PointXYZRGBNormalCloud &cloud,
                                       bool if_tilt) {
  if (ids_in_bbox[BBOX_ID].size() < 2) {
    EmptyBboxToMarker(BBOX_ID);
    return;
  }
  const PointXYZRGBNormal &ppt = cloud[ids_in_bbox[BBOX_ID][0]];
  const Eigen::Vector3f ept(ppt.x, ppt.y, ppt.z);

  const Eigen::Vector3f r_ept = ept;
  float *shape_;

  if (if_tilt == false) {
    float shape[6] = {r_ept.x(), r_ept.x(), r_ept.y(),
                      r_ept.y(), r_ept.z(), r_ept.z()};

    for (uint64 i = 1; i < ids_in_bbox[BBOX_ID].size(); i++) {
      const PointXYZRGBNormal &ppt_ = cloud[ids_in_bbox[BBOX_ID][i]];
      const Eigen::Vector3f ept_(ppt_.x, ppt_.y, ppt_.z);

      const Eigen::Vector3f r_ept_ = ept_;

      if (line(r_ept_.x()) < line(shape[0])) {
        shape[0] = r_ept_.x();
      } else if (line(r_ept_.x()) > line(shape[1])) {
        shape[1] = r_ept_.x();
      }

      if (line(r_ept_.y()) < line(shape[2])) {
        shape[2] = r_ept_.y();
      } else if (line(r_ept_.y()) > line(shape[3])) {
        shape[3] = r_ept_.y();
      }

      if (line(r_ept_.z()) < line(shape[4])) {
        shape[4] = r_ept_.z();
      } else if (line(r_ept_.z()) > line(shape[5])) {
        shape[5] = r_ept_.z();
      }
    }
    shape[0] += m_box_bias[BBOX_ID][2];
    shape[1] += m_box_bias[BBOX_ID][3];
    shape[2] += m_box_bias[BBOX_ID][0];
    shape[3] += m_box_bias[BBOX_ID][1];
    shape[4] += m_box_bias[BBOX_ID][4];
    shape[5] += m_box_bias[BBOX_ID][5];

    ROS_INFO(
        "rviz_cloud_annotation: shape[x1 %f x2 %f y1 %f y2 %f z1 %f z3 %f]",
        shape[0], shape[1], shape[2], shape[3], shape[4], shape[5]);
    shape_ = shape;

    AddBbox(0, 0, shape[0], shape[1], shape[2], shape[3], shape[4], shape[5],
            if_tilt);
  } else {
    float s[10] = {r_ept.x(), r_ept.y(), r_ept.x(), r_ept.y(), r_ept.x(),
                   r_ept.y(), r_ept.x(), r_ept.y(), r_ept.z(), r_ept.z()};

    for (uint64 i = 1; i < ids_in_bbox[BBOX_ID].size(); i++) {
      const PointXYZRGBNormal &ppt_ = cloud[ids_in_bbox[BBOX_ID][i]];
      const Eigen::Vector3f ept_(ppt_.x, ppt_.y, ppt_.z);
      const Eigen::Vector3f r_ept_ = ept_;

      float A = tan(BBOX_YAW);
      float B = tan(BBOX_YAW - _M_PI * 0.5);

      if (line(r_ept_.x(), r_ept_.y(), A) < line(s[0], s[1], A)) {
        s[0] = r_ept_.x();
        s[1] = r_ept_.y();
      } else if (line(r_ept_.x(), r_ept_.y(), A) > line(s[2], s[3], A)) {
        s[2] = r_ept_.x();
        s[3] = r_ept_.y();
      }

      if (line(r_ept_.x(), r_ept_.y(), B) < line(s[4], s[5], B)) {
        s[4] = r_ept_.x();
        s[5] = r_ept_.y();
      } else if (line(r_ept_.x(), r_ept_.y(), B) > line(s[6], s[7], B)) {
        s[6] = r_ept_.x();
        s[7] = r_ept_.y();
      }
      if (line(r_ept_.z()) < line(s[8])) {
        s[8] = r_ept_.z();
      } else if (line(r_ept_.z()) > line(s[9])) {
        s[9] = r_ept_.z();
      }
    }

    float T1 = 1 / sin(BBOX_YAW);
    float T2 = 1 / cos(BBOX_YAW);

    s[3] += m_box_bias[BBOX_ID][0] * T2;
    s[1] += m_box_bias[BBOX_ID][1] * T2;
    s[4] += m_box_bias[BBOX_ID][2] * T2;
    s[6] += m_box_bias[BBOX_ID][3] * T2;
    s[8] += m_box_bias[BBOX_ID][4];
    s[9] += m_box_bias[BBOX_ID][5];

    float shape[10];
    float A = tan(BBOX_YAW);
    float B = tan(BBOX_YAW - _M_PI * 0.5);
    float B1 = _max(s[5] - s[4] * B, s[7] - s[6] * B);
    float B2 = _min(s[5] - s[4] * B, s[7] - s[6] * B);
    float B3 = _max(s[3] - s[2] * A, s[1] - s[0] * A);
    float B4 = _min(s[3] - s[2] * A, s[1] - s[0] * A);

    shape[0] = ((s[5] - s[4] * B) - (s[3] - s[2] * A)) / (A - B);
    shape[1] = ((s[5] - s[4] * B) * A - (s[3] - s[2] * A) * B) / (A - B);

    shape[2] = ((s[5] - s[4] * B) - (s[1] - s[0] * A)) / (A - B);
    shape[3] = ((s[5] - s[4] * B) * A - (s[1] - s[0] * A) * B) / (A - B);

    shape[4] = ((s[7] - s[6] * B) - (s[1] - s[0] * A)) / (A - B);
    shape[5] = ((s[7] - s[6] * B) * A - (s[1] - s[0] * A) * B) / (A - B);

    shape[6] = ((s[7] - s[6] * B) - (s[3] - s[2] * A)) / (A - B);
    shape[7] = ((s[7] - s[6] * B) * A - (s[3] - s[2] * A) * B) / (A - B);

    shape[8] = s[8];
    shape[9] = s[9];

    shape_ = shape;

    AddBbox(A, B, B1, B2, B3, B4, shape[9], shape[8], if_tilt);
  }

  BboxToMarker(shape_, BBOX_ID, if_tilt);
}

//发布BBOX Marker
void RVizCloudAnnotation::BboxToMarker(const float shape[], const int id,
                                       bool if_tilt) {
  const pcl::RGB color = pcl::GlasbeyLUT::at((m_current_label - 1) % 256);

  InteractiveMarker marker1;

  Marker marker;

  marker.header.frame_id = m_frame_id;

  marker.id = id;

  marker.type = visualization_msgs::Marker::CUBE;

  marker.color.r = float(color.r) / 255.0;
  marker.color.g = float(color.g) / 255.0;
  marker.color.b = float(color.b) / 255.0;
  marker.color.a = 0.5;

  Marker markerHead;
  markerHead.header.frame_id = m_frame_id;
  markerHead.id = id;
  markerHead.type = visualization_msgs::Marker::CUBE;

  markerHead.color.r = float(color.r) / 255.0;
  markerHead.color.g = float(color.g) / 255.0;
  markerHead.color.b = float(color.b) / 255.0;
  markerHead.color.a = 1;

  markerHead.scale.x = 1;
  markerHead.scale.y = 0.1;
  markerHead.scale.z = 0.1;

  float sinyaw = 0;
  float cosyaw = 1;

  if (if_tilt == false) {
    marker.pose.position.x = 0.5 * (shape[0] + shape[1]);
    marker.pose.position.y = 0.5 * (shape[2] + shape[3]);
    marker.pose.position.z = 0.5 * (shape[4] + shape[5]);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = fabs(shape[0] - shape[1]);
    marker.scale.y = fabs(shape[2] - shape[3]);
    marker.scale.z = fabs(shape[4] - shape[5]);
    printf("180 Yaw %d", BBOX_YAW_ANGLE);
    if (BBOX_YAW_ANGLE == 0) {
      markerHead.pose.position.x =
          marker.pose.position.x + (0.5 * marker.scale.x + 0.5);
      markerHead.pose.position.y = marker.pose.position.y;
    } else if (BBOX_YAW_ANGLE == 90) {
      markerHead.pose.position.x = marker.pose.position.x;
      markerHead.pose.position.y =
          marker.pose.position.y + (0.5 * marker.scale.y + 0.5);
      markerHead.scale.x = 0.1;
      markerHead.scale.y = 1;
    } else if (BBOX_YAW_ANGLE == -90) {
      markerHead.pose.position.x = marker.pose.position.x;
      markerHead.pose.position.y =
          marker.pose.position.y - (0.5 * marker.scale.y + 0.5);
      markerHead.scale.x = 0.1;
      markerHead.scale.y = 1;
    } else if (BBOX_YAW_ANGLE == 180 || BBOX_YAW_ANGLE == -180) {
      markerHead.pose.position.x =
          marker.pose.position.x - (0.5 * marker.scale.x + 0.5);
      markerHead.pose.position.y = marker.pose.position.y;
    }
    markerHead.pose.position.z = marker.pose.position.z;
  } else {
    sinyaw = sinf(BBOX_YAW);
    cosyaw = cosf(BBOX_YAW);

    marker.pose.position.x = 0.5 * (shape[0] + shape[4]);
    marker.pose.position.y = 0.5 * (shape[1] + shape[5]);
    marker.pose.position.z = 0.5 * (shape[8] + shape[9]);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(BBOX_YAW * 0.5);
    marker.pose.orientation.w = cos(BBOX_YAW * 0.5);

    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.y = sqrt((shape[3] - shape[1]) * (shape[3] - shape[1]) +
                          (shape[2] - shape[0]) * (shape[2] - shape[0]));
    marker.scale.x = sqrt((shape[5] - shape[3]) * (shape[5] - shape[3]) +
                          (shape[4] - shape[2]) * (shape[4] - shape[2]));
    marker.scale.z = fabs(shape[8] - shape[9]);

    markerHead.pose.position.x =
        marker.pose.position.x + (0.5 * marker.scale.x + 0.5) * cosyaw;
    markerHead.pose.position.y =
        marker.pose.position.y + (0.5 * marker.scale.x + 0.5) * sinyaw;
    markerHead.pose.position.z = marker.pose.position.z;
  }

  float alpha = marker.pose.position.y > 0
                    ? atan2f(marker.pose.position.x, marker.pose.position.y)
                    : -atan2f(marker.pose.position.x, marker.pose.position.y);

  ROS_INFO("rviz_cloud_annotation: alpha is %f", alpha);

  BBOX_LABEL_SET[BBOX_ID][0] = m_current_label;
  BBOX_LABEL_SET[BBOX_ID][1] = m_bbox_occluded[BBOX_ID];
  BBOX_LABEL_SET[BBOX_ID][2] = marker.scale.x;
  BBOX_LABEL_SET[BBOX_ID][3] = marker.scale.y;
  BBOX_LABEL_SET[BBOX_ID][4] = marker.scale.z;
  BBOX_LABEL_SET[BBOX_ID][5] = marker.pose.position.x;
  BBOX_LABEL_SET[BBOX_ID][6] = marker.pose.position.y;
  BBOX_LABEL_SET[BBOX_ID][7] = marker.pose.position.z;
  BBOX_LABEL_SET[BBOX_ID][8] = alpha;
  BBOX_LABEL_SET[BBOX_ID][9] = BBOX_YAW_ANGLE;

  bbox_marker_pub.publish(marker);

  markerHead.pose.orientation.x = marker.pose.orientation.x;
  markerHead.pose.orientation.y = marker.pose.orientation.y;
  markerHead.pose.orientation.z = marker.pose.orientation.z;
  markerHead.pose.orientation.w = marker.pose.orientation.w;

  bbox_head_marker_pub.publish(markerHead);
}

//删除BBOX Marker
void RVizCloudAnnotation::EmptyBboxToMarker(const int id) {
  BBOX_SET[id][10] = 0;

  Marker marker;

  marker.header.frame_id = m_frame_id;

  marker.id = id;

  marker.action = visualization_msgs::Marker::DELETE;

  marker.scale.x = 0;
  marker.scale.y = 0;
  marker.scale.z = 0;

  Marker markerHead;

  markerHead.header.frame_id = m_frame_id;

  markerHead.id = id;

  markerHead.action = visualization_msgs::Marker::DELETE;

  markerHead.scale.x = 0;
  markerHead.scale.y = 0;
  markerHead.scale.z = 0;

  bbox_marker_pub.publish(marker);

  bbox_head_marker_pub.publish(markerHead);

  ROS_INFO("rviz_cloud_annotation: Delete BBOX now: %i", BBOX_ID);
}

//删除 路沿 Marker
void RVizCloudAnnotation::EmptyKerbToMarker(const int id) {
  Marker marker;

  marker.header.frame_id = m_frame_id;

  marker.id = id;

  marker.action = visualization_msgs::Marker::DELETE;

  kerb_marker_pub.publish(marker);
}

//删除 地面 Marker
void RVizCloudAnnotation::EmptyPlaneToMarker(const int id) {
  Marker marker;

  marker.header.frame_id = m_frame_id;

  marker.id = id;

  marker.action = visualization_msgs::Marker::DELETE;

  plane_marker_pub.publish(marker);
}

//删除 车道线 Marker
void RVizCloudAnnotation::EmptyLaneToMarker(const int id) {
  Marker marker;

  marker.header.frame_id = m_frame_id;

  marker.id = id;

  marker.action = visualization_msgs::Marker::DELETE;

  lane_marker_pub.publish(marker);
}

//计算直线
float RVizCloudAnnotation::line(float v) { return v; }
//计算倾斜直线
float RVizCloudAnnotation::line(float x, float y, float k) {
  float b;

  b = y - k * x;

  return b;
}

//记录 BBOX  信息
void RVizCloudAnnotation::AddBbox(float A, float B, float B1, float B2,
                                  float B3, float B4, float C1, float C2,
                                  bool tilt) {
  BBOX_SET[BBOX_ID][0] = A;
  BBOX_SET[BBOX_ID][1] = B1;
  BBOX_SET[BBOX_ID][2] = B2;
  BBOX_SET[BBOX_ID][3] = B;
  BBOX_SET[BBOX_ID][4] = B3;
  BBOX_SET[BBOX_ID][5] = B4;
  BBOX_SET[BBOX_ID][6] = C1;
  BBOX_SET[BBOX_ID][7] = C2;
  BBOX_SET[BBOX_ID][8] = m_current_label;

  BBOX_SET[BBOX_ID][10] = 1000;

  if (tilt == true) {
    BBOX_SET[BBOX_ID][9] = 1;
  } else {
    BBOX_SET[BBOX_ID][9] = -1;
  }
  ROS_INFO("BBOX[%d]:%f %f %f %f %f %f %f %f %f %f", BBOX_ID,
           BBOX_SET[BBOX_ID][0], BBOX_SET[BBOX_ID][1], BBOX_SET[BBOX_ID][2],
           BBOX_SET[BBOX_ID][3], BBOX_SET[BBOX_ID][4], BBOX_SET[BBOX_ID][5],
           BBOX_SET[BBOX_ID][6], BBOX_SET[BBOX_ID][7], BBOX_SET[BBOX_ID][8],
           BBOX_SET[BBOX_ID][9]);
}

// 导出车道线信息
void RVizCloudAnnotation::ExportLane(PointXYZRGBNormalCloud &cloud) {
  std::ofstream ofile;
  std::string save_label_name = m_label_files[FILE_ID];
  std::string line_path = save_label_name + ".csv";
  ofile.open(line_path.c_str(), std::ios::trunc);

  ROS_INFO("rviz_cloud_annotation: lane name: %s", line_path.c_str());

  for (int k = 0; k < LINENUMBER; k++) {
    for (int i = 0; i < ids_in_lane[k].size(); i++) {
      int index = ids_in_lane[k][i];
      ofile << index << "\t" << cloud[index].x << "\t" << cloud[index].y << "\t"
            << cloud[index].z << "\n";
    }
  }
  ofile.close();
}

//保存时最后生成每个点的Label信息
void RVizCloudAnnotation::FinalLabel(PointXYZRGBNormalCloud &cloud) {
  const uint64 cloud_size = cloud.size();

  Uint64Vector kerb_ids;
  Uint64Vector lane_ids;

  for (int i = 0; i < cloud_size; i++) {
    int64_t label = 2000;

    if (ids_in_plane_flag[i] == -3) {
      label = -3;
    }

    // if in line
    for (int k = 0; k <= KERB_ID; k++) {
      if (InKerb(cloud[i].x, cloud[i].y, cloud[i].z, i, k) == true) {
        label = -1;
        kerb_ids.push_back(i);
      }
    }
    for (int k = 0; k <= LANE_ID; k++) {
      if (InLane(cloud[i].x, cloud[i].y, cloud[i].z, i, k) == true) {
        label = -2;
        lane_ids.push_back(i);
      }
    }
    // if in Bbox
    for (int j = 0; j <= BBOX_ID; j++) {
      if (BBOX_SET[j][10] < 1) { //空BBOX
        continue;
      }
      // ROS_INFO("EXSIT BBOX1");
      if (InBbox(cloud[i].x, cloud[i].y, cloud[i].z, j) == true) {
        // ROS_INFO("EXSIT BBOX POINT");
        label = (int)BBOX_SET[j][8];
      }
    }

    m_label.push_back(label);
  }

  const Uint64Vector changed_labels =
      m_undo_redo.SetControlPointVector(kerb_ids, 0, 1);
  SendControlPointsMarker(changed_labels, true);
  SendPointCounts(changed_labels);
  SendUndoRedoState();

  const Uint64Vector changed_labels1 =
      m_undo_redo.SetControlPointVector(lane_ids, 0, 13);
  SendControlPointsMarker(changed_labels1, true);
  SendPointCounts(changed_labels1);
  SendUndoRedoState();
}

//点是否在 BBOX 内部
bool RVizCloudAnnotation::InBbox(float x, float y, float z, int i) {
  float A = BBOX_SET[i][0];
  float B1 = BBOX_SET[i][1];
  float B2 = BBOX_SET[i][2];
  float B = BBOX_SET[i][3];
  float B3 = BBOX_SET[i][4];
  float B4 = BBOX_SET[i][5];
  float C1 = BBOX_SET[i][6];
  float C2 = BBOX_SET[i][7];

  if (BBOX_SET[i][9] < 0) // no tilt
  {
    if (x <= B2 && x >= B1 && y <= B4 && y >= B3 && z <= C2 && z >= C1) {
      return true;
    } else {
      return false;
    }
  } else if (BBOX_SET[i][9] > 0) // tilt
  {
    if ((y - B * x - B1) <= 0 && (y - B * x - B2) >= 0 &&
        (y - A * x - B3) <= 0 && (y - A * x - B4) >= 0 && z <= C1 && z >= C2) {
      return true;
    } else {
      return false;
    }
  }
}

//点是否在路沿上
bool RVizCloudAnnotation::InKerb(float x, float y, float z, uint64 id,
                                 int kerb_id) {
  return false;
  for (int i = 0; i < KERB_SIZE[kerb_id]; i++) {
    if (ids_in_kerb[kerb_id][i] == id) {
      return true;
    }
  }

  for (int i = 0; i < KERB_SIZE[kerb_id] - 1; i++) {
    float x2 = KERB_SET[kerb_id][i + 1][0];
    float x1 = KERB_SET[kerb_id][i][0];
    float y2 = KERB_SET[kerb_id][i + 1][1];
    float y1 = KERB_SET[kerb_id][i][1];
    float z2 = KERB_SET[kerb_id][i + 1][2];
    float z1 = KERB_SET[kerb_id][i][2];

    float a1 = x1 - x;
    float a2 = y1 - y;
    float a3 = z1 - z;

    float A = sqrt(a1 * a1 + a2 * a2 + a3 * a3);

    float b1 = x2 - x1;
    float b2 = y2 - y1;
    float b3 = z2 - z1;

    float B = sqrt(b1 * b1 + b2 * b2 + b3 * b3);

    float a_b1 = a2 * b3 - a3 * b2;
    float a_b2 = a3 * b1 - a1 * b3;
    float a_b3 = a1 * b2 - a2 * b1;
    float C = sqrt(a_b1 * a_b1 + a_b2 * a_b2 + a_b3 * a_b3);

    if (C / B < 0.05 && A < B) {
      return true;
    }
  }
  return false;
}

//点是否在 车道线上
bool RVizCloudAnnotation::InLane(float x, float y, float z, uint64 id,
                                 int lane_id) {
  return false;
  for (int i = 0; i < LANE_SIZE[lane_id]; i++) {
    if (ids_in_lane[lane_id][i] == id) {
      return true;
    }
  }

  for (int i = 0; i < LANE_SIZE[lane_id] - 1; i++) {
    float x2 = LANE_SET[lane_id][i + 1][0];
    float x1 = LANE_SET[lane_id][i][0];
    float y2 = LANE_SET[lane_id][i + 1][1];
    float y1 = LANE_SET[lane_id][i][1];
    float z2 = LANE_SET[lane_id][i + 1][2];
    float z1 = LANE_SET[lane_id][i][2];

    float a1 = x1 - x;
    float a2 = y1 - y;
    float a3 = z1 - z;

    float A = sqrt(a1 * a1 + a2 * a2 + a3 * a3);

    float b1 = x2 - x1;
    float b2 = y2 - y1;
    float b3 = z2 - z1;

    float B = sqrt(b1 * b1 + b2 * b2 + b3 * b3);

    float a_b1 = a2 * b3 - a3 * b2;
    float a_b2 = a3 * b1 - a1 * b3;
    float a_b3 = a1 * b2 - a2 * b1;
    float C = sqrt(a_b1 * a_b1 + a_b2 * a_b2 + a_b3 * a_b3);

    if (C / B < 0.05 && A < B) {
      return true;
    }
  }
  return false;
}