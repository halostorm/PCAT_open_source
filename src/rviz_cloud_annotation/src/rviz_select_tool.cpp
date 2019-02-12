#include "rviz_select_tool.h"
#include "rviz_cloud_annotation.h"

#include <ros/time.h>

#include <rviz/default_plugin/tools/move_tool.h>
#include <rviz/default_plugin/tools/interaction_tool.h>

#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/viewport_mouse_event.h>

#include <rviz_cloud_annotation/RectangleSelectionViewport.h>

#include <OgreManualObject.h>

#include <cmath>

namespace rviz_cloud_annotation
{
AnnotationSelectionTool::AnnotationSelectionTool(): m_nh("~")
{
  access_all_keys_ = true;

  std::string param_string;

  m_nh.param<std::string>(PARAM_NAME_RECT_SELECTION_TOPIC,param_string,PARAM_DEFAULT_RECT_SELECTION_TOPIC);
  m_annotation_selection_pub = m_nh.advertise<rviz_cloud_annotation::RectangleSelectionViewport>(param_string,1);

  m_nh.param<std::string>(PARAM_NAME_SET_EDIT_MODE_TOPIC2,param_string,PARAM_DEFAULT_SET_EDIT_MODE_TOPIC2);
  m_on_set_mode_sub = m_nh.subscribe(param_string,1,&AnnotationSelectionTool::onSetEditMode,this);

  m_nh.param<std::string>(PARAM_NAME_TOOL_TYPE_TOPIC,param_string,PARAM_DEFAULT_TOOL_TYPE_TOPIC);
  m_on_set_tool_type_sub = m_nh.subscribe(param_string,1,&AnnotationSelectionTool::onSetToolType,this);

  m_nh.param<std::string>(PARAM_NAME_TOGGLE_NONE_TOPIC,param_string,PARAM_DEFAULT_TOGGLE_NONE_TOPIC);
  m_toggle_none_pub = m_nh.advertise<std_msgs::Empty>(param_string,1);

  m_move_tool = new rviz::MoveTool;
  m_interaction_tool = new rviz::InteractionTool;

  m_edit_mode = EDIT_MODE_NONE;
  m_edit_mode_selectable = false;
  m_tool_type = TOOL_TYPE_SINGLE_PICK;

  m_active = false;
  m_selecting = false;
  m_polyline_selecting = false;
  m_sel_start_x = 0;
  m_sel_start_y = 0;

  m_selection_rectangle = NULL;
  m_selection_rectangle_node = NULL;
  m_selection_viewport = NULL;
}

AnnotationSelectionTool::~AnnotationSelectionTool()
{
  delete m_move_tool;
  delete m_interaction_tool;
}

void AnnotationSelectionTool::onInitialize()
{
  m_move_tool->initialize(context_);
  m_interaction_tool->initialize(context_);

  InitRectangleSelection();
  InitPolylineSelection();
}

void AnnotationSelectionTool::InitRectangleSelection()
{
  std::stringstream ss;
  ss << "RVizCloudAnnotationSelectionTool" << std::hex << intptr_t(this);
  const std::string base_name = ss.str();

  m_selection_rectangle_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  m_selection_rectangle = new Ogre::Rectangle2D(true);

  const static uint32_t texture_data[1] = { 0xffff0080 };
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 4 ));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(base_name + "Texture",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, 1, 1, Ogre::PF_R8G8B8A8,
      Ogre::TEX_TYPE_2D, 0);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(base_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  m_selection_rectangle->setMaterial(material->getName());
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  m_selection_rectangle->setBoundingBox(aabInf);
  m_selection_rectangle->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tex_unit->setTextureName(tex->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  m_selection_rectangle_node->attachObject(m_selection_rectangle);
}

void AnnotationSelectionTool::InitPolylineSelection()
{
  std::stringstream ss;
  ss << "RVizCloudAnnotationSelectionToolPoly" << std::hex << intptr_t(this);
  const std::string base_name = ss.str();

  const static uint32_t texture_data[1] = { 0xff000080 };
  Ogre::DataStreamPtr pixel_stream;
  pixel_stream.bind(new Ogre::MemoryDataStream( (void*)&texture_data[0], 4 ));

  Ogre::TexturePtr tex = Ogre::TextureManager::getSingleton().loadRawData(base_name + "Texture",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      pixel_stream, 1, 1, Ogre::PF_R8G8B8A8,
      Ogre::TEX_TYPE_2D, 0);

  Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(base_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setLightingEnabled(false);
  material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  material->setCullingMode(Ogre::CULL_NONE);

  Ogre::TextureUnitState* tex_unit = material->getTechnique(0)->getPass(0)->createTextureUnitState();
  tex_unit->setTextureName(tex->getName());
  tex_unit->setTextureFiltering(Ogre::TFO_NONE);

  m_selection_polyline_node = scene_manager_->getRootSceneNode()->createChildSceneNode();

  m_selection_polyline = scene_manager_->createManualObject(base_name + "manual");
  m_selection_polyline->setUseIdentityProjection(true);
  m_selection_polyline->setUseIdentityView(true);

  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  m_selection_polyline->setBoundingBox(aabInf);
  m_selection_polyline->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);

  m_selection_polyline->begin(base_name,Ogre::RenderOperation::OT_TRIANGLE_LIST);

  m_selection_polyline->position(0.0,0.0,0.0);
  m_selection_polyline->position(1.0,1.0,0.0);
  m_selection_polyline->position(-1.0,1.0,0.0);

  m_selection_polyline->end();

  m_selection_polyline_node->attachObject(m_selection_polyline);
  m_selection_polyline_node->setVisible(false);
}

void AnnotationSelectionTool::UpdatePolylineSelection()
{
  const uint32 poly_size = m_sel_polyline.size();

  FloatVector conv_x(poly_size);
  FloatVector conv_y(poly_size);

  const float viewport_width = m_selection_viewport->getActualWidth();
  const float viewport_height = m_selection_viewport->getActualHeight();

  const float LINE_WIDTH_PX = 1.0;
  const float wx = (LINE_WIDTH_PX / viewport_width) * 2.0f;
  const float wy = (LINE_WIDTH_PX / viewport_height) * 2.0f;

  for (uint32 i = 0; i < poly_size; i++)
  {
    const Coords2D & c = m_sel_polyline[i];
    float nx = (c.x / viewport_width) * 2.0f - 1.0f;
    float ny = (c.y / viewport_height) * 2.0f - 1.0f;

    nx = nx < -1.0f ? -1.0f : (nx > 1.0f ? 1.0f : nx);
    ny = ny < -1.0f ? -1.0f : (ny > 1.0f ? 1.0f : ny);
    ny = -ny;

    conv_x[i] = nx;
    conv_y[i] = ny;
  }

  m_selection_polyline->beginUpdate(0);

  for (uint32 i = 1; i <= poly_size; i++)
  {
    const float cx = conv_x[i % poly_size];
    const float cy = conv_y[i % poly_size];
    const float px = conv_x[i - 1];
    const float py = conv_y[i - 1];
    const float ny = (cx - px);
    const float nx = -(cy - py);
    const float nnorm = std::sqrt(ny*ny + nx*nx);
    if (nnorm < 0.01)
      continue;
    const float dx = wx * nx / nnorm;
    const float dy = wy * ny / nnorm;
    m_selection_polyline->position(px - dx,py - dy,0.0);
    m_selection_polyline->position(px + dx,py + dy,0.0);
    m_selection_polyline->position(cx + dx,cy + dy,0.0);

    m_selection_polyline->position(px - dx,py - dy,0.0);
    m_selection_polyline->position(cx + dx,cy + dy,0.0);
    m_selection_polyline->position(cx - dx,cy - dy,0.0);
  }

  m_selection_polyline->end();
}

void AnnotationSelectionTool::ClearSelection()
{
  m_selecting = false;
  m_polyline_selecting = false;
}

void AnnotationSelectionTool::activate()
{
  setStatus("Annotation tool.");

  ClearSelection();
  m_active = true;
  UpdateCursor();
}

void AnnotationSelectionTool::deactivate()
{
  m_active = false;
  ClearSelection();
}

void AnnotationSelectionTool::update(float wall_dt, float ros_dt)
{
  m_selection_rectangle_node->setVisible(m_selecting);
  m_selection_polyline_node->setVisible(m_polyline_selecting);

  if (m_selecting)
  {
    float nx1 = ((float)m_sel_start_x / m_selection_viewport->getActualWidth()) * 2 - 1;
    float nx2 = ((float)m_sel_end_x / m_selection_viewport->getActualWidth()) * 2 - 1;
    float ny1 = -(((float)m_sel_start_y / m_selection_viewport->getActualHeight()) * 2 - 1);
    float ny2 = -(((float)m_sel_end_y / m_selection_viewport->getActualHeight()) * 2 - 1);

    nx1 = nx1 < -1 ? -1 : (nx1 > 1 ? 1 : nx1);
    ny1 = ny1 < -1 ? -1 : (ny1 > 1 ? 1 : ny1);
    nx2 = nx2 < -1 ? -1 : (nx2 > 1 ? 1 : nx2);
    ny2 = ny2 < -1 ? -1 : (ny2 > 1 ? 1 : ny2);

    m_selection_rectangle->setCorners(nx1,ny1,nx2,ny2);
  }

  if (m_polyline_selecting)
    UpdatePolylineSelection();
}

void AnnotationSelectionTool::onSetEditMode(const std_msgs::UInt32 & mode)
{
  if (m_edit_mode != mode.data)
    ClearSelection();

  m_edit_mode = mode.data;
  m_edit_mode_selectable =
    m_edit_mode == EDIT_MODE_CONTROL_POINT ||
    m_edit_mode == EDIT_MODE_ERASER;

  if (m_active)
    UpdateCursor();
}

void AnnotationSelectionTool::onSetToolType(const std_msgs::UInt32 & type)
{
  if (m_tool_type != type.data)
    ClearSelection();

  m_tool_type = type.data;

  if (m_active)
    UpdateCursor();
}

int AnnotationSelectionTool::processKeyEvent(QKeyEvent * event,rviz::RenderPanel * panel)
{
  if (event->key() == Qt::Key_Insert)
  {
    m_toggle_none_pub.publish(std_msgs::Empty());
    return 0;
  }

  if (event->key() == Qt::Key_Escape &&
      m_polyline_selecting)
  {
    ClearSelection();
    return Render;
  }

  return 0;
}

void AnnotationSelectionTool::UpdateCursor()
{
  if (m_edit_mode_selectable &&
          (m_tool_type == TOOL_TYPE_SHALLOW_SQUARE ||
           m_tool_type == TOOL_TYPE_DEEP_SQUARE ||
           m_tool_type == TOOL_TYPE_SHALLOW_POLY))
  {
    setCursor(Qt::CrossCursor);
  }
  else if (m_edit_mode == EDIT_MODE_NONE)
  {
    setCursor(m_move_tool->getCursor());
  }
  else
  {
    setCursor(Qt::PointingHandCursor);
  }
}

int AnnotationSelectionTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if ((m_tool_type == TOOL_TYPE_SINGLE_PICK || !m_edit_mode_selectable) &&
      (event.leftDown() || event.leftUp()))
  {
    return m_interaction_tool->processMouseEvent(event);
  }

  if (event.wheel_delta != 0 && !m_selecting && !m_polyline_selecting)
  {
    return m_move_tool->processMouseEvent(event);
  }

  if (event.rightDown() && !m_selecting && !m_polyline_selecting)
  {
    m_toggle_none_pub.publish(std_msgs::Empty());
    return 0;
  }

  if (m_edit_mode == EDIT_MODE_NONE)
  {
    return m_move_tool->processMouseEvent(event);
  }

  if (event.leftDown() &&
      (m_tool_type == TOOL_TYPE_DEEP_SQUARE || m_tool_type == TOOL_TYPE_SHALLOW_SQUARE))
  {
    StartRectangleSelection(event.x,event.y,event.viewport);

    return Render;
  }

  if (event.leftDown() &&
      (m_tool_type == TOOL_TYPE_SHALLOW_POLY) &&
      !m_polyline_selecting)
  {
    StartPolylineSelection(event.x,event.y,event.viewport);

    return Render;
  }

  if (m_selecting) //点选操作
  {
    m_sel_end_x = event.x;
    m_sel_end_y = event.y;
    if(event.leftUp())
    {
      SendViewportData(m_sel_start_x,m_sel_start_y,m_sel_end_x,m_sel_end_y,m_selection_viewport);
      ClearSelection();
    }
    return Render;
  }

  if (m_polyline_selecting) // 多边形选择操作
  {
    m_sel_polyline.back() = Coords2D(event.x,event.y);

    if (event.leftDown())
    {
      m_sel_polyline.push_back(Coords2D(event.x,event.y));
    }

    if (event.rightDown())
    {
      SendViewportPolylineData(m_selection_viewport,m_sel_polyline);
      ClearSelection();
    }

    return Render;
  }

  return 0;
}

void AnnotationSelectionTool::StartRectangleSelection(const int32 start_x,const int32 start_y,Ogre::Viewport * viewport)
{
  m_selecting = true;
  m_sel_start_x = start_x;
  m_sel_start_y = start_y;
  m_sel_end_x = start_x;
  m_sel_end_y = start_y;
  m_selection_viewport = viewport;
}

void AnnotationSelectionTool::StartPolylineSelection(const int32 start_x,const int32 start_y,Ogre::Viewport * viewport)
{
  m_sel_polyline.clear();
  m_sel_polyline.push_back(Coords2D(start_x,start_y));
  m_sel_polyline.push_back(Coords2D(start_x,start_y));
  m_polyline_selecting = true;
  m_polyline_self_intersect = false;
  m_selection_viewport = viewport;
}

void AnnotationSelectionTool::SendViewportPolylineData(const Ogre::Viewport * const viewport,
                                                       const Coords2DVector & polyline)
{
  const uint32 polyline_size = polyline.size();
  if (polyline_size < 3)
    return;

  int32 x1 = polyline[0].x;
  int32 x2 = polyline[0].x;
  int32 y1 = polyline[0].y;
  int32 y2 = polyline[0].y;

  for (uint32 i = 1; i < polyline_size; i++)
  {
    x1 = std::min(x1,polyline[i].x);
    x2 = std::max(x2,polyline[i].x);
    y1 = std::min(y1,polyline[i].y);
    y2 = std::max(y2,polyline[i].y);
  }

  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;

  if (x1 > viewport->getActualWidth() - 1) x1 = viewport->getActualWidth() - 1;
  if (y1 > viewport->getActualHeight() - 1) y1 = viewport->getActualHeight() - 1;

  if (x2 < 0) x2 = 0;
  if (y2 < 0) y2 = 0;
  if (x2 > viewport->getActualWidth() - 1) x2 = viewport->getActualWidth() - 1;
  if (y2 > viewport->getActualHeight() - 1) y2 = viewport->getActualHeight() - 1;

  const Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  const Ogre::Vector3 camera_position = viewport->getCamera()->getDerivedPosition();
  const Ogre::Quaternion camera_orientation = viewport->getCamera()->getDerivedOrientation();

  const float fovy = viewport->getCamera()->getFOVy().valueRadians();
  const float focal_length_px = viewport->getActualHeight() * std::tan((M_PI / 2) - (fovy / 2));

  rviz_cloud_annotation::RectangleSelectionViewport msg;

  for (uint32 i = 0; i < polyline_size; i++)
  {
    msg.polyline_x.push_back(polyline[i].x);
    msg.polyline_y.push_back(polyline[i].y);
  }

  for (uint32 y = 0; y < 4; y++)
    for (uint32 x = 0; x < 4; x++)
      msg.projection_matrix[x + y * 4] = proj_matrix[y][x];

  msg.camera_pose.orientation.x = camera_orientation.x;
  msg.camera_pose.orientation.y = camera_orientation.y;
  msg.camera_pose.orientation.z = camera_orientation.z;
  msg.camera_pose.orientation.w = camera_orientation.w;

  msg.camera_pose.position.x = camera_position.x;
  msg.camera_pose.position.y = camera_position.y;
  msg.camera_pose.position.z = camera_position.z;

  msg.viewport_width = viewport->getActualWidth();
  msg.viewport_height = viewport->getActualHeight();

  msg.focal_length = focal_length_px;

  msg.is_deep_selection = false;

  msg.start_x = x1;
  msg.end_x = x2;
  msg.start_y = y1;
  msg.end_y = y2;

  m_annotation_selection_pub.publish(msg);
}

void AnnotationSelectionTool::SendViewportData(const int32 start_x,const int32 start_y,
                                         const int32 end_x,const int32 end_y,
                                         const Ogre::Viewport * const viewport)
{
  int32 x1 = std::min(start_x,end_x + 1);
  int32 x2 = std::max(start_x,end_x + 1);
  int32 y1 = std::min(start_y,end_y + 1);
  int32 y2 = std::max(start_y,end_y + 1);

  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;

  if (x1 > viewport->getActualWidth() - 1) x1 = viewport->getActualWidth() - 1;
  if (y1 > viewport->getActualHeight() - 1) y1 = viewport->getActualHeight() - 1;

  if (x2 < 0) x2 = 0;
  if (y2 < 0) y2 = 0;
  if (x2 > viewport->getActualWidth() - 1) x2 = viewport->getActualWidth() - 1;
  if (y2 > viewport->getActualHeight() - 1) y2 = viewport->getActualHeight() - 1;

  const Ogre::Matrix4 proj_matrix = viewport->getCamera()->getProjectionMatrix();
  const Ogre::Vector3 camera_position = viewport->getCamera()->getDerivedPosition();
  const Ogre::Quaternion camera_orientation = viewport->getCamera()->getDerivedOrientation();

  const float fovy = viewport->getCamera()->getFOVy().valueRadians();
  const float focal_length_px = viewport->getActualHeight() * std::tan((M_PI / 2) - (fovy / 2));

  rviz_cloud_annotation::RectangleSelectionViewport msg;

  for (uint32 y = 0; y < 4; y++)
    for (uint32 x = 0; x < 4; x++)
      msg.projection_matrix[x + y * 4] = proj_matrix[y][x];

  msg.camera_pose.orientation.x = camera_orientation.x;
  msg.camera_pose.orientation.y = camera_orientation.y;
  msg.camera_pose.orientation.z = camera_orientation.z;
  msg.camera_pose.orientation.w = camera_orientation.w;

  msg.camera_pose.position.x = camera_position.x;
  msg.camera_pose.position.y = camera_position.y;
  msg.camera_pose.position.z = camera_position.z;

  msg.viewport_width = viewport->getActualWidth();
  msg.viewport_height = viewport->getActualHeight();

  msg.focal_length = focal_length_px;

  msg.is_deep_selection = (m_tool_type == TOOL_TYPE_DEEP_SQUARE);

  msg.start_x = x1;
  msg.end_x = x2;
  msg.start_y = y1;
  msg.end_y = y2;

  m_annotation_selection_pub.publish(msg);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_cloud_annotation::AnnotationSelectionTool,rviz::Tool)
