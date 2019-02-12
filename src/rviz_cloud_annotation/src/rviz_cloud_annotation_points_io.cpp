#include "rviz_cloud_annotation_points.h"
#include "rviz_cloud_annotation.h"

#define MAGIC_STRING "ANNOTATION"
#define MAGIC_MIN_VERSION (1)
#define MAGIC_MAX_VERSION (4)

RVizCloudAnnotationPoints::Ptr RVizCloudAnnotationPoints::Deserialize(std::istream & ifile,
                                                                      const uint32 weight_steps,
                                                                      PointNeighborhood::ConstPtr neighborhood)
{
  if (!ifile)
    throw IOE("Invalid file stream.");

  const std::string magic_string = MAGIC_STRING;
  Uint8Vector maybe_magic_string(magic_string.size() + 1);
  ifile.read((char *)(maybe_magic_string.data()),magic_string.size() + 1);
  if (!ifile)
    throw IOE("Unexpected EOF while reading magic string.");
  if (std::memcmp(magic_string.c_str(),maybe_magic_string.data(),magic_string.size() + 1) != 0)
    throw IOE("Invalid magic string.");

  uint64 version;
  ifile.read((char *)&version,sizeof(version));
  if (!ifile)
    throw IOE("Unexpected EOF while reading version.");
  if (version < MAGIC_MIN_VERSION || version > MAGIC_MAX_VERSION)
    throw IOE(std::string("Invalid version: ") + boost::lexical_cast<std::string>(version));

  uint64 cloud_size;
  ifile.read((char *)&cloud_size,sizeof(cloud_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading cloud size.");

  {
    const PointNeighborhood::Conf & conf = neighborhood->GetConf();
    float position_importance;
    ifile.read((char *)&position_importance,sizeof(position_importance));
    float color_importance;
    ifile.read((char *)&color_importance,sizeof(color_importance));
    float normal_importance;
    ifile.read((char *)&normal_importance,sizeof(normal_importance));
    float max_distance;
    ifile.read((char *)&max_distance,sizeof(max_distance));

    PointNeighborhoodSearch::Searcher::ConstPtr searcher;
    if (version <= 2)
    {
      float search_distance;
      ifile.read((char *)&search_distance,sizeof(search_distance));
      searcher = PointNeighborhoodSearch::CreateFromString(0,boost::lexical_cast<std::string>(search_distance));
    }
    else
    {
      try
      {
        searcher = PointNeighborhoodSearch::CreateFromIstream(ifile);
      }
      catch (const PointNeighborhoodSearch::ParserException & ex)
      {
        throw IOE(std::string("Invalid neighborhood configuration parameters: ") + ex.message);
      }
    }

    if (!ifile)
      throw IOE("Unexpected EOF while reading neighborhood configuration parameters.");

    if (position_importance != conf.position_importance ||
      color_importance != conf.color_importance ||
      normal_importance != conf.normal_importance ||
      max_distance != conf.max_distance ||
      !searcher->ApproxEquals(*conf.searcher))
    {
      const uint64 w = 30;
      std::ostringstream msg;
      msg << "Loaded neighborhood configuration parameters do not match: \n"
          << std::setw(w) << "Name" << std::setw(w) << "ROS param" << std::setw(w) << "File" << "\n"
          << std::setw(w) << PARAM_NAME_POSITION_IMPORTANCE
            << std::setw(w) << conf.position_importance << std::setw(w) << position_importance << "\n"
          << std::setw(w) << PARAM_NAME_COLOR_IMPORTANCE
            << std::setw(w) << conf.color_importance << std::setw(w) << color_importance << "\n"
          << std::setw(w) << PARAM_NAME_NORMAL_IMPORTANCE
            << std::setw(w) << conf.normal_importance << std::setw(w) << normal_importance << "\n"
          << std::setw(w) << PARAM_NAME_MAX_DISTANCE
            << std::setw(w) << conf.max_distance << std::setw(w) << max_distance << "\n"
          << std::setw(w) << PARAM_NAME_NEIGH_SEARCH_TYPE
            << std::setw(w) << conf.searcher->GetId() << std::setw(w) << searcher->GetId() << "\n"
          << std::setw(w) << PARAM_NAME_NEIGH_SEARCH_PARAMS
            << std::setw(w) << conf.searcher->ToString() << std::setw(w) << searcher->ToString() << "\n"
          ;
      throw IOE(msg.str());
    }

  }

  {
    uint32 file_weight_steps;
    if (version < 4)
      file_weight_steps = weight_steps;
    else
      ifile.read((char *)&file_weight_steps,sizeof(file_weight_steps));

    if (weight_steps != file_weight_steps)
    {
      const uint64 w = 30;
      std::ostringstream msg;
      msg << "Weight configuration does not match: \n"
          << std::setw(w) << "Name" << std::setw(w) << "ROS param" << std::setw(w) << "File" << "\n"
          << std::setw(w) << PARAM_NAME_WEIGHT_STEPS
          << std::setw(w) << weight_steps << std::setw(w) << file_weight_steps << "\n";
      throw IOE(msg.str());
    }
  }

  RVizCloudAnnotationPoints::Ptr resultptr(new RVizCloudAnnotationPoints(cloud_size,weight_steps,neighborhood));
  RVizCloudAnnotationPoints & result = *resultptr;

  uint64 control_points_size;
  ifile.read((char *)&control_points_size,sizeof(control_points_size));
  if (!ifile)
    throw IOE("Unexpected EOF while reading control point size.");
  result.ExpandLabelsUntil(control_points_size);

  for (uint64 i = 0; i < control_points_size; i++)
  {
    uint64 control_point_size;
    ifile.read((char *)&control_point_size,sizeof(control_point_size));
    if (!ifile)
      throw IOE("Unexpected EOF while reading size of control point " + boost::lexical_cast<std::string>(i) + ".");

    for (uint64 h = 0; h < control_point_size; h++)
    {
      uint64 point_index;
      ifile.read((char *)&point_index,sizeof(point_index));
      if (!ifile)
        throw IOE("Unexpected EOF while reading point index of control point " + boost::lexical_cast<std::string>(i) + ".");

      uint32 weight_step;
      if (version >= 4)
        ifile.read((char *)&weight_step,sizeof(weight_step));
      else
        weight_step = weight_steps;
      if (!ifile)
        throw IOE("Unexpected EOF while reading weight of control point " + boost::lexical_cast<std::string>(i) + ".");
      if (weight_step > weight_steps)
        throw IOE("Weight step " + boost::lexical_cast<std::string>(weight_step) + " is out of range (max is " +
                  boost::lexical_cast<std::string>(weight_steps) + " ) " + "while reading  control point " +
                  boost::lexical_cast<std::string>(i) + ".");

      result.InternalSetControlPoint(point_index,weight_step,i + 1);
    }
  }

  if (version >= 2)
  {
    for (uint64 i = 0; i < control_points_size; i++)
    {
      uint32 string_size;
      ifile.read((char *)&string_size,sizeof(string_size));
      if (!ifile)
        throw IOE("Unexpected EOF while reading text label size " + boost::lexical_cast<std::string>(i) + ".");
      Uint8Vector data(string_size + 1,0); // this is 0-terminated for sure
      ifile.read((char *)(data.data()),string_size);
      if (!ifile)
        throw IOE("Unexpected EOF while reading text label content " + boost::lexical_cast<std::string>(i) + ".");
      std::string str((const char *)(data.data()));
      result.m_ext_label_names[i] = str;
    }
  }

  BoolVector touched;
  result.RegenerateLabelAssoc(touched);

  return resultptr;
}

void RVizCloudAnnotationPoints::Serialize(std::ostream & ofile) const
{
  if (!ofile)
    throw IOE("Invalid file stream.");

  const std::string magic_string = MAGIC_STRING;
  ofile.write(magic_string.c_str(),magic_string.size() + 1);
  const uint64 version = MAGIC_MAX_VERSION;
  ofile.write((char *)&version,sizeof(version));
  const uint64 cloud_size = m_cloud_size;
  ofile.write((char *)&cloud_size,sizeof(cloud_size));

  {
    const PointNeighborhood::Conf & conf = m_point_neighborhood->GetConf();
    const float position_importance = conf.position_importance;
    ofile.write((char *)&position_importance,sizeof(position_importance));
    const float color_importance = conf.color_importance;
    ofile.write((char *)&color_importance,sizeof(color_importance));
    const float normal_importance = conf.normal_importance;
    ofile.write((char *)&normal_importance,sizeof(normal_importance));
    const float max_distance = conf.max_distance;
    ofile.write((char *)&max_distance,sizeof(max_distance));
    conf.searcher->Serialize(ofile);

    const uint32 weight_steps = GetWeightStepsCount();
    ofile.write((char *)&weight_steps,sizeof(weight_steps));
  }

  const uint64 control_points_size = m_control_points_for_label.size();
  ofile.write((char *)&control_points_size,sizeof(control_points_size));

  for (uint64 i = 0; i < control_points_size; i++)
  {
    const uint64 control_point_size = m_control_points_for_label[i].size();
    ofile.write((char *)&control_point_size,sizeof(control_point_size));
    for (uint64 h = 0; h < control_point_size; h++)
    {
      const uint64 control_point_index = m_control_points_for_label[i][h];
      const ControlPoint & cp = m_control_points[control_point_index - 1];
      const uint64 point_index = cp.point_id;
      ofile.write((char *)&point_index,sizeof(point_index));
      const uint32 weight_step = cp.weight_step_id;
      ofile.write((char *)&weight_step,sizeof(weight_step));
    }
  }

  for (uint64 i = 0; i < control_points_size; i++)
  {
    uint32 string_size = m_ext_label_names[i].size();
    ofile.write((char *)&string_size,sizeof(string_size));
    ofile.write(m_ext_label_names[i].c_str(),string_size);
  }

  if (!ofile)
    throw IOE("Write error.");
}
