#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <iostream>
#include <vector>

#define _M_PI 3.1415926

#define DISTANCE_LIMMIT 20
#define HEIGHT_LIMMIT 2

#define _max(a, b) (((a) > (b)) ? (a) : (b))
#define _min(a, b) (((a) > (b)) ? (b) : (a))

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class PointCloudFilesTool
{
  typedef std::vector<std::string> StringVector;

public:
  struct ScanFileComp
  {
    bool operator()(const fs::path &p1, const fs::path &p2)
    {
      return p1.stem().string() < p2.stem().string();
    }
  };

  int getFilesList(const std::string &path_name, StringVector &m_pcd_files)
  {
    std::vector<fs::path> files_list;
    fs::path path(path_name);

    if (!fs::is_directory(path))
    {
      return 0;
    }

    files_list.clear();
    for (fs::directory_iterator file(path), f_end; file != f_end; ++file)
    {
      files_list.push_back(file->path());
    }

    sort(files_list.begin(), files_list.end(), ScanFileComp());

    for (std::vector<fs::path>::const_iterator iter = files_list.begin(); iter != files_list.end(); iter++)
    {
      //iter->filename().replace_extension("pcd");
      std::cerr << (iter->filename()).c_str() << std::endl;
      m_pcd_files.push_back((iter->filename()).c_str());
    }
    return 1;
  }
};
