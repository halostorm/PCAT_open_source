//
// Created by nrsl on 2021/10/10.
//

#ifndef PC_UTILS_POINT_TYPE_H
#define PC_UTILS_POINT_TYPE_H

#include <boost/preprocessor/seq.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/***
 * @brief supported point type
 */
#define PC_UTILS_POINT_TYPE         \
((pcl::PointXYZ, XYZ))              \
((pcl::PointXYZI, XYZI))            \
((pcl::PointXYZL, XYZL))            \
((pcl::PointNormal, XYZN))          \
((pcl::PointXYZRGBNormal,XYZRGBN))



#endif //PC_UTILS_POINT_TYPE_H
