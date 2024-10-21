//
// Created by ou on 2021/11/17.
//

#ifndef SRC_COMMON_H
#define SRC_COMMON_H

#include "utils.h"

namespace pc_utils {
double inline to_rad(double degree) { return degree / 180 * M_PI; }

Eigen::Isometry3f inline fromXYZRPY(const Eigen::Vector3f &xyz, const Eigen::Vector3f &rpy) {
    Eigen::Isometry3f ret = Eigen::Isometry3f::Identity();
    Eigen::AngleAxisf
            r(rpy.x(), Eigen::Vector3f::UnitX()),
            p(rpy.y(), Eigen::Vector3f::UnitY()),
            y(rpy.z(), Eigen::Vector3f::UnitZ());
    ret.translate(xyz);
    ret.rotate(Eigen::Quaternionf{y * p * r});
    return ret;
}

Eigen::Isometry3f inline fromXYZRPY(float x, float y, float z, float r, float p, float yaw) {
    return fromXYZRPY(Eigen::Vector3f(x, y, z), Eigen::Vector3f(r, p, yaw));
}
}




PC_UTILS_LINK_HELPER_HEADER(common)
#endif //SRC_COMMON_H
