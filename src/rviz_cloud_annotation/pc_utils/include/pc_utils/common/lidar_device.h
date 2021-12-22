//
// Created by nrsl on 2021/10/11.
//

#ifndef PERCEPTION3D_LIDAR_H
#define PERCEPTION3D_LIDAR_H

#include "pc_utils/common/common.h"

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem/operations.hpp>

namespace pc_utils {
struct LidarDevice {
    int index;
    std::string name;
    std::string type;
    std::string frame_id;
    std::string ros_topic;
    std::string relative_to;
    Eigen::Isometry3f pose;
    bool enable;
    bool fuse_relative;
    bool fuse_global;

    static void from_yaml(const YAML::Node &config, std::vector<LidarDevice> &devices) {
        /***
        * 读取雷达配置
        */
        for (int i = 0; i < config.size(); i++) {
            auto device = config[i];
            auto lidar = device["device"];
            auto init = lidar["calibration"]["init_pose"].as<std::vector<float>>();

            devices.push_back({
                                      .index=i,
                                      .name=lidar["name"].as<std::string>(),
                                      .type=lidar["type"].as<std::string>(),
                                      .frame_id=lidar["frame_id"].as<std::string>(),
                                      .ros_topic=lidar["ros_topic"].as<std::string>(),
                                      .relative_to=lidar["calibration"]["relative_to"].as<std::string>(),
                                      .pose=fromXYZRPY(init[0], init[1], init[2], init[3], init[4], init[5]),
                                      .enable=lidar["calibration"]["enable"].as<bool>(),
                                      .fuse_relative=lidar["calibration"]["fuse_relative"].as<bool>(),
                                      .fuse_global=lidar["fuse_global"].as<bool>()
                              });
        }
    }

    static void from_yaml(const std::string &config_file, std::vector<LidarDevice> &devices) {
        if (not boost::filesystem::exists(config_file)) {
            throw std::runtime_error(config_file + std::string("no exist"));
        } else {
            auto config = YAML::LoadFile(config_file)["lidar"];
            from_yaml(config, devices);
        }
    }
};

}

#endif //PERCEPTION3D_LIDAR_H
