//
// Created by ou on 2021/12/7.
//

#ifndef PC_UTILS_BOUNDING_OBB_H
#define PC_UTILS_BOUNDING_OBB_H
#include <Eigen/Geometry>
namespace pc_utils{

class BoundingBox {
public:
    Eigen::Isometry3f pose{Eigen::Isometry3f::Identity()};
    Eigen::Vector3f dxyz{1, 1, 1};

    static void Corner3d(const BoundingBox &_box, Eigen::Vector3f (&_corner3d)[8]) {
        /**
         **  Box Corner  下/上
         **         2/3 __________ 6/7
         **            |    y     |
         **            |    |     |
         **            |    o - x |
         **            |          |
         **            |__________|
         **         0/1            4/5
         **********************************************/
        static Eigen::Array3f corner3d_unit[8] = {{-1, -1, -1},
                                                  {-1, -1, 1},
                                                  {-1, 1,  -1},
                                                  {-1, 1,  1},
                                                  {1,  -1, -1},
                                                  {1,  -1, 1},
                                                  {1,  1,  -1},
                                                  {1,  1,  1}};

        for (int i = 0; i < 8; i++) {
            _corner3d[i] = _box.pose * (0.5f * corner3d_unit[i] * _box.dxyz.array());
        }
    }


    static void LineList(const BoundingBox &_box, Eigen::Vector3f (&_lines)[24]) {
        static int table[] = {0, 1, 1, 3, 3, 2, 2, 0, 4, 5, 5, 7, 7, 6, 6, 4, 0, 4, 1, 5, 2, 6, 3, 7};
        Eigen::Vector3f corners[8];
        Corner3d(_box, corners);
        for (int i = 0; i < 24; i++) {
            _lines[i][0] = corners[table[i]][0];
            _lines[i][1] = corners[table[i]][1];
            _lines[i][2] = corners[table[i]][2];
        }
    }
};
}

#endif //PC_UTILS_BOUNDING_OBB_H
