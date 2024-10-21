//
// Created by nrsl on 2021/10/11.
//

#ifndef PERCEPTION3D_CLUSTER_H
#define PERCEPTION3D_CLUSTER_H

#include "pc_utils/common/common.h"
/**
 * @brief
 * @typedef EuclideanCluster
 * @typedef CurvedVoxelCluster
 */
#define PC_UTILS_CLUSTER_TYPE   \
define( EuclideanCluster    )   \
define( CurvedVoxelCluster  )


namespace pc_utils {

template<class PointT>
class Cluster {
public:
    virtual void extract(const typename PC<PointT>::Ptr &cloud_in,
                         std::vector<int> &cluster_indices,
                         std::vector<int> &cluster_id) = 0;
};


}   // namespace pc_utils






PC_UTILS_LINK_HELPER_HEADER(Cluster)
#endif //PERCEPTION3D_CLUSTER_H
