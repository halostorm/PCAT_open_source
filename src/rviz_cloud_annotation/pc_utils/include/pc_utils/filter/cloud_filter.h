//
// Created by ou on 2021/12/2.
//

#ifndef PC_UTILS_FILTER_H
#define PC_UTILS_FILTER_H

#include "pc_utils/common/common.h"

#define PC_UTILS_FILTER_TYPE                \
define( PassThroughFilter       )           \
define( CropAABoxFilter         )           \
define( CropOBoxFilter          )           \
define( RegionOfInterestFilter  )           \
define( ApproximateVoxelFilter  )           \
define( SelfFilter              )           \
define( MaxPointCountFilter     )           \
define( RandomSamplingFilter    )           \
define( RemoveNaNFilter         )           \
define( Filters                 )

namespace pc_utils {
template<class PointT>
class CloudFilter {
public:
    virtual void
    filter(const typename PC<PointT>::Ptr &input, typename PC<PointT>::Ptr &output, void *data = nullptr) = 0;

    virtual std::string class_name() = 0;
};  // namespace pc_utils


}


PC_UTILS_LINK_HELPER_HEADER(CloudFilter)

#endif //PC_UTILS_FILTER_H
