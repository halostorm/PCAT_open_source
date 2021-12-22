//
// Created by nrsl on 2021/10/11.
//

#ifndef PERCEPTION3D_BOX_EXTRACT_H
#define PERCEPTION3D_BOX_EXTRACT_H

#include "pc_utils/bound/OBB.h"
#include "pc_utils/common/common.h"


/**
 * @brief support type of box extractor
 * @typedef OrientedBBoxExtractor
 * @typedef OrientedBBox2p5DExtractor
 * @typedef AxiallyAlignedBBoxExtractor
 */
#define PC_UTILS_BBOX_TYPE                          \
define( AxiallyAlignedBBoxExtractor )               \
define( OrientedBBox3DExtractor     )               \
define( OrientedBBox2p5DExtractor   )

namespace pc_utils {

template<class PointT>
class BoundingExtractor {
public:
    virtual void extract(const typename PC<PointT>::Ptr &input, BoundingBox &output) = 0;
};

}  // namespace pc_utils











PC_UTILS_LINK_HELPER_HEADER(BoundingExtractor)
#endif //PERCEPTION3D_BOX_EXTRACT_H
