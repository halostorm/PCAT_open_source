//
// Created by ou on 2021/11/16.
//

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include "point_type.h"

/**
 * @brief: point cloud alias
 * @example: PCXYZ -> pcl::PointCloud<pcl::PointXYZ>
 */
#define PC_UTILS_DEF_CLOUD_TYPE(_, data, elem) \
using BOOST_PP_CAT(P,BOOST_PP_TUPLE_ELEM(1,elem))  = BOOST_PP_TUPLE_ELEM(0,elem); \
using BOOST_PP_CAT(PC,BOOST_PP_TUPLE_ELEM(1,elem)) = PC<BOOST_PP_TUPLE_ELEM(0,elem)>; \
using BOOST_PP_CAT(BOOST_PP_CAT(PC,BOOST_PP_TUPLE_ELEM(1,elem)),Ptr) = PC<BOOST_PP_TUPLE_ELEM(0,elem)>::Ptr;\
using BOOST_PP_CAT(BOOST_PP_CAT(PC,BOOST_PP_TUPLE_ELEM(1,elem)),ConstPtr) = PC<BOOST_PP_TUPLE_ELEM(0,elem)>::ConstPtr;

namespace pc_utils {
template<class PointT>
using PC = pcl::PointCloud<PointT>;
BOOST_PP_SEQ_FOR_EACH(PC_UTILS_DEF_CLOUD_TYPE, _, PC_UTILS_POINT_TYPE)
}  // namespace pc_utils


/***
 * @brief for template specialization
 * @example:
 * PC_UTILS_TEMPLATE_SPECIALIZATIONS(CurvedVoxelCluster)
 * -> template class CurvedVoxelCluster<PXYZ>;
 * -> template class CurvedVoxelCluster<PXYZI>;
 * -> ...
 */
#define PC_UTILS_TEMPLATE_SPECIALIZATIONS_ONE_TYPE(_, CLASS, ELEM)\
template class CLASS<BOOST_PP_TUPLE_ELEM(0,ELEM)>;

#define PC_UTILS_TEMPLATE_SPECIALIZATIONS(CLASS)\
BOOST_PP_SEQ_FOR_EACH(PC_UTILS_TEMPLATE_SPECIALIZATIONS_ONE_TYPE, CLASS, PC_UTILS_POINT_TYPE)


/***
 * @brief auto set params from yaml-node
 */
#define PC_UTILS_CLASS_MEMBER_0(_0, _1, _2) _0
#define PC_UTILS_CLASS_MEMBER_1(_0, _1, _2) _1
#define PC_UTILS_CLASS_MEMBER_2(_0, _1, _2) _2
#define PC_UTILS_CLASS_MEMBER_TYPE(pack) PC_UTILS_CLASS_MEMBER_0 pack
#define PC_UTILS_CLASS_MEMBER_NAME(pack) PC_UTILS_CLASS_MEMBER_1 pack
#define PC_UTILS_CLASS_MEMBER_VALUE(pack) PC_UTILS_CLASS_MEMBER_2 pack

/**
 @example   #undef PC_UTILS_CLASS_MEMBER_LIST
 @example   #define PC_UTILS_CLASS_MEMBER_LIST      \
 @example   ((float, min_dis   , ))                 \
 @example   ((float, min_points, ))
 @example   PC_UTILS_DEF_CLASS_MEMBER
 */
/* define class member variable from arg list*/
#define PC_UTILS_DEF_CLASS_MEMBER_IMPL(_, __, elem) \
PC_UTILS_CLASS_MEMBER_TYPE(elem)  PC_UTILS_CLASS_MEMBER_NAME(elem){PC_UTILS_CLASS_MEMBER_VALUE(elem)};

#define PC_UTILS_DEF_CLASS_MEMBER \
BOOST_PP_SEQ_FOR_EACH(PC_UTILS_DEF_CLASS_MEMBER_IMPL, _, PC_UTILS_CLASS_MEMBER_LIST)

/* set class member variable from yaml-node */
#define PC_UTILS_SET_CLASS_MEMBER_FROM_YAML_IMPL(_, data, elem) \
PC_UTILS_CLASS_MEMBER_NAME(elem) = params[BOOST_PP_STRINGIZE(PC_UTILS_CLASS_MEMBER_NAME(elem))].as<PC_UTILS_CLASS_MEMBER_TYPE(elem)>();

#define PC_UTILS_SET_CLASS_MEMBER_FROM_YAML \
BOOST_PP_SEQ_FOR_EACH(PC_UTILS_SET_CLASS_MEMBER_FROM_YAML_IMPL, _, PC_UTILS_CLASS_MEMBER_LIST)

#define PC_UTILS_CONSTRUCTOR_YAML_NODE(CLASS)     explicit CLASS(const YAML::Node &params) {PC_UTILS_SET_CLASS_MEMBER_FROM_YAML}

/***
 * @brief quickly auto register
 */
#define AUTO_REGISTER(base, der, ...) \
public AutoRegister<base,der>::template OverLoad<__VA_ARGS__>

/***
 *
 */
#define PC_UTILS_CLASS_DERIVATION_1(DER, _)                 DER
#define PC_UTILS_CLASS_DERIVATION(_)                        PC_UTILS_CLASS_DERIVATION_1 _
#define PC_UTILS_CLASS_BASE_1(_, BASE)                      BASE
#define PC_UTILS_CLASS_BASE(_)                              PC_UTILS_CLASS_BASE_1 _

#define PC_UTILS_CLASS_CONSTRUCTION_I(_, ctx, elem)         public AutoRegister<                                        \
                                                                PC_UTILS_CLASS_BASE_TYPE<PointT>,                       \
                                                                ctx<PointT>                                             \
                                                            >::template OverLoad<elem>,


#define PC_UTILS_BASE_LIST(DER)                             BOOST_PP_SEQ_FOR_EACH(                                      \
                                                            PC_UTILS_CLASS_CONSTRUCTION_I,                              \
                                                            DER,                                                        \
                                                            PC_UTILS_CLASS_CONSTRUCTION)                                \
                                                            public PC_UTILS_CLASS_BASE_TYPE<PointT>

/***
 * @brief for static link
 */
#define PC_UTILS_LINK_HELPER_HEADER(name)                           \
namespace pc_utils::_link {                                         \
extern bool link_##name;                                            \
static struct Link_##name##_Helper{                                 \
    Link_##name##_Helper(){if(link_##name) std::cout<<"";}          \
}helper##name;                                                      \
}

#define PC_UTILS_LINK_HELPER_CPP(name) \
namespace pc_utils::_link {bool link_##name = true;}



/**
 * class pre-declare
 */
namespace pc_utils {

class BoundingBox;

template<class PointT>
class BoundingExtractor;

template<class>
class CloudFilter;

template<class>
class Cluster;

template<class>
class GroundEstimator;

}// namespace pc_utils


#endif //SRC_UTILS_H
