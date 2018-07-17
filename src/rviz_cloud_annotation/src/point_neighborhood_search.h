/*
 * Copyright (c) 2016-2017, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture
 *   University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef POINT_NEIGHBORHOOD_SEARCH_H
#define POINT_NEIGHBORHOOD_SEARCH_H

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

// STL
#include <stdint.h>
#include <vector>
#include <istream>
#include <ostream>

namespace PointNeighborhoodSearch
{
  typedef uint64_t uint64;
  typedef uint32_t uint32;

  typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
  typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
  typedef pcl::KdTreeFLANN<PointXYZRGBNormal> KdTree;

  typedef std::vector<int> IntVector;
  typedef std::vector<float> FloatVector;

  enum PostProcessingType
  {
    // solve non-symmetric graph edges by removing them
    PPTYPE_REMOVE_UNIDIRECTIONAL_LINKS,
    // make non-symmetric graph edges symmetric by adding the opposite edge
    PPTYPE_COMPLETE_UNIDIRECTIONAL_LINKS
  };

  class Searcher
  {
    public:
    typedef boost::shared_ptr<Searcher> Ptr;
    typedef boost::shared_ptr<const Searcher> ConstPtr;
    virtual ~Searcher() {}

    virtual void Search(const KdTree & kdtree,const PointXYZRGBNormal & center,
      IntVector & indices,FloatVector & distances) const = 0;
    virtual void Serialize(std::ostream & ofile) const = 0;
    virtual std::string ToString() const = 0;
    virtual uint64 GetId() const = 0;

    virtual bool ApproxEquals(const Searcher & other) const = 0;

    virtual bool IsPostProcessingRequired(const PostProcessingType type) const = 0;
  };

  struct ParserException
  {
    ParserException(const std::string m): message(m) {}
    std::string message;
  };

  // from ROS parameter
  Searcher::ConstPtr CreateFromString(const uint64 id,const std::string & param);
  // from binary stream
  Searcher::ConstPtr CreateFromIstream(std::istream & ifile);

  class FixedDistanceSearcher;
  class KNearestNeighborsSearcher;
  class KNearestNeighborsSearcherAtleast;
  class KNearestNeighborsSearcherAtmost;
}

#endif // POINT_NEIGHBORHOOD_SEARCH_H
