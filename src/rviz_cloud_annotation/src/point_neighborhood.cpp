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

      #include "point_neighborhood.h"

      #include <Eigen/Dense>
      #include <Eigen/StdVector>

  PointNeighborhood::PointNeighborhood(PointXYZRGBNormalCloud::ConstPtr cloudptr,const Conf & conf)
  {
    const PointXYZRGBNormalCloud & cloud = *cloudptr;
    const uint64 cloud_size = cloud.size();

    m_conf = conf;

    m_index.resize(cloud_size);

    Uint64Vector temporary_indices_vector(cloud_size);

    KdTree::Ptr kdtree(new KdTree);

    kdtree->setInputCloud(cloudptr);

    uint64 counter = 0;

    std::vector<int> idxs;
    std::vector<float> dsts;
    for (uint64 i = 0; i < cloud_size; i++)
    {
      const PointXYZRGBNormal & pt = cloud[i];
      conf.searcher->Search(*kdtree,pt,idxs,dsts);
      const uint64 size = idxs.size();

      m_index[i].size = 0;

      if (size == 0)
        continue;
      m_neighbors.resize(counter + size);
      m_total_dists.resize(counter + size);
      m_position_dists.resize(counter + size);
      uint64 inc = 0;
      for (uint64 h = 0; h < size; h++)
            if (uint64(idxs[h]) != i) // do not add self
            {
              m_neighbors[counter + inc] = idxs[h];
              m_total_dists[counter + inc] = TotalDistance(cloud[i],cloud[idxs[h]],conf);
              m_position_dists[counter + inc] = std::sqrt(dsts[h]);
              inc++;
            }

            temporary_indices_vector[i] = counter;
            counter += inc;
            m_index[i].size = inc;
          }

        // if requested by the searcher, remove all non-symmetric links
          if (conf.searcher->IsPostProcessingRequired(PointNeighborhoodSearch::PPTYPE_REMOVE_UNIDIRECTIONAL_LINKS))
            RemoveUnidirectionalLinks(temporary_indices_vector);

          if (conf.searcher->IsPostProcessingRequired(PointNeighborhoodSearch::PPTYPE_COMPLETE_UNIDIRECTIONAL_LINKS))
            AddUnidirectionalLinks(temporary_indices_vector);

        // update pointers
          for (uint64 i = 0; i < cloud_size; i++)
          {
            m_index[i].neighbors = &(m_neighbors[temporary_indices_vector[i]]);
            m_index[i].total_dists = &(m_total_dists[temporary_indices_vector[i]]);
            m_index[i].position_dists = &(m_position_dists[temporary_indices_vector[i]]);
          }
        }

        void PointNeighborhood::RemoveUnidirectionalLinks(Uint64Vector & temporary_indices_vector)
        {
          const uint64 cloud_size = temporary_indices_vector.size();
          Uint64Vector temporary_indices_vector_in(cloud_size);
          NeighsVector index_in(cloud_size);
          Uint64Vector neighbors_in;
          FloatVector total_dists_in;
          FloatVector position_dists_in;
          temporary_indices_vector.swap(temporary_indices_vector_in);
          m_index.swap(index_in);
          m_neighbors.swap(neighbors_in);
          m_total_dists.swap(total_dists_in);
          m_position_dists.swap(position_dists_in);

          m_neighbors.reserve(neighbors_in.size());
          m_total_dists.reserve(total_dists_in.size());
          m_position_dists.reserve(position_dists_in.size());

          uint64 counter = 0;
          for (uint64 i = 0; i < cloud_size; i++)
          {
            const uint64 size = index_in[i].size;
            const uint64 start = temporary_indices_vector_in[i];

            uint64 inc = 0;

            for (uint64 h = 0; h < size; h++)
            {
              const uint64 i2 = neighbors_in[start + h];
              const float td = total_dists_in[start + h];
              const float pd = position_dists_in[start + h];

              const uint64 size2 = index_in[i2].size;
              const uint64 start2 = temporary_indices_vector_in[i2];

              for (uint64 k = 0; k < size2; k++)
                if (neighbors_in[start2 + k] == i)
                {
                  m_neighbors.push_back(i2);
                  m_total_dists.push_back(td);
                  m_position_dists.push_back(pd);
                  inc++;
                  break;
                }
              }

              temporary_indices_vector[i] = counter;
              counter += inc;
              m_index[i].size = inc;
            }
          }

          void PointNeighborhood::AddUnidirectionalLinks(Uint64Vector & temporary_indices_vector)
          {
            const uint64 cloud_size = temporary_indices_vector.size();
            Uint64Vector temporary_indices_vector_in(cloud_size);
            NeighsVector index_in(cloud_size);
            Uint64Vector neighbors_in;
            FloatVector total_dists_in;
            FloatVector position_dists_in;
            temporary_indices_vector.swap(temporary_indices_vector_in);
            m_index.swap(index_in);
            m_neighbors.swap(neighbors_in);
            m_total_dists.swap(total_dists_in);
            m_position_dists.swap(position_dists_in);

            Uint64VectorVector dv_neighbors(cloud_size);
            FloatVectorVector dv_total_dists(cloud_size);
            FloatVectorVector dv_position_dists(cloud_size);

            uint64 counter = 0;
            for (uint64 i = 0; i < cloud_size; i++)
            {
              const uint64 size = index_in[i].size;
              const uint64 start = temporary_indices_vector_in[i];

              Uint64Vector & v_neighbors = dv_neighbors[i];
              FloatVector & v_total_dists = dv_total_dists[i];
              FloatVector & v_position_dists = dv_position_dists[i];

              for (uint64 h = 0; h < size; h++)
              {
                const uint64 i2 = neighbors_in[start + h];
                const float td = total_dists_in[start + h];
                const float pd = position_dists_in[start + h];

                if (std::find(v_neighbors.begin(),v_neighbors.end(),i2) == v_neighbors.end())
                {
                  v_neighbors.push_back(i2);
                  v_total_dists.push_back(td);
                  v_position_dists.push_back(pd);
                  counter++;
                }

                Uint64Vector & v2_neighbors = dv_neighbors[i2];
                FloatVector & v2_total_dists = dv_total_dists[i2];
                FloatVector & v2_position_dists = dv_position_dists[i2];

                if (std::find(v2_neighbors.begin(),v2_neighbors.end(),i) == v2_neighbors.end())
                {
                  v2_neighbors.push_back(i);
                  v2_total_dists.push_back(td);
                  v2_position_dists.push_back(pd);
                  counter++;
                }
              }
            }

            m_neighbors.resize(counter);
            m_total_dists.resize(counter);
            m_position_dists.resize(counter);

            counter = 0;
            for (uint64 i = 0; i < cloud_size; i++)
            {
              const uint64 size = dv_neighbors[i].size();
              m_index[i].size = size;
              const uint64 start = counter;
              temporary_indices_vector[i] = counter;
              for (uint64 h = 0; h < size; h++)
              {
                m_neighbors[start + h] = dv_neighbors[i][h];
                m_total_dists[start + h] = dv_total_dists[i][h];
                m_position_dists[start + h] = dv_position_dists[i][h];
              }
              counter += size;
            }
          }

          float PointNeighborhood::TotalDistance(const PointXYZRGBNormal & a,const PointXYZRGBNormal & b,const Conf & conf) const
          {
            const Eigen::Vector3f epta(a.x,a.y,a.z);
            const Eigen::Vector3f eptb(b.x,b.y,b.z);
            const Eigen::Vector3f eca(a.r,a.g,a.b);
            const Eigen::Vector3f ecb(b.r,b.g,b.b);
            const Eigen::Vector3f ena(a.normal_x,a.normal_y,a.normal_z);
            const Eigen::Vector3f enb(b.normal_x,b.normal_y,b.normal_z);

            float spatial_dist = (epta - eptb).norm() / conf.max_distance;
            float color_dist = (eca - ecb).norm() / (255.0f * std::sqrt(3.0f));
            float cos_angle_normal = 1.0f - ena.dot(enb);
            return spatial_dist * conf.position_importance + cos_angle_normal * conf.normal_importance + color_dist * conf.color_importance;
          }

