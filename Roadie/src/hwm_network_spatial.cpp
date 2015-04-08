/*
 * hwm_network_spatial.cpp
 * Road Network Library
 *
 * Copyright (c) 2010-2016 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * David Wilkie, Jason Sewall, Weizi Li, Ming C. Lin
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RoadLib/>
 */

#include "hwm_network.hpp"

namespace hwm
{
    void network_aux::build_spatial()
    {
        road_space.build(net.lane_width, rrm);
    }

    network_aux::road_spatial::entry::entry() : lc(0)
    {
    }

    network_aux::road_spatial::entry::entry(network_aux::road_rev_map::lane_cont *in_lc, const aabb2d &in_rect) : lc(in_lc), rect(in_rect)
    {
    }

    network_aux::road_spatial::road_spatial() : tree(0)
    {
    }

    network_aux::road_spatial::~road_spatial()
    {
        if(tree)
            delete tree;
    }

    void network_aux::road_spatial::build(float lane_width, strhash<network_aux::road_rev_map>::type &roads)
    {
        if(tree)
            delete tree;

        std::vector<rtree2d::entry> leaves;
        typedef strhash<road_rev_map>::type::value_type rrm_pair;
        for(rrm_pair &rp: roads)
        {
            for(partition01<road_rev_map::lane_cont>::iterator current = rp.second.lane_map.begin(); current != rp.second.lane_map.end(); ++current)
            {
                const vec2f interval(rp.second.lane_map.containing_interval(current));
                const aabb2d rect(current->second.planar_bounding_box(lane_width, interval));
                leaves.push_back(rtree2d::entry(rect, items.size()));
                items.push_back(entry(&(current->second), rect));
            }
        }

        tree = rtree2d::hilbert_rtree(leaves);
    }

    std::vector<network_aux::road_spatial::entry> network_aux::road_spatial::query(const aabb2d &rect) const
    {
        std::vector<entry> res;
        if(tree)
        {
            const std::vector<size_t> q_res(tree->query(rect));
            for(const size_t &i: q_res)
            {
                res.push_back(items[i]);
            }
        }

        return res;
    }
}
