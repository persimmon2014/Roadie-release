/*
 * hwm_network.cpp
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
    float maximum_cornering_speed(const float radius, const float g, const float static_friction)
    {
        return std::sqrt(radius*g*static_friction);
    }

    network::serial_state::serial_state()
    {
    }

    network::serial_state::serial_state(const network &n)
    {
        lane_states.reserve(n.lanes.size());
        for(const lane_pair &lp: n.lanes)
        {
            lane_states.push_back(lp.second.serial());
        }
        intersection_states.reserve(n.intersections.size());
        for(const intersection_pair &ip: n.intersections)
        {
            intersection_states.push_back(ip.second.serial());
        }
    }

    void network::serial_state::apply(network &n) const
    {
        size_t i = 0;
        for(lane_pair &lp: n.lanes)
        {
            assert(i < lane_states.size());
            lane_states[i].apply(lp.second);
            ++i;
        }
        i = 0;
        for(intersection_pair &ip: n.intersections)
        {
            assert(i < intersection_states.size());
            intersection_states[i].apply(ip.second);
            ++i;
        }
    }

    network::network(const network &n)
    {
        copy(n);
    }
    


    void network::copy(const network &n)
    {
        name       = n.name;
        gamma      = n.gamma;
        lane_width = n.lane_width;

        roads         = n.roads;
        lanes         = n.lanes;
        intersections = n.intersections;

        // Fill in pointers in lanes with other lanes
        for(lane_pair &l: lanes)
        {
            const str &lane_id      = l.first;
            lane      &current_lane = l.second;

            // Find lane in 'n' that corresponds to this lane
            const lane_map::const_iterator other = n.lanes.find(lane_id);
            assert(other != n.lanes.end());
            assert(other->first == lane_id);

            const lane &other_lane = other->second;
            {
                lane::road_membership::intervals::iterator       my_memb    = current_lane.road_memberships.begin();
                lane::road_membership::intervals::const_iterator other_memb =   other_lane.road_memberships.begin();
                for(; my_memb  != current_lane.road_memberships.end() &&
                    other_memb !=   other_lane.road_memberships.end();
                    ++my_memb, ++other_memb)
                {
                    assert(my_memb->second.parent_road);
                    const road_map::iterator my_road = roads.find(other_memb->second.parent_road->id);
                    assert(my_road != roads.end());
                    my_memb->second.parent_road = &(my_road->second);
                }
            }
            {
                lane::adjacency::intervals::iterator       my_adj    = current_lane.left.begin();
                lane::adjacency::intervals::const_iterator other_adj =   other_lane.left.begin();
                for(; my_adj  != current_lane.left.end() &&
                    other_adj !=   other_lane.left.end();
                    ++my_adj, ++other_adj)
                {
                    if(my_adj->second.neighbor)
                    {
                        const lane_map::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
                        assert(my_lane != lanes.end());
                        my_adj->second.neighbor = &(my_lane->second);
                    }
                }
            }
            {
                lane::adjacency::intervals::iterator       my_adj    = current_lane.right.begin();
                lane::adjacency::intervals::const_iterator other_adj =   other_lane.right.begin();
                for(; my_adj  != current_lane.right.end() &&
                    other_adj !=   other_lane.right.end();
                    ++my_adj, ++other_adj)
                {
                    if(my_adj->second.neighbor)
                    {
                        const lane_map::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
                        assert(my_lane != lanes.end());
                        my_adj->second.neighbor = &(my_lane->second);
                    }
                }
            }

            current_lane.start->update_pointers(*this);
            current_lane.end->update_pointers(*this);
        }
        // typedef strhash<intersection>::type::value_type ival;
        // for(ival &i: intersections)
        // {
        //     const str    &intersection_id      = i.first;
        //     intersection &current_intersection = i.second;

        //     const strhash<intersection>::type::const_iterator other = n.intersections.find(i.first);
        //     assert(other != n.intersections.end());
        //     assert(other->first == intersection_id);

        //     const intersection &other_intersection = other->second;

        //     {
        //         std::vector<lane*>::iterator       my_inc     = current_intersection.incoming.begin();
        //         std::vector<lane*>::const_iterator other_inc  =   other_intersection.incoming.begin();
        //         for(; my_inc  != current_intersection.incoming.end() &&
        //             other_inc !=   other_intersection.incoming.end();
        //             ++my_inc, ++other_inc)
        //         {
        //             assert(*my_inc);
        //             const strhash<lane>::type::iterator my_lane = lanes.find((*other_inc)->id);
        //             assert(my_lane != lanes.end());
        //             *my_inc = &(my_lane->second);
        //         }
        //     }

        //     {
        //         std::vector<lane*>::iterator       my_out     = current_intersection.outgoing.begin();
        //         std::vector<lane*>::const_iterator other_out  =   other_intersection.outgoing.begin();
        //         for(; my_out  != current_intersection.outgoing.end() &&
        //             other_out !=   other_intersection.outgoing.end();
        //             ++my_out, ++other_out)
        //         {
        //             assert(*my_out);
        //             const strhash<lane>::type::iterator my_lane = lanes.find((*other_out)->id);
        //             assert(my_lane != lanes.end());
        //             *my_out = &(my_lane->second);
        //         }

        //         std::vector<intersection::state>::iterator       my_state    = current_intersection.states.begin();
        //         std::vector<intersection::state>::const_iterator other_state =   other_intersection.states.begin();
        //         for(;  my_state != current_intersection.states.end() &&
        //             other_state !=   other_intersection.states.end();
        //             ++my_state, ++other_state)
        //         {
        //             *my_state = *other_state;
        //             std::vector<intersection::state::out_id>::iterator       my_out    =    my_state->in_states.begin();
        //             std::vector<intersection::state::out_id>::const_iterator other_out = other_state->in_states.begin();
        //             for(;  my_out !=    my_state->in_states.end() &&
        //                 other_out != other_state->in_states.end();
        //                 ++my_out, ++other_out)
        //             {
        //                 my_out->fict_lane = 0;
        //             }
        //             std::vector<intersection::state::in_id>::iterator       my_in    =    my_state->out_states.begin();
        //             std::vector<intersection::state::in_id>::const_iterator other_in = other_state->out_states.begin();
        //             for(;  my_in !=    my_state->out_states.end() &&
        //                 other_in != other_state->out_states.end();
        //                 ++my_in, ++other_in)
        //             {
        //                 my_in->fict_lane = 0;
        //             }
        //         }
        //}
        //}
    }

    network &network::operator=(const network &n)
    {
        this->network::~network();
        copy(n);
        return *this;
    }

    void network::check() const
    {
       
        if(gamma <= 0.0f || gamma >= 1.0f)
            throw std::runtime_error("Gamma in network out of range");

        if(lane_width <= 0.0f)
            throw std::runtime_error("lane_width in network out of range");

        for(const road_pair &r: roads)
        {
            if(r.first != r.second.id)
                throw std::runtime_error("Container id and local id mismatch for road");
            r.second.check();
        }

        for(const lane_pair &l: lanes)
        {
            if(l.first != l.second.id)
                throw std::runtime_error("Container id and local id mismatch for lane");
            l.second.check();
        }

        for(const intersection_pair &i: intersections)
        {
            if(i.first != i.second.id)
                throw std::runtime_error("Container id and local id mismatch for road");
               i.second.check();
        }
    }

    void network::translate(const vec3f &o)
    {
        for(road_pair &rp: roads)
        {
            rp.second.translate(o);
        }

        for(intersection_pair &ip: intersections)
        {
            ip.second.translate(o);
        }
    }

    void network::build_intersections()
    {
        for(intersection_pair &ip: intersections)
        {
            ip.second.build_shape(lane_width);
        }
    }

    void network::build_fictitious_lanes()
    {
        for(intersection_pair &ip: intersections)
        {
            ip.second.build_fictitious_lanes();
        }
    }

    void network::auto_scale_memberships()
    {
        for(lane_pair &lp: lanes)
        {
            lp.second.auto_scale_memberships();
        }
    }

    network::serial_state network::serial() const
    {
        return serial_state(*this);
    }

    void network::center(const bool z)
    {
        vec3f low(FLT_MAX);
        vec3f high(-FLT_MAX);
        bounding_box(low, high);
        vec3f center((low+high)/2);

        std::cout << center[0] << " " << center[1] << std::endl;
        if(!z)
            center[2] = 0;
        translate(vec3f(-center));
    }

    void network::bounding_box(vec3f &low, vec3f &high) const
    {
        for(const road_pair &rv: roads)
        {
            rv.second.bounding_box(low, high);
        }
    }

    template <class T>
    static inline T &retrieve(typename strhash<T>::type &m, const str &id)
    {
        assert(id != str());
        typename strhash<T>::type::iterator entry(m.find(id));
        if(entry == m.end())
        {
            entry = m.insert(entry, std::make_pair(id, T()));
            entry->second.id = id;
        }

        return entry->second;
    }

    network from_sumo(const str &name, const float gamma, const float lane_width, const sumo::network &snet)
    {
      
        network hnet;
//         typedef strhash<sumo::node>::type::value_type      sumo_node_pair;
//         typedef strhash<sumo::edge_type>::type::value_type sumo_type_pair;
//         typedef strhash<sumo::edge>::type::value_type      sumo_edge_pair;
// 
//         
//         hnet.name       = name;
//         hnet.gamma      = gamma;
//         hnet.lane_width = lane_width;
// 
//         strhash<size_t>::type node_degree;
//         for(const sumo_node_pair &np: snet.nodes)
//         {
//             node_degree.insert(std::make_pair(np.first, 0));
//         }
// 
//         for(const sumo_edge_pair &ep: snet.edges)
//         {
//             const sumo::edge &e = ep.second;
// 
//             ++node_degree[e.from->id];
//             ++node_degree[e.to->id];
// 
//             road &new_road = retrieve<road>(hnet.roads, e.id);
//             new_road.name = new_road.id;
// 
//             new_road.rep.points_.reserve(2 + e.shape.size());
//             new_road.rep.points_.push_back(vec3f(e.from->xy[0],
//                                                  e.from->xy[1],
//                                                  0.0f));
//             for(const vec2d &v: e.shape)
//             {
//                 new_road.rep.points_.push_back(vec3f(v[0],
//                                                      v[1],
//                                                      0.0f));
//             }
//             new_road.rep.points_.push_back(vec3f(e.to->xy[0],
//                                                  e.to->xy[1],
//                                                  0.0f));
//             if(!new_road.rep.initialize_from_polyline(lane_width, new_road.rep.points_))
//                 throw std::runtime_error("Failed to initialize arc_road in from_sumo");
//         }
// 
//         for(const strhash<size_t>::type::value_type &ndeg: node_degree)
//         {
//             if(ndeg.second > 1)
//                 retrieve<intersection>(hnet.intersections, ndeg.first);
//         }
// 
//         for(const sumo_edge_pair &ep: snet.edges)
//         {
//             const sumo::edge      &e           = ep.second;
//             const sumo::edge_type &et          = *(e.type);
//             road                  *parent_road = &retrieve<road>(hnet.roads, e.id);
//             std::vector<lane*>     newlanes(et.nolanes);
// 
//             intersection *start_inters, *end_inters;
//             {
//                 intersection_map::iterator the_inters = hnet.intersections.find(e.from->id);
//                 start_inters                          = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
//                 the_inters                            = hnet.intersections.find(e.to->id);
//                 end_inters                            = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
//             }
// 
//             for(int lanect = 0; lanect < et.nolanes; ++lanect)
//             {
// 		//lane &new_lane = retrieve<lane>(hnet.lanes, std::string(e.id + "_" +std::to_string(lanect));
//                 newlanes[lanect] = &new_lane;
// 
//                 new_lane.speedlimit = et.speed;
//                 lane::road_membership rm;
//                 rm.parent_road = parent_road;
//                 rm.interval[0] = 0.0f;
//                 rm.interval[1] = 1.0f;
//                 rm.lane_position = lanect;
//                 new_lane.road_memberships.insert(0.0, rm);
// 
//                 // new_lane.start.inters = start_inters;
//                 // if(start_inters)
//                 // {
//                 //     start_inters->outgoing.push_back(&new_lane);
//                 //     new_lane.start.intersect_in_ref = start_inters->outgoing.size()-1;
//                 // }
//                 // else
//                 //     new_lane.start.intersect_in_ref = -1;
// 
//                 // new_lane.end.inters = end_inters;
//                 // if(end_inters)
//                 // {
//                 //     end_inters->incoming.push_back(&new_lane);
//                 //     new_lane.end.intersect_in_ref = end_inters->incoming.size()-1;
//                 // }
//                 // else
//                 //     new_lane.end.intersect_in_ref = -1;
//             }
// 
//             for(int lanect = 0; lanect < et.nolanes; ++lanect)
//             {
//                 if(lanect > 0)
//                 {
//                     lane::adjacency la;
//                     la.neighbor = newlanes[lanect-1];
//                     la.neighbor_interval[0] = 0.0f;
//                     la.neighbor_interval[1] = 1.0f;
//                     newlanes[lanect]->left.insert(0.0f, la);
//                 }
//                 if(lanect < et.nolanes - 1)
//                 {
//                     lane::adjacency la;
//                     la.neighbor = newlanes[lanect+1];
//                     la.neighbor_interval[0] = 0.0f;
//                     la.neighbor_interval[1] = 1.0f;
//                     newlanes[lanect]->right.insert(0.0f, la);
//                 }
//             }
//         }

        return hnet;
    }

    network from_osm(const str &name, const float gamma, osm::network &onet)
    {
        float lane_width = onet.lane_width;
	float cull_prox =  onet.node_culling_threshold;
	
        typedef strhash<osm::node>::type::value_type      node_pair;
        typedef strhash<osm::edge_type>::type::value_type type_pair;
        typedef strhash<osm::edge>::type::value_type      edge_pair;
        network                                           hnet;
        hnet.name	  = name;
        hnet.gamma	  = gamma;
        hnet.lane_width	  = lane_width;
	hnet.center_point = onet.center;

	
	//// store all nodes id
        strhash<size_t>::type	node_degree;
        for(const node_pair &np: onet.nodes)
        {
            node_degree.insert(std::make_pair(np.first, 0)); 
        }

        //// for each edge creates an arc road
        for(const osm::edge& e: onet.edges)
        {     
	    // for each edge, make the first and last node's degree 1
            ++node_degree[e.shape[0]->id];
            ++node_degree[e.shape.back()->id];

	    // create a new road for each edge in osm network
            road &new_road = retrieve<road>(hnet.roads, e.id);
            new_road.name = new_road.id;
	    
	    // rep is the arc road
            new_road.rep.points_.reserve(2 + e.shape.size()); 
            const osm::node* last;
	    	    
            for(const osm::node *n: e.shape)
            {
                if (n->id != e.shape[0]->id)
                {
                    if(planar_distance(n->xy, last->xy) > 1e-7)
                        new_road.rep.points_.push_back(n->xy);
                }
                else
                    new_road.rep.points_.push_back(n->xy);

                last = n;
            }
	    
            //std::cout << "working on " << e.id << std::endl;
            if(!new_road.rep.initialize_from_polyline(cull_prox, new_road.rep.points_))
                throw std::runtime_error("Failed to initialize arc_road in from_osm");
        }
        
        
        //// create new intersections
        for(const strhash<osm::intersection>::type::value_type &isect: onet.intersections)
        {   	    
            retrieve<intersection>(hnet.intersections, isect.first); 
        }

        typedef std::pair<std::vector<lane*>, std::vector<lane*> > forward_and_reverse;
        std::map<str, forward_and_reverse> roads_to_lanes;

	// creating lanes for each edge (road)
        for(const osm::edge& e: onet.edges)
        {
            const osm::edge_type  &et          = *(e.type);
            road                  *parent_road = &retrieve<road>(hnet.roads, e.id);
            //std::cout << "working on " << e.id << std::endl;
            std::vector<lane*>     newlanes(et.nolanes); // for highway is 5 lanes
            std::vector<lane*>     new_reverse_lanes(et.nolanes);

	    
	    // if current edge starts or ends at some intersections, store the intersection information 
            intersection *start_inters, *end_inters;
            {
                intersection_map::iterator the_inters = hnet.intersections.find(e.from);
                start_inters                          = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
                the_inters                            = hnet.intersections.find(e.to);
                end_inters                            = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
            }
            
//          if(start_inters)
//          std::cout << "start_inters id=" << start_inters->id << std::endl;
//             
// 	    if(end_inters)
// 	    std::cout << "end_inters id=" << end_inters->id << std::endl;
	    
	    
            for(int lanect = 0; lanect < et.nolanes; ++lanect)
            {
	        // for any road isn't a one way, create a forward and a reverse lane for each lanect, so the total number of lanes would be 2 * et.nolanes
	        // forward lanes have negative offset (e.g. -1.75, -5.25)
	        // reverse lanes have positive offset (e.g. 1.75, 5.25)
                if (et.oneway == 0)
                {
		    // lane id would be the edge name + "_" + lanect
                    lane &new_lane = retrieve<lane>(hnet.lanes, std::string(e.id + "_" + std::to_string(lanect)));		  
		    newlanes[lanect] = &new_lane;

                    //Store road to lane pointers
                    roads_to_lanes[e.id].first.push_back(&new_lane); // store forward lanes

                    new_lane.speedlimit = et.speed;
                    
		    // road membership dictates which portion of the parent road belongs to the current lane; 
		    // from 0 to 1 meaning the parent road fully belongs to the current lane
		    lane::road_membership rm;
                    rm.parent_road = parent_road;
                    rm.interval[0] = 0.0f;
                    rm.interval[1] = 1.0f;
		    
		    //Soooo forward lanes have a negative offset.
                    rm.lane_position = lane_width*(-0.5 + -1*(lanect));
		    //std::cout << "lane id=" << new_lane.id <<" position="<<rm.lane_position<<std::endl;

                    new_lane.road_memberships.insert(0.0, rm);

                    // Assign lane starting and ending types, if one has not been assigned.
                    if (!new_lane.start) // start is a terminus*, if the terminus has not been assigned
                    {
                        if(start_inters) // current lane starts with an intersection terminus
                        {
                            start_inters->outgoing.push_back(&new_lane);
                            new_lane.start = new lane::intersection_terminus(start_inters, start_inters->outgoing.size()-1); // the second parameter is a ref number
                        }
                        else // current lane starts with a lane terminus
                            new_lane.start = new lane::terminus();
                    }
                    if (!new_lane.end)
                    {
                        if(end_inters)
                        {
                            end_inters->incoming.push_back(&new_lane);
                            new_lane.end = new lane::intersection_terminus(end_inters, end_inters->incoming.size()-1);
                        }
                        else
                            new_lane.end = new lane::terminus();
                    }

                    
                    // Add reverse lanes  (TODO one way roads)
                    lane &new_reverse_lane = retrieve<lane>(hnet.lanes, std::string(e.id + "_" + std::to_string(lanect) + "_reverse"));
                    
                    new_reverse_lanes[lanect] = &new_reverse_lane;

                    roads_to_lanes[e.id].second.push_back(&new_reverse_lane); // store reverse lanes

                    new_reverse_lane.speedlimit = et.speed;
                    lane::road_membership rm_rev;
                    rm_rev.parent_road = parent_road;
                    rm_rev.interval[0] = 1.0f;
                    rm_rev.interval[1] = 0.0f;
		    //Reverse lanes have positive offset.
                    rm_rev.lane_position = lane_width*(0.5 + (lanect));;
                    //std::cout << "lane id=" << new_reverse_lane.id <<" position="<<rm_rev.lane_position<<std::endl;
                    new_reverse_lane.road_memberships.insert(0.0, rm_rev);
                    
		    // Create intersections only if one has not been assigned.
                    if (!new_reverse_lane.start)
                    {
                        if(end_inters)
                        {
                            end_inters->outgoing.push_back(&new_reverse_lane);
                            new_reverse_lane.start = new lane::intersection_terminus(end_inters, end_inters->outgoing.size()-1);
                        }
                        else
                            new_reverse_lane.start = new lane::terminus();
                    }

                    if (!new_reverse_lane.end)
                    {
                        if(start_inters)
                        {
                            start_inters->incoming.push_back(&new_reverse_lane);
                            new_reverse_lane.end = new lane::intersection_terminus(start_inters, start_inters->incoming.size()-1);
                        }
                        else
                            new_reverse_lane.end = new lane::terminus();
                    }
                }
                else //Oneway road
                {
		    lane &new_lane = retrieve<lane>(hnet.lanes, std::string(e.id + "_" + std::to_string(lanect)));		    	    
                    newlanes[lanect] = &new_lane;
                    roads_to_lanes[e.id].first.push_back(&new_lane); // store forward lanes

                    new_lane.speedlimit = et.speed;
                    lane::road_membership rm;
                    rm.parent_road = parent_road;
                    rm.interval[0] = 0.0f;
                    rm.interval[1] = 1.0f;

                    rm.lane_position = lane_width*(lanect + -((et.nolanes - 1)/2.0));
		    //std::cout << "lane id=" << new_lane.id <<" position="<<rm.lane_position<<std::endl;
                    
                    new_lane.road_memberships.insert(0.0, rm);

                    if (!new_lane.start)
                    {
                        if(start_inters)
                        {
                            start_inters->outgoing.push_back(&new_lane);
                            new_lane.start = new lane::intersection_terminus(start_inters, start_inters->outgoing.size()-1);
                        }
                        else
                            new_lane.start = new lane::terminus();
                    }

                    if (!new_lane.end)
                    {
                        if(end_inters)
                        {
                            end_inters->incoming.push_back(&new_lane);
                            new_lane.end = new lane::intersection_terminus(end_inters, end_inters->incoming.size()-1);
                        }
                        else
                            new_lane.end = new lane::terminus();
                    }
                }
            }

            
            
//             if(e.id == "16700848") {
// 	      for(int i=0; i<newlanes.size(); i++) {
// 		
// 		std::cout << "lane id=" << newlanes[i]->id<<std::endl;
// 	      }
// 	    }
            
            // establish neighbor lanes
            for(int lanect = 0; lanect < et.nolanes; ++lanect)
            {
                if(lanect > 0)
                {
                    lane::adjacency la;
                    la.neighbor = newlanes[lanect-1];
                    la.neighbor_interval[0] = 0.0f;
                    la.neighbor_interval[1] = 1.0f;
                    newlanes[lanect]->right.insert(0.0f, la); 
                }
                
                if(lanect < et.nolanes - 1)
                {
                    lane::adjacency la;
                    la.neighbor = newlanes[lanect+1];
                    la.neighbor_interval[0] = 0.0f;
                    la.neighbor_interval[1] = 1.0f;
                    newlanes[lanect]->left.insert(0.0f, la);
                }

                if (et.oneway == 0) // not a one way, repeat for reverse lanes
                {
                    if(lanect > 0)
                    {
                        lane::adjacency la;
                        la.neighbor = new_reverse_lanes[lanect-1];
                        la.neighbor_interval[0] = 0.0f;
                        la.neighbor_interval[1] = 1.0f;
                        new_reverse_lanes[lanect]->right.insert(0.0f, la);
                    }
                    
                    if(lanect < et.nolanes - 1)
                    {
                        lane::adjacency la;
                        la.neighbor = new_reverse_lanes[lanect+1];
                        la.neighbor_interval[0] = 0.0f;
                        la.neighbor_interval[1] = 1.0f;
                        new_reverse_lanes[lanect]->left.insert(0.0f, la);
                    }
                }
            }
        }

        
        // creating ramps
        for(const osm::edge& e: onet.edges)
        {
            const osm::edge_type &et          = *(e.type);

            typedef strhash<hwm::lane>::type::value_type hwm_l_pair;
            for(const hwm_l_pair& l: hnet.lanes)
            {
                l.second.check();
                assert(l.second.length() > 0);
            }

            for(const osm::edge::lane& l: e.additional_lanes)
            {
                str id = std::string(e.id + "_" + std::to_string(l.start_t) + "_" + std::to_string(l.end_t) + "_" + std::to_string(l.offset));
	      
	        road                  *parent_road = &retrieve<road>(hnet.roads, e.id);
                lane &new_lane = retrieve<lane>(hnet.lanes, id);
                //Store road to lane pointers.
                roads_to_lanes[e.id].first.push_back(&new_lane);

                new_lane.speedlimit = et.speed;
                lane::road_membership rm;
                rm.parent_road = parent_road;
                rm.interval[0] = l.start_t;
                rm.interval[1] = l.end_t;

                //TODO use lane width value, not constant

                float position = l.offset;
                //Units possibly in half lane widths...

                rm.lane_position = position;
                new_lane.road_memberships.insert(0.0, rm);

		str ramp_id = std::string(l.ramp_id + "_" + std::to_string(0));
		
                lane &ramp_lane = retrieve<lane>(hnet.lanes, ramp_id);

                //Create lane terminus so that ramp flows into extra lane.
                if (l.offramp)
                {
                    lane::lane_terminus* foo = new lane::lane_terminus();
                    foo->adjacent_lane = &ramp_lane;
                    new_lane.end = foo;

                    delete ramp_lane.start;
                    lane::lane_terminus* bar = new lane::lane_terminus();
                    bar->adjacent_lane = &new_lane;
                    ramp_lane.start = bar;

                    new_lane.start = new lane::terminus();
                }
                else
                {
                    lane::lane_terminus* foo = new lane::lane_terminus();
                    foo->adjacent_lane = &ramp_lane;
                    new_lane.start = foo;

                    delete ramp_lane.end;
                    lane::lane_terminus* bar = new lane::lane_terminus();
                    bar->adjacent_lane = &new_lane;
                    ramp_lane.end = bar;

                    new_lane.end = new lane::terminus();
                }

                lane::adjacency la;
                lane* adjacent = 0;
                float min_offset = std::numeric_limits<float>::max();
                //Search forward lanes for the adjacent lane based on offset
                for(lane* neighbor: roads_to_lanes[e.id].first)
                {
                    //We want to find the closest adjacent lane, not one that would line up exactly.
                    if (std::abs(neighbor->road_memberships.find(l.start_t)->second.lane_position - l.offset) == 0)
                    {
                        continue;
                    }

                    float pos_diff = std::abs(neighbor->road_memberships.find(l.start_t)->second.lane_position - l.offset);
                    if (pos_diff < min_offset)
                    {
                        min_offset = pos_diff;
                        adjacent = neighbor;
                    }
                }

                // //Search reverse lanes for the adjacent lane based on offset
                // for(lane* neighbor: roads_to_lanes[e.id].second)
                // {
                //     float pos_diff = std::abs(neighbor->road_memberships.find(l.start_t)->second.lane_position - l.offset);
                //     if (pos_diff < min_offset)
                //     {
                //         min_offset = pos_diff;
                //         adjacent = neighbor;
                //     }
                // }
                
                la.neighbor = adjacent;
                la.neighbor_interval[0] = l.start_t;
                la.neighbor_interval[1] = l.end_t;
                new_lane.left.insert(0.0f, la);

                if (adjacent->right.size() == 0)
                {
                    lane::adjacency nop;
                    nop.neighbor = 0;
                    adjacent->right.insert(0, nop);
                }
                lane::adjacency highway_start;
                highway_start.neighbor = &new_lane;
                highway_start.neighbor_interval[0] = 0;
                highway_start.neighbor_interval[1] = 1;
                adjacent->right.insert(l.start_t, highway_start);
                lane::adjacency highway_end;
                highway_end.neighbor = 0;
                adjacent->right.insert(l.end_t, highway_end);

                //TODO add adjacency to highway lane

                typedef strhash<hwm::lane>::type::value_type hwm_l_pair;
                for(const hwm_l_pair& l_chk: hnet.lanes)
                {
                    l_chk.second.check();
                }
            }
        }

 

        
        
	
        // creating intersections
        typedef strhash<hwm::lane>::type::value_type hwm_l_pair;
        for(const hwm_l_pair& l: hnet.lanes)
        {
            l.second.check();
            assert(l.second.length() > 0);
        }
               
        float STATE_DURATION = 20;
        typedef std::pair<const str, osm::intersection> isect_pair;
        for(const isect_pair& i_pair: onet.intersections)
        {
            const osm::intersection& osm_isect = i_pair.second;

            assert(osm_isect.id_from_node != "");

            hwm::intersection& hwm_isect = hnet.intersections[osm_isect.id_from_node];
            if (hwm_isect.id == "")
                assert(0);

	    	    
            // Add states for every pairing of roads that are ending here, this step looks at edges start and end at the intersection
	    // usually for first set of edges end and this section and another set of edges start at this section, there exists state_pairs
	    // but for edges both start and edge at the section, since some of them are not one way, state pairs could also exist between incoming lanes and reversed outgoing lanes
	    // Also the order for first checking edges end here, then edges start here, then start and end here determine the order of intersection states
            for(int i = 0; i < static_cast<int>(osm_isect.edges_ending_here.size()); i++) // for each edge ends at this intersection
            {
                osm::edge& i_edge = (*osm_isect.edges_ending_here[i]);
                for(int j = i+1; j < static_cast<int>(osm_isect.edges_ending_here.size()); j++) // go through other edges that end at this intersection
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane
                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane
                    //These are "incoming" roads, so forward lanes will be incoming lanes, and reverse lanes will be outgoing lanes.
                    //And the lane "end" is this intersection
		   
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()); k++) // for each forward lane
                    {
                        lane* l = roads_to_lanes[i_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        // get reverse lanes of other edges end at this intersection
			// TODO this means at most one outgoing lane will be connected by the incoming lane
			// this makes the case many incoming lanes one outgoing lane, only one incoming lane can connect to the outgoing lane
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()))
			//for(int kk = 0; kk <static_cast<int>(roads_to_lanes[j_edge.id].second.size()); kk++)
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                                
			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
			    //std::cout<<l_it->intersect_in_ref<<","<<l_j_it->intersect_in_ref<<std::endl;
                        }
                    }

                    //Now match up the forward lanes of the j_edge with the reverse of the i_edge
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].second[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);

			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
			    //std::cout<<l_it->intersect_in_ref<<","<<l_i_it->intersect_in_ref<<std::endl;
                        }
                    }

                    hwm_isect.states.push_back(state);
                }
            }

            typedef strhash<hwm::intersection>::type::value_type hwm_i_pair;
            for(const hwm_i_pair& i: hnet.intersections)
            {
                assert(i.second.id != "");
            }

            //Add states for every pairing of roads that are starting here.
            for(int i = 0; i < static_cast<int>(osm_isect.edges_starting_here.size()); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = i+1; j < static_cast<int>(osm_isect.edges_starting_here.size()); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_starting_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane

                    //These are "outgoing" roads, so reverse lanes will be incoming lanes, and forward lanes will be outgoing lanes.
                    //And the lane "start" is this intersection

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        //Other outgoing road, so we need its forward lanes
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()))
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].first[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                            
			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
                        }
                    }

                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);
                            
			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
			    
                        }
                    }
                    hwm_isect.states.push_back(state);
                }
            }

            //Every pair of outgoing to incoming roads
            for(int i = 0; i < static_cast<int>(osm_isect.edges_starting_here.size()); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = 0; j < static_cast<int>(osm_isect.edges_ending_here.size()); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //Match the reverse lanes (incoming) of the road starting here
                    // with the reverse lanes (outgoing) of the road ending here

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        //Other incoming road, so we need its reverse lanes
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()))
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                            
			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
                        }
                    }

                    
                    // Match the forward lanes (outgoing) of the road ending here
                    // with the forward lanes (incoming) of the road starting here.
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);

			    state.state_pairs.push_back(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
                        }
                    }

                    hwm_isect.states.push_back(state);
                }
            }
        }

        typedef strhash<hwm::intersection>::type::value_type hwm_i_pair;
        for(const hwm_i_pair& i: hnet.intersections)
        {
            assert(i.second.id != "");
        }

        return hnet;
    }
}
