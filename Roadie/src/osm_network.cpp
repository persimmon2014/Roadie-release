/*
 * osm_network.cpp
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

#include "osm_network.hpp"
#include "arc_road.hpp"
#include <vector>
#include <sstream>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <proj_api.h>

static const float  MIPH_TO_MEPS = 1609.344/(60.0*60.0);

typedef unsigned int uint;

namespace osm
{

//Instantiate static
size_t network::new_edges_id = 0;

typedef std::pair<const str, edge>         edge_pair;
typedef std::pair<const str, node>         node_pair;
typedef std::pair<const str, intersection> intr_pair;


node *network::add_node(const vec3f &v, const bool is_overpass)
{
   str id;
   strhash<node>::type::iterator res;
   do
   {
      id = std::to_string(rand());
      res = nodes.find(id);
   }
   while(res != nodes.end());

   res                     = nodes.insert(res, std::make_pair(id, node()));
   res->second.id          = id;
   res->second.xy          = v;
   res->second.is_overpass = is_overpass;

   return &(res->second);
}


bool out_of_bounds(const vec3f &pt)
{
   bool impose_bounds = true;

   float x_lower, x_upper, y_lower, y_upper;

   if (impose_bounds)
   {
      x_lower = -123.105;
      x_upper = -121.439;
      y_upper = 38.09;
      y_lower = 36.911;
   }

   bool in_bounds = ((pt[0] >= x_lower and pt[0] <= x_upper)
                     and
                     (pt[1] >= y_lower and pt[1] <= y_upper));
   return !in_bounds;
}


void network::clip_roads_to_bounds()
{
   //// Remove any edge that has less than 2 nodes after removing out bounds nodes in that edge
   size_t j = 0;
   
   //// address each edge
   while(j < edges.size())
   {
      edge&  e = edges[j];
      
      //// address each node in the edge
      size_t i = 0;
      while (i < e.shape.size()) 
      {
	 bool withinBounds = ((e.shape[i]->xy[0] >= topleft[0] and e.shape[i]->xy[0] <= bottomright[0])
				  and
				  (e.shape[i]->xy[1] >= bottomright[1] and e.shape[i]->xy[1] <= topleft[1]));
         
	 if (!withinBounds) {
#ifdef OSM_DEBUG
	    std::cout<<"Removing out bounds node: "<<e.shape[i]->id<<std::endl;
#endif
            e.shape.erase(e.shape.begin() + i);
	 }
         else
            i++;
      }

      // e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
      // e.shape.erase(e.shape.begin() + new_end, e.shape.end());
      // e.to   = e.shape.back()->id;
      // e.from = e.shape[0]->id;

      if (e.shape.size() < 2) {
#ifdef OSM_DEBUG
	 std::cout<<"Removing edge that has either 0 or 1 node: "<<e.id<<std::endl;
#endif
         edges.erase(edges.begin() + j);
      }
      else
      {
         e.to   = e.shape.back()->id;
         e.from = e.shape[0]->id;
         j++;
	 
      }
   }
}

void network::populate_edges_from_hash()
{
   for(strhash<edge>::type::value_type& hash: edge_hash)
   {
      edges.push_back(hash.second);
   }
}

void network::remove_duplicate_nodes()
{
   for(edge& e: edges)
   {
      e.remove_duplicate_nodes();
   }
}

void network::edges_check()
{
   for(const edge& e: edges)
   {
      assert(e.shape.size() > 1);
   }
}

void network::node_degrees_and_edges_agree()
{
   strhash<int>::type node_degree_check;
   for(const osm::node_pair &np: nodes)
   {
      node_degree_check.insert(std::pair<str,int>(np.first, 0));
   }

   for(osm::edge &e: edges)
   {
      for(osm::node *n: e.shape)
      {
         node_degree_check[n->id]++;
      }
   }

   typedef std::pair<const str, int> n_d;
   for(n_d& nodepair: node_degree_check)
   {
      assert(node_degrees[nodepair.first] == nodepair.second);
   }

   //Check intersections
   for(intr_pair& ip: intersections)
   {
      assert(node_degrees[ip.first] > 1);
   }

   for(const edge& ep: edges)
   {
      assert(ep.to == ep.shape.back()->id);
      assert(ep.from == ep.shape[0]->id);
   }
}

void network::list_edges()
{
   for(edge& e: edges)
   {
      std::cout << " edge id " << e.id << std::endl;
   }
}


void network::populate_edge_hash_from_edges()
{
   edge_hash.erase(edge_hash.begin(), edge_hash.end());

   for(edge& e: edges)
   {
      assert(e.length() < 10e10);
      edge_hash.insert(std::make_pair(e.id, e));
   }
}

float edge::length() const
{
   float len_thus_far = 0;
   for(int i = 0; i < static_cast<int>(shape.size()) - 1; i++)
      len_thus_far += planar_distance(shape[i + 1]->xy, shape[i]->xy);

   return len_thus_far;
}

void network::print_highway_lengths()
{
   std::cout << "___________________________________\n";
   for (int i = 0; i < (int) edges.size(); i++)
   {
      edge& e = edges[i];

      if (e.highway_class == "motorway")
      {
         std::cout << e.length() << std::endl;
      }
   }
   std::cout << "___________________________________\n";
}

void network::remove_small_roads()
{
   assert(road_remove_threshold >= 0);
   // the length comparision is in 2d (using xy[0] and xy[1] only)
   for(int i = 0; i < static_cast<int>(edges.size()); )
   {
      edge& e = edges[i];
     
      if (e.length() < road_remove_threshold)
      {             
         // //Update intersections
         // if (intersections.find(e.to) != intersections.end())
         // {
         //     intersection& i_to = intersections[e.to];
         //     i_to.edges_ending_here.erase(find(i_to.edges_ending_here.begin(), i_to.edges_ending_here.end(), &e));
         // }

         // if (intersections.find(e.from) != intersections.end())
         // {
         //     intersection& i_from = intersections[e.from];
         //     i_from.edges_starting_here.erase(find(i_from.edges_starting_here.begin(), i_from.edges_starting_here.end(), &e));
         // }

         //Update degree count for all nodes
         for(node* n: e.shape)
         {
            node_degrees[n->id]--;
         }

         //TODO join roads that this edge connected...        
         // remove small roads
         
          
         std::swap(edges[i], edges.back());
         edges.pop_back();
#ifdef OSM_DEBUG
	 std::cout<<"Removing [length < "<<road_remove_threshold<<"] edge: "<<e.id<<std::endl;
#endif
      }
      else {
         ++i;
#ifdef OSM_DEBUG
	 std::cout<<"Length of edge "<<e.id<<": "<<e.length()<<"(m)"<<std::endl;
#endif
      }
   }
}

void network::remove_small_roads_agree()
{
   assert(node_culling_threshold >= 0.0);
   
   for(edge& e: edges)
   {
     vec3f current = e.shape[0]->xy;
     for(int i = 1; i < e.shape.size(); i++) {
	
       if(distance2<vec3f>(current, e.shape[i]->xy) > node_culling_threshold * node_culling_threshold)
	 break;
      
       if(i == e.shape.size()-1) {
	 std::cerr<<"Need to increase roads removing threshold!"<<std::endl;
	 std::cerr<<"Edge "<<e.id<<" fails."<<std::endl;
	 exit(0); 
       }
       
     }
   }
}

// Creates a w by h grid of nodes and connects them with roads of length dw and dh.
void network::create_grid(int w, int h, double dw, double dh)
{
   std::vector<std::vector< node*> > node_grid (w, std::vector<node*>(h));

   for(int i = 0; i < w; i++)
   {
      for(int j = 0; j < h; j++)
      {
         std::stringstream node_id;
         node_id << "node " << i << "_" << j;
         node* n = retrieve<node>(nodes, str(node_id.str()));
         n->id = str(node_id.str());
         n->xy[0] = i*dw;
         n->xy[1] = j*dh;
         n->xy[2] = 0.0;

         std::cout << n->xy[0] << " " << n->xy[1] << std::endl;

         node_grid[i][j] = n;

         if (j != 0) //create vertical edge
         {
            str   e_id = n->id+"to"+node_grid[i][j-1]->id;
            edge* e    = NULL;
            for(int k=0; k < static_cast<int>(edges.size()); k++)
               if (edges[k].id == e_id)
               {
                  e = &edges[k];
                  break;
               }
            if (e == NULL)
            {
               edges.push_back(edge());
               e = &edges.back();
            }
            e->id = e_id;
            e->from = n->id;
            e->to = node_grid[i][j-1]->id;
            e->highway_class = "residential";
            e->shape.push_back(&nodes[e->from]);
            e->shape.push_back(&nodes[e->to]);
         }

         if (i != 0)
         {
            str   e_id = n->id+"to"+node_grid[i-1][j]->id;
            edge* e    = NULL;
            for(int k=0; k < static_cast<int>(edges.size()); k++)
               if (edges[k].id == e_id)
               {
                  e = &edges[k];
                  break;
               }
            if (e == NULL)
            {
               edges.push_back(edge());
               e = &edges.back();
            }
            e->id            = e_id;
            e->from          = n->id;
            e->to            = node_grid[i-1][j]->id;
            e->highway_class = "residential";
            e->shape.push_back(&nodes[e->from]);
            e->shape.push_back(&nodes[e->to]);
         }
      }
   }
}

void network::compute_node_degrees()
{
   //// compute node degrees <str,int>: the number of edges that each node belongs to, most nodes will have the degree=1; after removing out bounds nodes, some nodes will have degree=0
   node_degrees.erase(node_degrees.begin(), node_degrees.end());
   assert(node_degrees.size() == 0);
  
   //// osm::node_pair <const str, node> 
   for(osm::node_pair &np: nodes)
   {
      np.second.edges_including.erase(np.second.edges_including.begin(), np.second.edges_including.end());
   }

   for(const osm::node_pair &np: nodes)
   {
      node_degrees.insert(std::pair<str,int>(np.first, 0));
   }

   for(osm::edge &e: edges)
   {
      for(osm::node *n: e.shape)
      {
         node_degrees[n->id]++; // if a node belongs to an edge, increase its degree
         n->edges_including.push_back(&e); // also store the edge that the node belongs to
      }
   }
   
   //// Debug
//    std::cout<<"node_degrees size: "<<node_degrees.size()<<std::endl;
// for(auto iter = node_degrees.begin(); iter != node_degrees.end(); iter++)  {
// 	if(iter->second >1)
// 	std::cout<<iter->first<<":"<<iter->second<<std::endl;
// }

}

void network::edges_including_rebuild()
{
   for(osm::node_pair &np: nodes)
   {
      np.second.edges_including.erase(np.second.edges_including.begin(), np.second.edges_including.end());
      assert(np.second.edges_including.size() == 0);
   }

   for(osm::edge &e: edges)
   {
      for(node* n: e.shape)
      {
         n->edges_including.push_back(&e);
      }
   }
}

namespace create_ramps
{

_min_pair find_closest(vec3f pt, arc_road road, float offset)
{
   //Initialize minimum data
   _min_pair min_pair;
   min_pair.dist = std::numeric_limits<float>::max();

   return create_ramps::find_closest_recurse(pt, road, 0, 1, min_pair, offset);
}

_min_pair find_closest_recurse(vec3f pt, arc_road road, float start_t, float end_t, _min_pair min_pair, float offset)
{
   //Test n locations.
   float n = 10.0;
   for (float i = 0; i < n; i++)
   {
      float sample_t = (i / n)*(end_t - start_t) + start_t;
      vec3f sample_pt = road.point(sample_t, offset);

      //std::cout << "road point: " << sample_pt[0] << " " << sample_pt[1] << " " << sample_pt[2]<<std::endl;
      float dist = std::sqrt(dot(pt - sample_pt, pt - sample_pt));
      if (dist < min_pair.dist)
      {
         min_pair.t = sample_t;
         min_pair.dist = dist;
      }
   }

   const float DIST_EPS = 0.01;
   if ((std::abs(min_pair.dist) < DIST_EPS) or (std::abs(end_t - start_t) < DIST_EPS))
   {
      //Return when you find something close enough or your bounded region is small enough.
      return min_pair;
   }
   else
   {
      float new_start_t = std::max(0.0, (min_pair.t - ((1.0/n)*(end_t - start_t))));
      float new_end_t = std::min(1.0, (min_pair.t + ((1.0/n)*(end_t - start_t))));
      return find_closest_recurse(pt, road, new_start_t, new_end_t, min_pair, offset);
   }
}

}

void network::create_ramps()
{
   assert(lane_width > 0);
   edges_including_rebuild();

   for(osm::edge &e: edges)
   {
      //For each ramp..
      if (e.highway_class == "motorway_link")
      {	
         for (int b = 0; b < 2; b++)
         {
            size_t i;
            if (b == 0)
               i = 0; // get the first node index
            else if (b == 1)
               i = ((int)e.shape.size()) - 1; // get the last node index
            else
               assert(0);

            node* n = e.shape[i]; // get the node
            if (n->ramp_merging_point != NULL)
            {
               node*      highway_node         = n->ramp_merging_point;
               bool       highway_intersection = false;
               osm::edge* highway              = NULL;
               for(osm::edge *e: highway_node->edges_including)
               {
                  highway_intersection = (e->highway_class == "motorway");

                  if (highway_intersection)
                  {
                     highway = e;
                     break;
                  }
               }

               if (highway_intersection)
               {
                  //Create arc road for highway.
                  arc_road highway_shape;
                  for(node* foo: highway->shape)
                  {
                     highway_shape.points_.push_back(foo->xy);
                  }

                  highway_shape.initialize_from_polyline(0.7, highway_shape.points_);

                  ///First, we need the offset of the point where the ramp will merge.
                  ///This will always be on the right side of the road, one lane beyond the end of the road.
                  float lanect  = -1;
                  float nolanes = highway->type->nolanes;

                  //TODO use lane width value, not constant
                  float offset = lane_width*(lanect + -((nolanes - 1)/2.0));

                  //Find t of road where the ramp intersects
                  float t_offset = create_ramps::find_closest(highway_node->xy, highway_shape, offset).t;
                  float t = create_ramps::find_closest(highway_node->xy, highway_shape, 0.0f).t;

                  vec3f pt = highway_shape.point(t_offset, offset);

                  n->xy = pt;

                  float len = 15;
                  //Load the tangent based on the direction of the ramp.
                  //Move the next to last (or second) point to make the ramp tangent to the highway.
                  if (i == 0)
                  {
                     vec3f tan(col(highway_shape.frame(t, offset, false), 0));
                     e.shape.insert(e.shape.begin() + 1, new node);
                     e.shape[i + 1]->id = str(std::to_string(rand()));
                     e.shape[i + 1]->xy = len*tan + pt;
                  }
                  else if (i + 1 == e.shape.size())
                  {
                     vec3f tan(col(highway_shape.frame(t, offset, true), 0));
                     e.shape.insert(e.shape.begin() + i, new node);
                     e.shape[i]->id = str(std::to_string(rand()));
                     e.shape[i]->xy = len*tan + pt;
                  }
                  else
                  {
                     assert(0); //The ramp should only connect at its edges.
                  }

                  float t_center = t;
                  float merge_lane_len            = 60;
                  float merge_lane_parametric_len = merge_lane_len / highway_shape.length(offset);

                  //Add a merging lane to the highway
                  if (i == 0)//Offramp
                  {
                     if (std::abs(std::max((float)0, t_center - merge_lane_parametric_len) - t_center) > 0)
                     {
                        highway->additional_lanes.push_back(osm::edge::lane(std::max((float)0, t_center - merge_lane_parametric_len),
                                                                            t_center,
                                                                            offset,
                                                                            e.id,
                                                                            true));
                     }
                  }
                  else//Onramp
                  {
                     if (std::abs(std::min((float)1, t_center + merge_lane_parametric_len) - t_center) > 0)
                     {
                        highway->additional_lanes.push_back(osm::edge::lane(t_center,
                                                                            std::min((float)1, t_center + merge_lane_parametric_len),
                                                                            offset,
                                                                            e.id,
                                                                            false));
                     }
                  }
               }
            }
         }
      }
   }
}

struct _intersection
{
   vec3f xy;
   osm::edge* hwy1;
   int opass_start_point_hwy1;
   int lvl;
};

struct isects
{
   int lvl;
   std::vector<_intersection> crossings;
};

void network::assign_highway_heights()
{
   struct _aux
   {
      static bool between(float test, float b1, float b2)
      {
         float min = std::min(b1, b2);
         float max = std::max(b1, b2);

         if ((min < test) and (test < max))
            return true;
         else
            return false;
      }
   };

   typedef std::pair<osm::edge*, isects> intersections_elem;
   std::map<osm::edge*, isects> intersections;

   typedef std::pair<osm::edge*, std::vector<osm::edge*> > map_elem;
   typedef std::map<osm::edge*, std::vector<osm::edge*> > Q_map;
   Q_map conflicting;

   edges_including_rebuild();
   compute_node_degrees();

   //Find line-line intersections (for those edges intersect with each other but don't have a shared node as the intersection point
   for (size_t l = 0; l < edges.size(); l++)
   {
      osm::edge& e = edges[l];
      for (size_t m = 0; m < edges.size(); m++)
      {
         osm::edge& f = edges[m];
         if (e == f)
         {
            continue;
         }

         for (size_t i = 1; i < e.shape.size(); i++)
         {
            for (size_t j = 1; j < f.shape.size(); j++)
            {
               float x1 = e.shape[i - 1]->xy[0];
               float y1 = e.shape[i - 1]->xy[1];
               float x2 = e.shape[i]->xy[0];
               float y2 = e.shape[i]->xy[1];

               float x3 = f.shape[j - 1]->xy[0];
               float y3 = f.shape[j - 1]->xy[1];
               float x4 = f.shape[j]->xy[0];
               float y4 = f.shape[j]->xy[1];

               //calculate intersect point
               float ua = ((x4 - x3)*(y1 - y3) - (y4 - y3)*(x1 - x3)) /
                          ((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1));

               if (((y4 - y3)*(x2 - x1) - (x4 - x3)*(y2 - y1))  == 0)
                  continue;

               float x = x1 + ua*(x2 - x1);
               float y = y1 + ua*(y2 - y1);

               if (_aux::between(x, x1, x2) and _aux::between(y, y1, y2)
                   and
                   _aux::between(x, x3, x4) and _aux::between(y, y3, y4))
               {
                  //Store intersection.
                  _intersection isect1;
                  isect1.xy[0] = x;
                  isect1.xy[1] = y;
                  isect1.hwy1 = &e;
                  isect1.opass_start_point_hwy1 = i-1;
                  intersections[&e].crossings.push_back(isect1); // for each edge e store the intersecting

                  _intersection isect2;
                  isect2.xy[0] = x;
                  isect2.xy[1] = y;
                  isect2.hwy1 = &f;
                  isect2.opass_start_point_hwy1 = j-1;
                  intersections[&f].crossings.push_back(isect2); // for each edge f also store the intersecting

                  //Add to conflicting sets
                  conflicting[&e].push_back(&f);
               }
            }
         }
      }
   }

   //// Assign Levels
   //// Normally, these would need to be sorted.  However, they are all 2
   Q_map Qi = conflicting;
   int lvl = 0;  
   
#ifdef OSM_DEBUG
   std::cout<<"Assigning highway levels......"<<std::endl;
#endif
   
   
   // each iteration assigns edges in Qi.first the same level value, and store their conflicts temporarily in Qipp
   // after these steps, set Qi as Qipp and continue to assign level value which is increased by 1
   while (Qi.size() != 0)
   {
      Q_map Qipp;
      
      for(map_elem p: Qi)
      {
         //Assign level to p in Qi
         osm::edge* e = p.first;
         intersections[e].lvl = lvl;
#ifdef OSM_DEBUG
         std::cout<<"Edge "<<e->id<< " gets level "<<lvl<<std::endl;
#endif
         //Add its conflicts to Qipp and remove them from Qi
         for (int k = 0; k < p.second.size(); k++)
         {
            if (Qi[p.second[k]].size() > 0)
            {
#ifdef OSM_DEBUG
               std::cout << "Adding " << e->id << std::endl;
#endif
               Qipp.insert(std::make_pair(p.second[k], Qi[p.second[k]]));
            }
            Qi.erase(Qi.find(p.second[k]));
         }
      }

      Qi = Qipp;
      lvl++;
   }


   //Assign heights from levels
   float level_height = 1;
   for(intersections_elem isect_pair: intersections)
   {
      osm::edge& e = *isect_pair.first;
      for (int i = 0; i < isect_pair.second.crossings.size(); i++)
      {
         _intersection& isect = isect_pair.second.crossings[i];
         e.shape[isect_pair.second.crossings[i].opass_start_point_hwy1]->xy[2] = isect_pair.second.lvl * level_height;
         e.shape[isect_pair.second.crossings[i].opass_start_point_hwy1 + 1]->xy[2] = isect_pair.second.lvl * level_height;
      }
   }

   //// Address each edge when nodes in the edge have different levels
   for (int i = 0; i < edges.size(); i++)
   {
      osm::edge& e = edges[i];
      bool first_found = false;
      int first = 0;
      int last = 0;

      
      //// find the first node with y>0, and the last node with y>0   
      float height = 0;
      for (int j = 0; j < e.shape.size(); j++)
      {
         if ((e.shape[j]->xy[2] > 0) and (!first_found))
         {
            first_found = true;
#ifdef OSM_DEBUG
            std::cout << "for " << e.id << " found " << j << std::endl;
#endif
            first = j;
            height = e.shape[j]->xy[2];
         }

         if (e.shape[j]->xy[2] > 0)
         {
            last = j;
#ifdef OSM_DEBUG
            std::cout << " to " << j << std::endl;
#endif
         }
      }
      
      
      // height stores the first node on edge that has y>0, here we address edges that do have inside nodes y>0
      if (height > 0)
      {
         for (int j = first; j < last; j++)
         {
#ifdef OSM_DEBUG
            std::cout << j << " gets " << height << std::endl;
#endif
            e.shape[j]->xy[2] = height;
         }

         float ramp_dist = 100.0;
	 
	 // from the first node on edge to first node with y>0
         for (int j = 0; j < first; j++)
         {
#ifdef OSM_DEBUG
            std::cout << "first: " << std::endl;
#endif
            //ramp up
            float dist = std::sqrt(pow(e.shape[j]->xy[0] - e.shape[first]->xy[0], 2)
                                   +
                                   pow(e.shape[j]->xy[1] - e.shape[first]->xy[1], 2));
#ifdef OSM_DEBUG	    
            std::cout << (std::min(dist, ramp_dist)/ramp_dist) << std::endl;
#endif
	    
	    // assign actual y values based on distance between the current node to first node with y>0
            e.shape[j]->xy[2] = height*(1 - (std::min(dist, ramp_dist)/ramp_dist));
         }

         
         // from the last node with y>0 the the last node on the edge
         for (int j = last; j < e.shape.size(); j++)
         {
#ifdef OSM_DEBUG
            std::cout << "last: " << std::endl;
#endif
            //ramp down
            float dist = std::sqrt(pow(e.shape[j]->xy[0] - e.shape[last]->xy[0], 2)
                                   +
                                   pow(e.shape[j]->xy[1] - e.shape[last]->xy[1], 2));
#ifdef OSM_DEBUG
            std::cout << (1 - (std::max(dist, ramp_dist)/ramp_dist)) << std::endl;
#endif
	    // assign actual y values based on distance between the last node with y>0 and the last node on the edge
            e.shape[j]->xy[2] = height*(1 - (std::min(dist, ramp_dist)/ramp_dist));
         }
      }
   }
}


void network::remove_motorway_intersections()
{
   //edges_including_rebuild();
   compute_node_degrees();

   for(osm::edge &e: edges)
   {
      if (e.highway_class == "motorway")
      {
         for(size_t i = 0; i < e.shape.size(); i++)
         {
            bool   ramp_node = false;
            node*& n         = e.shape[i];

            //Found an intersection
            if (node_degrees[n->id] > 2)
            {
               //Removing node from highway
               node_degrees[n->id]--;

               node* old    = n;
               str   new_id = old->id + "_HWY";
               n            = retrieve<node>(nodes, new_id);
	       std::cout<<"road id="<<new_id<<" is created."<<std::endl;

               //If there is a ramp "motorway_link" at this intersection, store the connecting node
               for(edge* e: old->edges_including)
               {
                  if (e->highway_class == "motorway_link")
                  {
                     old->ramp_merging_point = n;
                     ramp_node = true;
                  }
               }

               //Make old node an overpass if it isn't a merging point
               if (!ramp_node)
                  old->is_overpass = true;

	       
               n->xy = old->xy;
               //TODO edges_including..
               n->id = new_id;
               n->edges_including.push_back(&e);
	       
	       // remove current edge from the intersection node, since this node is treated either as ramp_node or overpass
               if (find(old->edges_including.begin(),
                        old->edges_including.end(),
                        &e) != old->edges_including.end())
                  old->edges_including.erase(find(old->edges_including.begin(),
                                                  old->edges_including.end(),
                                                  &e));


               if (node_degrees.find(n->id) == node_degrees.end())
                  node_degrees[n->id] = 0;
	       
               node_degrees[n->id]++;

               //If node is at the end of the road
               if (i + 1 == e.shape.size())
                  e.to = n->id;
               if (i == 0)
                  e.from = n->id;

            }
            else if (n->ramp_merging_point != NULL || n->is_overpass == true)
            {
               //Removing node from motorway
               node_degrees[n->id]--;

               node* old    = n;
               str   new_id = old->id + "_HWY";
               n            = retrieve<node>(nodes, new_id);
               n->xy = old->xy;
               n->id = new_id;
               n->edges_including.push_back(&e);
               if (find(old->edges_including.begin(),
                        old->edges_including.end(),
                        &e) != old->edges_including.end())
                  old->edges_including.erase(find(old->edges_including.begin(),
                                                  old->edges_including.end(),
                                                  &e));


               if (node_degrees.find(n->id) == node_degrees.end())
                  node_degrees[n->id] = 0;
               node_degrees[n->id]++;

               //If node is at the end of the road
               if (i + 1 == e.shape.size())
                  e.to = n->id;
               if (i == 0)
                  e.from = n->id;

            }
         }
      }
   }
}

typedef std::pair<vec2f, vec2f> pair_of_isects;
static pair_of_isects circle_line_intersection(const vec2f &pt1,
                                               const vec2f &pt2,
                                               const vec2f &center,
                                               float r)
{
   const float x1 = pt1[0] - center[0];
   const float y1 = pt1[1] - center[1];
   const float x2 = pt2[0] - center[0];
   const float y2 = pt2[1] - center[1];

   const float dx = (x2 - x1);
   const float dy = (y2 - y1);
   const float dr = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
   const float D  = x1*y2 - x2*y1;

   const vec2f isect1(static_cast<float>((D*dy + copysign(1.0f, dy)*dx*std::sqrt(r*r*dr*dr - D*D))/(dr*dr)) + center[0],
                      -D*dx + std::abs(dy)*std::sqrt(r*r*dr*dr - D*D)/(dr*dr) + center[1]);

   const vec2f isect2(static_cast<float>((D*dy - copysign(1.0f, dy)*dx*std::sqrt(r*r*dr*dr - D*D))/(dr*dr) + center[0]),
                      -D*dx - std::abs(dy)*std::sqrt(r*r*dr*dr - D*D)/(dr*dr) + center[1]);

   return std::make_pair(isect1, isect2);
}

void network::compute_node_heights()
{
   // this function assumes that if two lines intersect but don't share a common node as the intersect point, then two lines (roads) must have different heights (rely on osm file correctness)
   float overpass_height = 5;

   for(osm::edge& e: edges)
   {
      std::vector<node*> new_shape;
      for(int i = 0; i < static_cast<int>(e.shape.size()); i++)
      {
         osm::node* n = e.shape[i];
         //Find node that's part of overpass.
         if (n->is_overpass)
         {
            //TODO What if road ends before ramp radius is reached? Could continue to traverse connecting roads, but that could produce odd effects.

            n->xy[2] += overpass_height;

            //         vec2f n_2d(n->xy[0], n->xy[1]);

            //         //Walk back until
            //         //  1) another overpass is found, or
            //         //  2) until a line segment intersects with a circle centered at point i
            //         float overpass_radius = 60;
            //         float len_thus_far = 0;
            //         int last_index = -1;
            //         for(int j = i - 1; j >= 0; j--)
            //         {
            //             osm::node* j_node = e.shape[j];

            //             vec3f j_3d(j_node->xy[0], j_node->xy[1], 0);
            //             vec3f jp1_3d(e.shape[j + 1]->xy[0], e.shape[j + 1]->xy[1], 0);

            //             if (j_node->is_overpass)
            //                 break;

            //             float dist = norm2(j_3d - jp1_3d);
            //             if (len_thus_far + dist > overpass_radius)
            //             {
            //                 float param = (overpass_radius - len_thus_far) / dist;
            //                 vec3f new_point(param*(j_3d - jp1_3d) + jp1_3d);
            //                 osm::node ramp_start;
            //                 ramp_start.id = n->id + "_opassramp";
            //                 ramp_start.xy = vec3f(new_point);
            //                 ramp_start.edges_including.push_back(&e);
            //                 node_degrees[ramp_start.id] = 1;
            //                 strhash<node>::type::iterator pos = nodes.insert(std::make_pair(ramp_start.id, ramp_start)).first;
            //                 new_shape.push_back(&(pos->second));
            //                 break;
            //             }
            //             else
            //             {
            //                 len_thus_far += dist;
            //                 new_shape[j]->xy[2] = (len_thus_far / overpass_radius) / overpass_height;
            //             }
            //         }
            //         new_shape.push_back(n);
            //     }
            //     else
            //         new_shape.push_back(n);
         }

         // e.shape.clear();
         // for (int i = 0; i < new_shape.size(); i++)
         // {
         //     e.shape.push_back(new_shape[i]);
         // }
      }
   }
}


void network::intersection_check()
{
   for(const osm::intr_pair &ip: intersections)
   {
      assert(ip.first == ip.second.id_from_node);
      assert(node_degrees[ip.first] > 1);
   }

   //Check that intersections don't occur in the middle of roads.
   for(const osm::edge &e: edges)
   {
      for (int i = 1; i < static_cast<int>(e.shape.size()) - 1; i++)
         assert(node_degrees[e.shape[i]->id] == 1);
   }
}

namespace create_intersections
{
struct Edge_Offset
{
   float offset;
   vec2f approach_vector;
   int num_of_lanes;
   float angle;
   osm::edge* parent_edge;
   static bool clockwise(const Edge_Offset& a, const Edge_Offset& b){return a.angle < b.angle;}
};

typedef enum x{SIGNAL, MERGE} intersection_class;

intersection_class classify(const intersection& intr, std::vector<create_intersections::Edge_Offset> edge_offsets)
{
   bool all_highway = true;
   for (int i = 0; i < intr.edges_ending_here.size(); i++)
   {
      all_highway = all_highway and (intr.edges_ending_here[i]->highway_class == "motorway");
   }
   for (int i = 0; i < intr.edges_starting_here.size(); i++)
   {
      all_highway = all_highway and (intr.edges_starting_here[i]->highway_class == "motorway");
   }

   //MERGE type not used yet

   return SIGNAL;
}

};

void network::create_intersections()
{
   assert(lane_width > 0);
   compute_node_degrees();
   
//    for(osm::edge &e: edges) 
//    {
//      std::cout <<"Edge "<< e.id << std::endl;
//      
//      for(int i=0; i<e.shape.size(); i++) {
//        std::cout<<e.shape[i]->id<<std::endl;
//     }
//     
//     std::cout<<std::endl;
//    }
//  

   for(osm::edge &e: edges)
   {
      assert(e.to == e.shape.back()->id);
      assert(e.from == e.shape[0]->id);
      if (node_degrees[e.to] > 2) // if the last node of this edge connects more than 2 roads, it's an intersection
      {
         if (e.highway_class == "motorway")
         {
            std::cout <<"Edge "<< e.id << " is an intersection to node " << e.to << std::endl;
            std::cout << node_degrees[e.to] << std::endl;
         }
         intersection* curr;
         curr = retrieve<intersection>(intersections, e.to); // create an intersection with id e.to
         curr->edges_ending_here.push_back(&e); // store edges that ends at this intersection
         curr->id_from_node = e.to;
	 //std::cout <<"An intersection is created " << e.to << std::endl;
      }
      if (node_degrees[e.from] > 2)
      {
         if (e.highway_class == "motorway")
         {
            std::cout <<"Edge "<< e.id << " is an intersection from node " << e.from << std::endl;
            std::cout << node_degrees[e.from] << std::endl;
         }
         intersection* curr;
         curr = retrieve<intersection>(intersections, e.from);
         curr->edges_starting_here.push_back(&e);
         curr->id_from_node = e.from;
	 //std::cout <<"An intersection is created " << e.from << std::endl;
      }
   }

   int intersection_count = 0;
   std::cout << "Total number of intersections: " << intersections.size() << std::endl; // till this point, intersections are starting or ending nodes that have degree more than 2
   std::map<std::string, bool> offset_once;

   //Pull back roads to make room for intersections.
   for(const osm::intr_pair &ip: intersections)
   {
      const intersection&  i = ip.second;
      const node &intersection_node(nodes[i.id_from_node]);

      // Create a vector of edge offsets
      std::vector<create_intersections::Edge_Offset> edge_offsets;
      for(edge* an_edge: i.edges_starting_here)
      {
         create_intersections::Edge_Offset an_edge_offset;
         //std::cout << "Edge starting here id=" << an_edge->id << std::endl;
	 
         //Compute the unit vector of an edge starting at this intersection (using first two nodes)
         vec2f vec(an_edge->shape[1]->xy[0] - an_edge->shape[0]->xy[0],
                   an_edge->shape[1]->xy[1] - an_edge->shape[0]->xy[1]);
	 
         vec *= 1.0/tvmet::norm2(vec);
         
	 assert(!isnan(vec[0]));
         assert(!isnan(vec[1]));

         an_edge_offset.approach_vector  = vec;

         //Record the number of lanes
         an_edge_offset.num_of_lanes = an_edge->type->nolanes;

         //Calculate the angle
         float x = an_edge_offset.approach_vector[0];
         float y = an_edge_offset.approach_vector[1];
         an_edge_offset.angle = atan2(y, x);

         //Initialize the offset to -inf (essentially)
         an_edge_offset.offset = -1*std::numeric_limits<float>::max();

         //Record the edge used to create this structure
         an_edge_offset.parent_edge = an_edge;

         //Save the edge offset
         edge_offsets.push_back(an_edge_offset);
      }

      for(edge* an_edge: i.edges_ending_here)
      {
         create_intersections::Edge_Offset an_edge_offset;
	 //std::cout << "Edge ending here id=" << an_edge->id << std::endl;
	 
         size_t last = an_edge->shape.size() - 1;
         size_t penultimate = an_edge->shape.size() - 2;
         vec2f vec(an_edge->shape[penultimate]->xy[0] - an_edge->shape[last]->xy[0],
                   an_edge->shape[penultimate]->xy[1] - an_edge->shape[last]->xy[1]);
         assert(!isnan(vec[0]));
         assert(!isnan(vec[1]));
         
	 vec *= 1.0/tvmet::norm2(vec);
         an_edge_offset.approach_vector  = vec;

         //Calculate the angle
         float x = an_edge_offset.approach_vector[0];
         float y = an_edge_offset.approach_vector[1];
         an_edge_offset.angle = atan2(y, x);

         //Initialize the offset to -inf
         an_edge_offset.offset = -1*std::numeric_limits<float>::max();

         //Record the number of lanes
         an_edge_offset.num_of_lanes = an_edge->type->nolanes;

         //Record the edge used to create this structure
         an_edge_offset.parent_edge = an_edge;

         edge_offsets.push_back(an_edge_offset);
      }

      sort(edge_offsets.begin(), edge_offsets.end(), create_intersections::Edge_Offset::clockwise);
      
      assert(i.edges_ending_here.size() + i.edges_starting_here.size() == edge_offsets.size());

      //Classify intersection
      create_intersections::intersection_class intersection_class = create_intersections::classify(i, edge_offsets);
     
      //Create a map of edges to their offsets
      std::map<edge*, create_intersections::Edge_Offset*> edges_to_offsets;
      if (intersection_class == create_intersections::SIGNAL)
      {
         intersection_count++; // for now it will be the same value as intersections.size() since all intersections are signalized

#ifdef OSM_DEBUG
         std::cout << "Intersection id=" << i.id_from_node << std::endl;
         std::cout << intersection_count << " intersections so far" << std::endl;
#endif
	 
         //For signalized intersection, calculate offsets
         assert(edge_offsets.size() > 1);
         
	 /* TODO This parameter could be learned from images. */
         float initial_min_radius = 5; // set initial min radius which is equivalent to MIN_LEN_EPS 
         float min_radius;
         float max_offset;

         const float MIN_RAD_EPS = 1;
         const float MIN_LEN_EPS = 5; // the road length should be at least initial_min_radius

         for (size_t j = 0; j < edge_offsets.size(); j++)
         {
            int iteration_number = 0;
            do
            {
               min_radius = std::max(initial_min_radius / (iteration_number + 1), MIN_RAD_EPS);
               max_offset = edge_offsets[j].parent_edge->length() - MIN_LEN_EPS; // edge length - MIN_LEN_EPS
               bool visited = offset_once.find(edge_offsets[j].parent_edge->id) != offset_once.end();
               if (!visited)
               {
                  max_offset /= 2.0;
               }


               edge_offsets[j].offset = -1*std::numeric_limits<float>::max();
               {
                  //                            int clockwise_neighbor = (j == edge_offsets.size() - 1? 0 : j + 1);
                  int clockwise_neighbor = (j == 0? edge_offsets.size() - 1 : j - 1);

                  int no_lanes = std::max(edge_offsets[j].num_of_lanes, edge_offsets[clockwise_neighbor].num_of_lanes);

                  float intersection_min = no_lanes * lane_width + min_radius;

                  vec2f clockwise_perp_a;
                  // clockwise_perp_a[0]  = -edge_offsets[j].approach_vector[1];
                  // clockwise_perp_a[1]  = edge_offsets[j].approach_vector[0];
                  clockwise_perp_a[0]  = edge_offsets[j].approach_vector[1];
                  clockwise_perp_a[1]  = -edge_offsets[j].approach_vector[0];
                  clockwise_perp_a    *= (no_lanes*lane_width + min_radius);

                  vec2f cclockwise_perp_b;
                  // cclockwise_perp_b[0]  = edge_offsets[clockwise_neighbor].approach_vector[1];
                  // cclockwise_perp_b[1]  = -edge_offsets[clockwise_neighbor].approach_vector[0];
                  cclockwise_perp_b[0]  = -edge_offsets[clockwise_neighbor].approach_vector[1];
                  cclockwise_perp_b[1]  = edge_offsets[clockwise_neighbor].approach_vector[0];
                  cclockwise_perp_b    *= (no_lanes*lane_width + min_radius);

                  vec2f sum(clockwise_perp_a + cclockwise_perp_b);
                  float len = norm2(sum);
                  //                            float theta = edge_offsets[clockwise_neighbor].angle - edge_offsets[j].angle;
                  float theta = edge_offsets[j].angle - edge_offsets[clockwise_neighbor].angle;

                  if (theta < 0){
                     theta += 2*M_PI;
                  }

                  //                                             float offset = len / (2.0f * sin(theta / 2.0f));

                  //float offset = len * (cos(theta / 2.0f));
                  float offset = (0.5 * no_lanes*lane_width + min_radius) / (tan(theta/2.0f));

                  if (offset > edge_offsets[j].offset)
                  {
                     //Store the max of the offset and a minimum intersection
                     edge_offsets[j].offset = std::max(offset, intersection_min);
                  }
               }
               {
                  //                            int cclockwise_neighbor = (j == 0? edge_offsets.size() - 1 : j - 1);
                  int cclockwise_neighbor = (j == edge_offsets.size() - 1? 0 : j + 1);

                  int no_lanes = std::max(edge_offsets[j].num_of_lanes, edge_offsets[cclockwise_neighbor].num_of_lanes);
                  float intersection_min = no_lanes * lane_width + min_radius;

                  vec2f clockwise_perp_a;
                  clockwise_perp_a[0]  = -edge_offsets[j].approach_vector[1];
                  clockwise_perp_a[1]  = edge_offsets[j].approach_vector[0];
                  clockwise_perp_a    *= (no_lanes*lane_width + min_radius);

                  vec2f cclockwise_perp_b;
                  cclockwise_perp_b[0]  = edge_offsets[cclockwise_neighbor].approach_vector[1];
                  cclockwise_perp_b[1]  = -edge_offsets[cclockwise_neighbor].approach_vector[0];
                  cclockwise_perp_b    *= (no_lanes*lane_width + min_radius);

                  //was -
                  vec2f sum(clockwise_perp_a + cclockwise_perp_b);
                  float len = norm2(sum);
                  //float theta = edge_offsets[j].angle - edge_offsets[cclockwise_neighbor].angle;
                  float theta = edge_offsets[cclockwise_neighbor].angle - edge_offsets[j].angle;

                  if (theta < 0){
                     theta += 2*M_PI;
                  }

                  //                            float offset = len / (2.0f * sin(theta / 2.0f));
                  //                        float offset = len * (cos(theta / 2.0f));
                  float offset = (0.5 * no_lanes*lane_width + min_radius) / (tan(theta/2.0f));

                  if (offset > edge_offsets[j].offset)
                  {
                     //Store the max of the offset and a minimum intersection
                     edge_offsets[j].offset = std::max(offset, intersection_min);
                  }
               }
               iteration_number++;

               if (iteration_number > 1000)
                  edge_offsets[j].offset = max_offset - 1;
            }
            while (edge_offsets[j].offset >= max_offset);
         }

         for(create_intersections::Edge_Offset& an_offset: edge_offsets)
         {
            edges_to_offsets[an_offset.parent_edge] = &an_offset;
         }

         //Pull back roads
         for(edge* edge_p: i.edges_starting_here)
         {
            edge& e = (*edge_p);

            float offset = edges_to_offsets[edge_p]->offset;

            assert(!isnan(offset));

            double len_thus_far = 0;
            double _len         = 0;
            //Remove elements until the next segment's length is greater than offset - previous segments.
            int    new_start    = -1;
            do
            {
               //TODO could go infinite for tiny roads.
               len_thus_far                 += _len;
               new_start++;
               _len                          = planar_distance(e.shape[new_start + 1]->xy, e.shape[new_start]->xy);
#ifdef OSM_DEBUG
	       std::cout << len_thus_far << " " << offset << std::endl;
#endif
            }
            while (len_thus_far + _len<= offset); //TODO degenerate case when equal.

            //Update node degree count.
            for (int i = 1; i <= new_start; i++)
               node_degrees[e.shape[i]->id]--;

            //Modify geometry to make room for intersection.
            vec3f start_seg(e.shape[new_start + 1]->xy - e.shape[new_start]->xy);
            start_seg[2] = 0.0f;
            const double len = length(start_seg);
            double factor = (len - (offset - len_thus_far))/len;
            start_seg *= factor;

            e.shape[new_start]        = new node(*e.shape[new_start]);
            e.shape[new_start]->id    = e.shape[0]->id;
            e.shape[new_start]->xy[0] = e.shape[new_start + 1]->xy[0] - start_seg[0];
            e.shape[new_start]->xy[1] = e.shape[new_start + 1]->xy[1] - start_seg[1];
            e.shape[new_start]->xy[2] = intersection_node.xy[2];

            assert(!isnan(e.shape[new_start]->xy[0]));
            assert(!isnan(e.shape[new_start]->xy[1]));
            assert(!isnan(e.shape[new_start]->xy[2]));
            assert(new_start < e.shape.size());

            if (new_start != 0) {
#ifdef OSM_DEBUG
	       std::cout<<"Removing nodes no edge "<<e.id<<" for making room for intersections."<<std::endl;
#endif	       
               e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
	    }

            //Update the map to say that this edge has been offset.
            //Used for determining the max offset
            offset_once[edge_p->id] = true;

         }

         for(edge* edge_p: i.edges_ending_here)
         {
            edge& e = (*edge_p);

            float offset = edges_to_offsets[edge_p]->offset;

            double len_thus_far = 0;
            double _len         = 0;
            int    new_end      = e.shape.size();
            do
            {
               //TODO could go infinite for tiny roads.
               len_thus_far += _len;
               new_end--;
               _len          = planar_distance(e.shape[new_end]->xy, e.shape[new_end-1]->xy);
            }
            while(len_thus_far + _len <= offset);

            //Update node degree count
            //Don't change count for the last node, as we use its id.
            for (int i = new_end; i < static_cast<int>(e.shape.size()) - 1; i++)
               node_degrees[e.shape[i]->id]--;

            vec3f end_seg = e.shape[new_end]->xy;

            end_seg -=  e.shape[new_end - 1]->xy;

            const double len = planar_distance(e.shape[new_end]->xy, e.shape[new_end - 1]->xy);
            double factor = (len - (offset - len_thus_far))/len;

            end_seg                 *= factor;
            e.shape[new_end]         = new node(*e.shape[new_end]);
            e.shape[new_end]->id     = e.shape[e.shape.size() - 1]->id;
            e.shape[new_end]->xy[0]  = e.shape[new_end - 1]->xy[0] + end_seg[0];
            e.shape[new_end]->xy[1]  = e.shape[new_end - 1]->xy[1] + end_seg[1];
            e.shape[new_end]->xy[2]  = intersection_node.xy[2];

            assert(!isnan(e.shape[new_end]->xy[0]));
            assert(!isnan(e.shape[new_end]->xy[1]));
            assert(!isnan(e.shape[new_end]->xy[2]));

            if (new_end != static_cast<int>(e.shape.size()) - 1) {
#ifdef OSM_DEBUG	      
	       std::cout<<"Removing nodes no edge "<<e.id<<" for making room for intersections."<<std::endl;
#endif
               e.shape.erase(e.shape.begin() + new_end + 1, e.shape.end());
	    }

            //Update the map to store that this map has been visited once.
            //Used for determining the max offset
            offset_once[edge_p->id] = true;
         }
      }
   }
}



// ////  code from http://wiki.openstreetmap.org/wiki/Mercator#C_implementation
// double merc_x (double lon) {
//         return 6378137.0 * lon * (M_PI/180.0); // degree to radian
// }
//  
// double merc_y (double lat) {
//         double ratio = 6356752.3142 / 6378137.0;  // (R_MINOR/R_MAJOR)
//         double eccent = sqrt(1.0 - (ratio * ratio));
//         double com = 0.5 * eccent;
//   
//         lat = fmin (89.5, fmax (lat, -89.5));
//         double phi = lat * (M_PI/180.0);
//         double sinphi = sin(phi);
//         double con = eccent * sinphi;
//         con = pow((1.0 - con) / (1.0 + con), com);
//         double ts = tan(0.5 * (M_PI * 0.5 - phi)) / con;
//         return 0 - 6378137.0 * log(ts);
// }

void network::scale_and_translate()
{
   //// shift all the nodes
   float optional_offset_x = 0;
   float optional_offset_y = 0;

   bool first = true;
   vec2d bias;
   
   // Scale data to meters from
   // double survey_foot_to_meter = 1200.0/3937.0; //s. feet/meters
   double deg_to_rad = M_PI / 180.0;


   projPJ pj_merc, pj_latlong;
   //// Mercator projection with a Clarke 1866 ellipsoid and a 37.32° latitude of true scale and prints the projected cartesian values in meters     
   // if (!(pj_merc = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=37.5942 +units=m")) )
   if (!(pj_merc = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=37.32 +units=m")) )
      exit(1);
   if (!(pj_latlong = pj_init_plus("+proj=latlong +ellps=clrk66")) )
      exit(1);
   
    center[0] *= deg_to_rad;
    center[1] *= deg_to_rad;
   int p = pj_transform(pj_latlong, pj_merc, 1, 1, &center[0], &center[1], NULL );
   
   std::cout<<std::setprecision(9)<<"Projected Center(m): ctrlon="<<center[0]<<", ctrlat="<<center[1]<<std::endl;

   for(osm::node_pair &np: nodes)
   {
      np.second.xy[0] *= deg_to_rad;
      np.second.xy[1] *= deg_to_rad;
      double x = np.second.xy[0];
      double y = np.second.xy[1];
      int p = pj_transform(pj_latlong, pj_merc, 1, 1, &x, &y, NULL );
      
      np.second.xy[0] = x;
      np.second.xy[1] = y;
      np.second.xy[0] -= center[0];
      np.second.xy[1] -= center[1];
      np.second.xy[0] += optional_offset_x;
      np.second.xy[1] += optional_offset_y;
   }
}

void network::join(osm::edge* a, osm::edge* b)
{
   //Adds b to a
   assert(a->shape[0]->id == a->from);
   assert(a->shape[a->shape.size() - 1]->id == a->to);
   assert(b->shape[0]->id == b->from);
   assert(b->shape[b->shape.size() - 1]->id == b->to);
   assert(a->to == b->from);

   uint a_size = a->shape.size();
   uint b_size = b->shape.size();

   //Add all of b's nodes except the first one.
   for (int i = 1; i < static_cast<int>(b->shape.size()); i++)
   {
      b->shape[i]->edges_including.erase(find(b->shape[i]->edges_including.begin(), b->shape[i]->edges_including.end(), b));
      b->shape[i]->edges_including.push_back(a);
      a->shape.push_back(b->shape[i]);
   }

   a->to = b->to;

   assert(a->shape.size() == a_size + b_size - 1);
}

void edge::remove_duplicate_nodes()
{
   //// remove duplicate nodes in "shape" (vector<*node>) where stores ALL nodes for an edge
   if (shape.size() == 0) {
     std::cerr<<"No nodes in way "<<id<<", check the osm file!"<<std::endl;
     return;
   }
      
   std::vector<node*> new_node_list;
   str last_id = shape[0]->id;
   vec3f last_vec = shape[0]->xy;
   new_node_list.push_back(shape[0]);
   for(size_t i = 1; i < shape.size(); i++)
   {
      if ((shape[i]->id != last_id)
          && ((shape[i]->xy[0] != last_vec[0])
              ||
              (shape[i]->xy[1] != last_vec[1])))
      {
         new_node_list.push_back(shape[i]);
         last_id = shape[i]->id;
         last_vec = shape[i]->xy;
      }
   }
   shape.clear();
   for(size_t i = 0; i < new_node_list.size(); i++)
      shape.push_back(new_node_list[i]);
}

void edge::reverse()
{
   std::swap(from, to);
   std::reverse(shape.begin(), shape.end());
}

void network::join_logical_roads()
{
   compute_node_degrees();
   edges_including_rebuild();
   node_degrees_and_edges_agree();
   std::vector<str> edges_to_delete;

   for(osm::node_pair &np: nodes)
   {
      if (node_degrees[np.first] == 2)
      {
         assert(np.second.edges_including.size() < 3);
         if(np.second.edges_including.size() == 2)
         {
            edge* e = np.second.edges_including[0]; // the first edge that current node belongs to
            edge* o = np.second.edges_including[1]; // the second edge that current node belongs to

            if (o == e)
               continue;

            if (o->highway_class != e->highway_class)
               continue;

            if ((nodes[o->to].id == nodes[e->from].id) && (o->to == np.first))
            {
               std::swap(o, e);
            }

            // if the current node is the last node of its first belonging edge and the first node of its second belonging edge, join them
            if ((nodes[e->to].id == nodes[o->from].id) && (e->to == np.first))
            {
               node_degrees[e->to]--;

               int e_size = e->shape.size();
               int o_size = o->shape.size();
               join(e, o);
               assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);

               edges_to_delete.push_back(o->id);
               // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);

               // assert(np.second.edges_including.end() != ei_it);
               // np.second.edges_including.erase(ei_it);
            }
            else if ((nodes[e->to].id == nodes[o->to].id) && (e->to == np.first))
            {
               node_degrees[e->to]--;

               int e_size = e->shape.size();
               int o_size = o->shape.size();

               o->reverse();
               join(e, o); // update the shape of edge e by putting all nodes in o into e

               assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);
               // std::vector<edge>::iterator j_it = find(edges.begin(), edges.end(), *o);
               // assert(j_it != edges.end());
               // edges.erase(j_it);
               edges_to_delete.push_back(o->id);
               // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);
               // assert(np.second.edges_including.end() != ei_it);
               // np.second.edges_including.erase(ei_it);
            }
            else if ((nodes[e->from].id == nodes[o->from].id) && (e->from == np.first))
            {
               node_degrees[e->from]--;

               int e_size = e->shape.size();
               int o_size = o->shape.size();

               e->reverse();
               join(e, o);

               assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);
               // std::vector<edge>::iterator j_it = find(edges.begin(), edges.end(), *o);
               // assert(j_it != edges.end());
               // edges.erase(j_it);
               edges_to_delete.push_back(o->id);

               // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);
               // assert(np.second.edges_including.end() != ei_it);
               // np.second.edges_including.erase(ei_it);
            }
         }
      }
   }

   for(size_t i = 0; i < edges_to_delete.size(); i++)
   {
      for(size_t j = 0; j < edges.size(); j++)
      {
         if (edges[j].id == edges_to_delete[i])
         {
            edges.erase(edges.begin() + j);
            break;
         }
      }
   }
}

void network::display_used_node_heights()
{
   for(edge& e: edges)
   {
      for(node* n: e.shape)
      {
         if (n->xy[2] > 0)
            std::cout << n->id << " " << n->xy[2] << std::endl;
      }
   }
}

edge network::copy_edge_without_shape(const edge& e)
{
   edge tmpEdge;
   tmpEdge.type = e.type; // type includes id, nolanes, speed, priority, oneway

   //Initialized to values that must be changed.
   tmpEdge.from = "-1";
   tmpEdge.to   = "-1";
   tmpEdge.highway_class = e.highway_class;
   tmpEdge.id = std::to_string(network::new_edges_id); // new_edges_id is a static member of class network which starts from 0 
   network::new_edges_id++;
   
   return tmpEdge;
}


void network::split_into_road_segments()
{
   // Locate all split points (nodes with degree > 1)
   std::map<str, std::vector<int> > road_split_points;
   for(edge &ep: edges)
   {
      //for(int i=0; i<ep.shape.size(); i++) {	
	 //std::cout<<"asdf points: "<<ep.shape[i]->id<<" on edge "<<ep.id<<std::endl;	
      //}

      //Check nodes for split points, but skip the first and last, node_degrees > 1 meaning the nodes belongs to more than 1 edge
      for (int i = 1; i < static_cast<int>(ep.shape.size()) - 1; i++)
      {	 
         if (node_degrees[ep.shape[i]->id] > 1)
         {
#ifdef OSM_DEBUG
	    std::cout<<"Splitting points: "<<ep.shape[i]->id<<" on edge "<<ep.id<<std::endl;
#endif
            road_split_points[ep.id].push_back(i); // store all nodes that belongs to more than one edge
         }
      }
   }

   //Split each edge at its split points.
   std::vector<edge> new_edges;
   for(edge &_edge: edges)
   {
      int node_index        = 0;
      int _edge_ending_node = 0;

      //Find first splitter.
      bool _first = true;
      for(int split_index: road_split_points[_edge.id])
      {
	 // each split_index indicates some node that hasn't been addressed along the edge
	 // store the first node of the new edge
         if (!_first)
         {
            new_edges.push_back(copy_edge_without_shape(_edge));
            new_edges.back().from = _edge.shape[node_index]->id; // which is the same as previous edge.to
            new_edges.back().shape.push_back(&nodes[_edge.shape[node_index]->id]);
         }

         //Increase the node degree as the road is being split.
         node_degrees[_edge.shape[split_index]->id]++;

         //Add each node to new edge up to the split_index node also including the split_index node
         //Set the first node of the new edge
         while (node_index != split_index)
         {
            node_index++;
            if (!_first)
            {
               //Add nodes to new road
               new_edges.back().shape.push_back(&nodes[_edge.shape[node_index]->id]);
               new_edges.back().to = _edge.shape[node_index]->id;
            }
         }

         // Ensure the new edge has least two nodes (this is guaranteed to be true since when extracting split nodes we skip the first and last nodes on the original edge)
         if (!_first)
            assert(new_edges.back().shape.size() > 1);

         // Node index is at the index of the split point.
         if (_first)
         {
            //Update the node this road goes up to.
            _edge.to          = _edge.shape[node_index]->id;
            _edge_ending_node = node_index;
            _first            = false;
         }
      }

      
      //Now node_index is on the final splitter.
      if (road_split_points[_edge.id].size() > 0)
      {
         //Add that splitter and all remaining nodes to a new edge.
         new_edges.push_back(copy_edge_without_shape(_edge));
         new_edges.back().from = _edge.shape[node_index]->id;

         for (;node_index < static_cast<int>(_edge.shape.size()); node_index++)
         {
            new_edges.back().shape.push_back(_edge.shape[node_index]);
            new_edges.back().to = _edge.shape[node_index]->id;
         }

         assert((--new_edges.end())->shape.size() > 1);
         //Remove the deleted nodes from the original edge
         _edge.shape.erase(_edge.shape.begin()+ _edge_ending_node + 1, _edge.shape.end());
      }
      
      assert(_edge.shape.size() > 1);

   }

   //Add all created edges
   for(edge& e: new_edges)
   {
#ifdef OSM_DEBUG
      std::cout<<"Add a newly split edge id="<<e.id<<std::endl;
#endif
      edges.push_back(e);
   }
   
}

void network::compute_edge_types()
{
   // accroding to different types of road set speedlimit and number of lanes
   for(edge &e: edges)
   {
      edge_type* e_type = retrieve<edge_type>(types, e.id);
      e.type            = e_type;
      e_type->oneway    = e.oneway;
      
      
      if(e.highway_class.size() > 0) {
	e_type->speed = road_speeds[e.highway_class];
	e_type->nolanes = road_nolanes[e.highway_class];
	
	if(e_type->nolanes == 1 && e.oneway == 1) {
	   e_type->nolanes = 2;
	}
	
	//std::cout<<"Highway type="<<e.highway_class<<" speed="<<e.type->speed<<" oneway="<<e.oneway<<" nolanes="<<e.type->nolanes<<std::endl;
      }
      else { // in case something values are not specified in the config file.	
	e_type->speed     = 25;
	e_type->nolanes   = 1;
      }
      
      e_type->speed *= MIPH_TO_MEPS;
      
//       if(e.highway_class == "motorway")
//       {
//          e_type->speed   = 65;
//          e_type->nolanes = 5;
//          // e_type->oneway  = 1;
//       }
//       else if(e.highway_class == "trunk")
//       {
//          e_type->speed   = 50;
// 	 e_type->nolanes = 2;
//          if (e_type->oneway)
//             e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }
//       else if(e.highway_class == "primary")
//       {
//          e_type->speed   = 50;
//          e_type->nolanes = 2;
//          if (e_type->oneway)
//             e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }
//       else if(e.highway_class == "secondary")
//       {
//          e_type->speed   = 40;
//          e_type->nolanes = 1;
//          if (e_type->oneway)
//             e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }
//       else if(e.highway_class == "tertiary")
//       {
//          e_type->speed   = 40;
//          e_type->nolanes = 1;
//          if (e_type->oneway)
//             e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }
//       else if(e.highway_class == "motorway_link")
//       {
//          e_type->speed   = 30;
//          e_type->nolanes = 1;
//          // e_type->oneway  = 1;
//       }
//       else if(e.highway_class == "primary_link")
//       {
//          e_type->speed   = 30;
//          e_type->nolanes = 1;
// 
//          // e_type->oneway  = 1;
//       }
//       else if(e.highway_class == "secondary_link")
//       {
//          e_type->speed   = 30;
//          e_type->nolanes = 1;
//          // e_type->oneway  = 1;
//       }
//       else if(e.highway_class == "residential")
//       {
//          e_type->speed   = 30;
//          e_type->nolanes = 1;
//          if (e_type->oneway)
//             e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }
// 
//       else if(e.highway_class == "service")
//       {
//          e_type->speed   = 25;
//          e_type->nolanes = 1;
//          // e_type->oneway  = 0;
//       }
//       else if(e.highway_class == "urban")
//       {
//          //Classes added for grid
//          e_type->speed   = 30;
//          e_type->nolanes = 2;
//          // e_type->oneway  = 0;
//       }    
   }

   for(osm::edge &e: edges)
   {
      assert(e.type->nolanes > 0);
      assert(e.type->nolanes < 10);
   }
}


void network::export_map_for_GeoMatch()
{
   std::ofstream fout("tmp.map");
   for(osm::edge &e: edges)
   {
      for( int i = 0; i < e.shape.size() - 1; i++)
      {
         fout << std::setprecision(8) << e.id << " " << e.shape[i]->xy[0] << " " << e.shape[i]->xy[1] << " "<< e.shape[i+1]->xy[0] << " " << e.shape[i+1]->xy[1] << std::endl;
      }
   }

}
}

