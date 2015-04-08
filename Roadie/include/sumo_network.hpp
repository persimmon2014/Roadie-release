/*
 * sumo_network.hpp
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

#ifndef _SUMO_NETWORK_HPP_
#define _SUMO_NETWORK_HPP_

#include "libroad_common.hpp"

#include <vector>

using std::vector;

namespace sumo
{
struct node
{
   typedef enum {priority, traffic_light, unknown, unregulated} TYPES;

   str   id;
   vec2d xy;
   TYPES type;
};

struct edge_type
{
   str    id;
   int    nolanes;
   double speed;
   int    priority;
   double length;
};

struct edge
{
   struct shape_t : public std::vector<vec2d>
   {
   };
   typedef enum {center, right} SPREAD;

   str        id;
   node*      from;
   node*      to;
   edge_type* type;
   shape_t    shape;
   SPREAD     spread;
};

struct connection
{
   edge* from;
   edge* to;
   int fromLane;
   int toLane;
};

struct traffic_light {
   int cycles;
   str id;
   vector<str> states;
   vector<int> durations;
};

struct light_connection {
   str from;
   str to;
   int from_lane;
   int to_lane;
   str traffic_light_id;
   int state_index;
};


struct network
{
   network() : anon_node_count(0), anon_edge_type_count(0)
   {}

   strhash<node>::type          nodes;
   size_t                       anon_node_count;
   strhash<edge_type>::type     types;
   size_t                       anon_edge_type_count;
   strhash<edge>::type          edges;
   std::vector<connection>      connections;
   strhash<traffic_light>::type traffic_lights;
   std::vector<light_connection> light_connections;

   bool check_edge(const edge &e) const;
   bool check_node(const node &n) const;
   bool check() const;
};

network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file);

network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file,
                         const char *connections_file);

network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file,
                         const char *connections_file,
                         const char *traffic_light_file);

}

inline std::ostream &operator<<(std::ostream &o, const sumo::node::TYPES &t)
{
   switch(t)
   {
   case sumo::node::priority:
      o << "priority";
      break;
   case sumo::node::traffic_light:
      o << "traffic_light";
      break;
   case sumo::node::unregulated:
      o << "unregulated";
      break;
   default:
      //      o << "unknown_node_type";
      throw std::exception();
      break;
   }
   return o;
}

inline std::istream &operator>>(std::istream &i, sumo::node::TYPES &t)
{
   str src;
   i >> src;

   if(src == "priority")
      t = sumo::node::priority;
   else if(src == "traffic_light")
      t = sumo::node::traffic_light;
   else if(src == "unregulated")
      t = sumo::node::unregulated;
   else
      t = sumo::node::unknown;

   return i;
}

inline std::ostream &operator<<(std::ostream &o, const sumo::edge::SPREAD &t)
{
   switch(t)
   {
   case sumo::edge::center:
      o << "center";
      break;
   case sumo::edge::right:
      o << "right";
      break;
   default:
      o << "unknown_spread_type";
      break;
   }
   return o;
}

inline std::istream &operator>>(std::istream &i, sumo::edge::SPREAD &t)
{
   str src;
   i >> src;

   if(src == "center")
      t = sumo::edge::center;
   else
      t = sumo::edge::right;

   return i;
}


#endif
