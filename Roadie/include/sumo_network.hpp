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
