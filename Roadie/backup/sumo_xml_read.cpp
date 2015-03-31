#include "sumo_network.hpp"
#include "xml_util.hpp"

namespace sumo
{
static inline void read_shape(edge::shape_t &shape, const std::string &s)
{
  std::cout<<"Temp commented out, sumo_xml_read.cpp"<<std::endl;
//    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
//    boost::char_separator<char> sep(" ");
//    tokenizer                   tokens(s, sep);
// 
//    for(const std::string &vs: tokens)
//    {
//       boost::char_separator<char> csep(",");
//       tokenizer                   vtokens(vs, csep);
//       tokenizer::iterator         tok        = vtokens.begin();
// 
//       //double x = boost::lexical_cast<double>(*tok); 
//       double x = std::stod(std::string(*tok)); 
//       ++tok;
//       //double y = boost::lexical_cast<double>(*tok); 
//       double y = std::stod(std::string(*tok)); 
//       shape.push_back(vec2d(x, y));
//   }
}

node *anon_node(network &n, vec2d &pos)
{
   std::string id(std::string("anon_node" + std::to_string(n.anon_node_count++)));

   node no;
   no.id      = id;
   no.xy      = pos;
   no.type    = node::unknown;
   n.nodes[id] = no;

   return &(n.nodes[id]);
}

edge_type *anon_edge_type(network &n, const int priority, const int nolanes, const double speed)
{
   std::string id(std::string("anon_edge_type" + std::to_string(n.anon_edge_type_count++)));

   edge_type et;
   et.id       = id;
   et.priority = priority;
   et.nolanes  = nolanes;
   et.speed    = speed;
   n.types[id]   = et;

   return &(n.types[id]);
}

template <class T>
static inline T* retrieve(typename strhash<T>::type &m, const str &id)
{
   typedef typename strhash<T>::type val;
   typename strhash<T>::type::iterator entry(m.find(id));
   if(entry == m.end())
      m.insert(entry, std::make_pair(id, T()));

   return &(m[id]);
}

bool xml_read(network &n, node &no, xmlpp::TextReader &reader)
{
   get_attribute(no.id,    reader, "id");
   get_attribute(no.xy[0], reader, "x");
   get_attribute(no.xy[1], reader, "y");
   no.xy[2] = 0.0f;

   try
   {
      get_attribute(no.type, reader, "type");
   }
   catch(...)
   {
      // std::cerr << "WARNING: A node type could not be processed for "
      //           << no.id << std::endl;
      no.type = node::unknown;
   }

   return true;
}

static inline bool xml_read(network &n, edge_type &et, xmlpp::TextReader &reader)
{
   bool discard = false;
   if(get_attribute(discard, reader, "discard")) {
      if (discard == true)
         return false;
   }

   get_attribute(et.id,       reader, "id");
   get_attribute(et.priority, reader, "priority");
   get_attribute(et.nolanes,  reader, "numLanes");
   get_attribute(et.speed,    reader, "speed");

   return true;
}

static inline bool xml_read(network &n, edge &e, xmlpp::TextReader &reader)
{
   get_attribute(e.id, reader, "id");

   str from_id;
   str to_id;

   bool valid = true;

   try
   {
      get_attribute(from_id, reader, "from");
      get_attribute(to_id,   reader, "to");
      e.from = retrieve<node>(n.nodes, from_id);
      e.to   = retrieve<node>(n.nodes, to_id);
   }
   catch(missing_attribute &mi)
   {
      vec2d from;
      vec2d to;
      get_attribute(from[0], reader, "xfrom");
      get_attribute(from[1], reader, "yfrom");
      get_attribute(to[0],   reader, "xto");
      get_attribute(to[1],   reader, "yto");

      e.from = anon_node(n, from);
      e.to   = anon_node(n, to);
   }

   try
   {
      str type_id;
      get_attribute(type_id, reader, "type");
      if (n.types.find(type_id) == n.types.end()) {
         // std::cerr << "WARNING: Did not find edge type " << type_id
         //           << " for id " << e.id << std::endl;
         valid = false;
      }
      else
         e.type = retrieve<edge_type>(n.types, type_id);
   }
   catch(missing_attribute &mi)
   {
      int priority;
      int nolanes;
      double speed;
      get_attribute(priority, reader, "priority");
      get_attribute(nolanes, reader, "nolanes");
      get_attribute(speed, reader, "speed");

      e.type = anon_edge_type(n, priority, nolanes, speed);
   }

   try
   {
      get_attribute(e.spread, reader, "spreadType");
   }
   catch(missing_attribute &mi)
   {
      e.spread = edge::right;
   }

   str shape_str;
   try
   {
      get_attribute(shape_str, reader, "shape");
      read_shape(e.shape, shape_str);
   }
   catch(missing_attribute &mi)
   {
   }

   return valid;
}

static inline bool xml_read(network &n,
                            connection &c,
                            xmlpp::TextReader &reader)
{
   std::string from_id, to_id;
   get_attribute(from_id, reader, "from");

   // Tests to see if the edge reference exists.
   if (n.edges.find(from_id) == n.edges.end())
      return false;
   else
      c.from = retrieve<edge>(n.edges, from_id);

   get_attribute(to_id, reader, "to");

   // Tests to see if the edge reference exists.
   if (n.edges.find(to_id) == n.edges.end())
      return false;
   else
      c.to = retrieve<edge>(n.edges, to_id);

   get_attribute(c.fromLane, reader, "fromLane");
   get_attribute(c.toLane, reader, "toLane");

   return true;
}

static inline bool xml_read(network &n,
                            traffic_light &c,
                            xmlpp::TextReader &reader)
{
   std::string id;
   get_attribute(c.id, reader, "id");

   return true;
}

static inline bool xml_read_nodes(network &n, xmlpp::TextReader &reader)
{
   assert(is_opening_element(reader, "nodes"));
   read_skip_comment(reader);
   // Defined in xml_util.hpp
   sumo_read_map(n, n.nodes, reader, "node", "nodes");
   return true;
}

static inline bool xml_read_types(network &n, xmlpp::TextReader &reader)
{
   assert(is_opening_element(reader, "types"));
   read_skip_comment(reader);
   // Defined in xml_util.hpp
   sumo_read_map(n, n.types, reader, "type", "types");
   return true;
}


static inline bool xml_read_edges(network &n, xmlpp::TextReader &reader)
{
   assert(is_opening_element(reader, "edges"));
   read_skip_comment(reader);
   // Defined in xml_util.hpp
   sumo_read_map(n, n.edges, reader, "edge", "edges");
   return true;
}

static inline bool xml_read_connections(network &n,
                                        xmlpp::TextReader &reader)
{
   assert(is_opening_element(reader, "connections"));
   read_skip_comment(reader);
   // Defined in xml_util.hpp
   sumo_read_vector(n, n.connections, reader, "connection", "connections");
   return true;
}

inline void sumo_read_traffic_light(
                          network &n,
                          strhash<traffic_light>::type &themap,
                          vector<light_connection>& light_connections,
                          xmlpp::TextReader &reader,
                          const str &item_name,
                          const str &container_name) {

   traffic_light* current_light = 0;
   do
   {
      read_skip_comment(reader);

      if(reader.get_node_type() == xmlpp::TextReader::Element) {

         // Reads a phase, child of tlLogic.
         if(reader.get_name() == "phase") {
            // Gets the elements id.
            const str duration(reader.get_attribute("duration"));
            const str state(reader.get_attribute("state"));

            current_light->states.push_back(state);
            current_light->durations.push_back(atoi(duration.c_str()));
         }
         else if (reader.get_name() == "tlLogic") {

            // Gets the elements id.
            const str id(reader.get_attribute("id"));

            // Retrieves the element from the map or creates it if needed.
            auto vp = themap.find(id);
            if(vp == themap.end()) {
               auto new_pair =
               std::make_pair(id, traffic_light());
               vp = themap.insert(vp, new_pair);
            }

            // Sets the id of the new element.
            vp->second.id = vp->first;

            // In sumo_xml_read.cpp
            // Reads the attributes and buildes the element in the map.
            // Tests to see if the attribute was fully read. If not, removes it
            // from the map.
            if (!xml_read(n, themap[id], reader)) {
               themap.erase(vp);
            }

            current_light = &themap[id];
         }
         else if (reader.get_name() == "connection") {
            light_connection this_connection;
            this_connection.from = reader.get_attribute("from");
            this_connection.to = reader.get_attribute("to");
            this_connection.from_lane =
               atoi(reader.get_attribute("fromLane").c_str());
            this_connection.to_lane =
               atoi(reader.get_attribute("toLane").c_str());
            this_connection.traffic_light_id = reader.get_attribute("tl");
            this_connection.state_index =
               atoi(reader.get_attribute("linkIndex").c_str());

            light_connections.push_back(this_connection);
         }
         else {
	    std::cerr << std::string("Found stray [" + reader.get_name() + "] in " + container_name + " container search (expected " + item_name + ")") << std::endl;
         }
      }
   }
   while(!is_closing_element(reader, container_name));
}

static inline bool xml_read_traffic_lights(network &n,
                                           xmlpp::TextReader &reader)
{
   assert(is_opening_element(reader, "tlLogics"));
   read_skip_comment(reader);
   // Defined in xml_util.hpp
   sumo_read_traffic_light(n,
                           n.traffic_lights,
                           n.light_connections,
                           reader,
                           "tlLogic",
                           "tlLogics");
   return true;
}

static inline bool xml_read_nodes(network &n, const char *filename)
{
   xmlpp::TextReader reader(filename);
   read_skip_comment(reader);
   bool res = xml_read_nodes(n, reader);
   reader.close();
   return res;
}

static inline bool xml_read_types(network &n, const char *filename)
{
   xmlpp::TextReader reader(filename);
   read_skip_comment(reader);
   bool res = xml_read_types(n, reader);

   reader.close();
   return res;
}

static inline bool xml_read_edges(network &n, const char *filename)
{
   xmlpp::TextReader reader(filename);
   read_skip_comment(reader);
   bool res = xml_read_edges(n, reader);
   reader.close();
   return res;
}

static inline bool xml_read_connections(network &n, const char *filename)
{
   xmlpp::TextReader reader(filename);
   read_skip_comment(reader);
   bool res = xml_read_connections(n, reader);
   reader.close();
   return res;
}

static inline bool xml_read_traffic_lights(network &n, const char *filename)
{
   xmlpp::TextReader reader(filename);
   read_skip_comment(reader);
   bool res = xml_read_traffic_lights(n, reader);
   reader.close();
   return res;
}

network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file,
                         const char *connections_file,
                         const char *traffic_light_file)
{
   network n;

   n = load_xml_network(node_file, edge_type_file, edge_file);

   bool success = true;
   success = xml_read_connections(n, connections_file) and success;

   success = success
             and xml_read_traffic_lights(n, traffic_light_file) and success;

   if (!success)
      throw std::exception();

   return n;
}

network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file,
                         const char *connections_file)
{
   network n;

   n = load_xml_network(node_file, edge_type_file, edge_file);

   bool success = true;
   success = xml_read_connections(n, connections_file) and success;

   if (!success)
      throw std::exception();

   return n;
}


network load_xml_network(const char *node_file,
                         const char *edge_type_file,
                         const char *edge_file)
{
   network n;

   bool success = true;
   success = xml_read_nodes(n, node_file) and success;
   success = xml_read_types(n, edge_type_file) and success;
   success = xml_read_edges(n, edge_file) and success;

   if (!success)
      throw std::exception();

   return n;
}

}
