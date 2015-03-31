#ifndef _OSM_NETWORK_HPP_
#define _OSM_NETWORK_HPP_

#include "libroad_common.hpp"
#include "arc_road.hpp"
#include <vector>
#include <set>


//#define OSM_DEBUG

namespace osm
{
  namespace create_ramps
  {
    struct _min_pair
    {
      float dist;
      float t;
    };
    _min_pair find_closest(vec3f, arc_road, float);
    _min_pair find_closest_recurse(vec3f, arc_road, float, float, _min_pair, float);
  }

    struct edge;

    struct node
    {
        node(){ is_overpass = false; ramp_merging_point = NULL;}

        str                id;
        vec3f              xy;
        std::vector<edge*> edges_including; //Currently not maintained
        bool               is_overpass;
        node*              ramp_merging_point;
    };

    struct edge_type
    {
        str    id;
        int    nolanes;
        double speed;
        int    priority;
        bool   oneway;
    };

    struct shape_t : public std::vector<node*>
    {
    };

    struct edge
    {
        ~edge(){}

        bool operator == (const edge& e) {return id == e.id;}

        typedef enum {center, right} SPREAD;

        str        id;
        str        from;
        str        to;
        edge_type* type;
        shape_t    shape;
        SPREAD     spread;

        //Poor architecture leads to repitition:
        str        highway_class; // indicate different types of the highway of osm, e.g. motorway, trunk, etc.
        bool       oneway;


        struct lane
        {
            lane(float s, float e, float off, str id, bool offr):start_t(s), end_t(e), offset(off), ramp_id(id), offramp(offr){}

            float start_t;
            float end_t;
            float offset;
            str   ramp_id;
            bool  offramp;
        };

        std::vector<node*> overpass_nodes;
        std::vector<lane>  additional_lanes;

        void remove_duplicate_nodes();
        void reverse();
        float length() const;
    };

    struct intersection
    {
        std::vector<edge*> edges_ending_here;
        std::vector<edge*> edges_starting_here;
        str                id_from_node;
    };

    struct network
    {
        strhash<node>::type              nodes;
        strhash<edge_type>::type         types;
        strhash<edge>::type              edge_hash;
        strhash<intersection>::type      intersections;
        strhash<edge>::type              road_segs;
        strhash<int>::type               node_degrees;
        strhash<std::vector<str> >::type node_connections;
        std::vector<edge>                edges;

        vec2d center;
        vec2d topleft;
        vec2d bottomright;
	
	static size_t new_edges_id;
	
	// variables for the configuration file
      	float lane_width;
	float road_remove_threshold;
	float node_culling_threshold;
	std::set<std::string> road_types;
	std::map<std::string, float> road_speeds;
	std::map<std::string, int> road_nolanes;
	
	
      	/**
         * Constructor
         */
        network() {}

        /**
         * Add a node into the network
	 * @param v node coordinate
	 * @param is_overpass is newly added node an overpass or not
	 * @return newly added node
         */
        node *add_node(const vec3f &v, const bool is_overpass);

	/**
         * Remove duplicate nodes on each edge
         */
        void remove_duplicate_nodes();
	
	/**
         * For each node on each edge, store the edge for the node
         */
        void edges_including_rebuild();

	/**
         * Remove any edge that contains less than 2 nodes after removing out of bounds nodes on that edge
         */
        void clip_roads_to_bounds();
	
	/**
         * Create ramps
         */
        void create_ramps();
	
	/**
         * Extract edges from the edge_hash structure
         */
        void populate_edges_from_hash();
	
	/**
         * Store edges in the edge_hash structure
         */
        void populate_edge_hash_from_edges();
        
	/**
         * Remove small roads according to the threshold specified in the configuration file, note the road length is the sum of piecewise road segments length
         */
	void remove_small_roads();
	
	/**
         * Check if two nearby nodes on a edge has the distance larger than nodes culling threshold specified in the configuration file
         */
	void remove_small_roads_agree();
	
	/**
         * Remove intersection nodes on motorway, identify whether it's a merging point or a overpass
         */
        void remove_motorway_intersections();
        
	/**
         * Assign highway heights
         */
	void assign_highway_heights();
        
	/**
         * Creates a w by h grid of nodes and connects them with roads of length dw and dh.
         */
	void create_grid(int, int, double, double);
        
	/**
         * Make translation from latitude and longtitude coordinates to cartesian coordinates
         */
	void scale_and_translate();
        
	/**
         * Compute node heights
         */
	void compute_node_heights();
        
	/**
         * Set parameters of roads according to their types
         */
	void compute_edge_types();
	
	/**
         * Draw the network
         */
        void draw_network() ;
	
	/**
         * Create intersections at starting and ending nodes of an edge which has degree more than 2
         */
        void create_intersections();
        
	/**
         * Compute node degrees
         */
	void compute_node_degrees();
        
	/**
         * Join logical roads
         */
	void join_logical_roads();
	
	/**
         * Split edges into road segments
         */
        void split_into_road_segments();
        
	/**
         * Join two edges
         */
	void join(edge* , edge*);
        
	/**
         * Copy edge without copying interior nodes
         */
	edge copy_edge_without_shape(const edge& e);
        
	/**
         * Export a map file for gis matching
         */
	void export_map_for_GeoMatch();

	/**
         * Print highway lengths
         */
        void print_highway_lengths();
	
	/**
         * Display node heights if its xy[2] > 0
         */
        void display_used_node_heights();
        
	/**
         * Print out all edges with their IDs
         */
	void list_edges();
	
	// Error checking functions
	void check_nodes();
	void check_edge(const edge &e) const;
	void check_node(const node &n) const;
	void check() const;
	void node_degrees_and_edges_agree();
	void intersection_check();
	void edges_check();
      
    };
  
    /**
      * Check if a point is out of bounds
      * @param vec3f checking point
      * @return true if the checking point is out of bounds otherwise return false
      */
    bool out_of_bounds(const vec3f &);
    
    /**
      * Load both the osm and configuration files
      * @param osm_file osm file name
      * @param config_file configuration file name
      * @return osm network
      */
    network load_xml_network(const char *osm_file, const char *config_file);
    
    /**
     * Print highway length from the osm file
     * @param osm_file osm file name
     */
    void print_lengths(const char* osm_file);
    
    /**
     * project longtitude coordinate to cartesian coordinate
     * @param lon longtitude
     * @return longtitude in cartesian coordinates
     */
    double merc_x (double lon);
    
    /**
     * project latitude coordinate to cartesian coordinate
     * @param lat latitude
     * @return latitude in cartesian coordinates
     */
    double merc_y (double lat);
}


inline std::ostream &operator<<(std::ostream &o, const osm::edge::SPREAD &t)
{
    switch(t)
    {
    case osm::edge::center:
        o << "center";
        break;
    case osm::edge::right:
        o << "right";
        break;
    default:
        o << "unknown_spread_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, osm::edge::SPREAD &t)
{
    str src;
    i >> src;

    if(src == "center")
        t = osm::edge::center;
    else
        t = osm::edge::right;

    return i;
}

template <class T>
static inline int is_def(typename strhash<T>::type &m, const str &id)
{
    typedef typename strhash<T>::type val;
    typename strhash<T>::type::iterator entry(m.find(id));
    if(entry == m.end())
        return 0;

    return 1;
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



#endif

