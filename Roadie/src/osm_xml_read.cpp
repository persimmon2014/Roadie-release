#include "osm_network.hpp"
#include "xml_util.hpp"

#include <iostream>
#include <fstream>

#ifdef HAVE_PROJ4
#include <proj_api.h>
#endif


namespace osm
{
    //// read all nodes
    static inline void xml_read(network &n, node &no, xmlpp::TextReader &reader)
    {
        get_attribute(no.id,    reader, "id");
        get_attribute(no.xy[0], reader, "lon");
        get_attribute(no.xy[1], reader, "lat");
        no.xy[2] = 0.0f;
	
    }

    //// read ways
    static inline void xml_read(network &n, edge &e, xmlpp::TextReader &reader)
    {
        bool first_node = true;
        bool is_road = false;
        do
        {
            read_skip_comment(reader);

            if (reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if (reader.get_name() == "nd") // get a node
                {
                    str n_id;
                    node* current_node;
                    get_attribute(n_id, reader, "ref"); // get the node's id
                    current_node = retrieve<node>(n.nodes, n_id); // retrieve the node

                    e.shape.push_back(current_node); // push back to current edge's shape; shape - vector<*node>
                    current_node->edges_including.push_back(&e); // a node could be included in multiple edges

                    //Set the "to node" -- which is expected to be the last node --
                    //every time.  On the final iteration, the last node will be kept.
                    e.to = current_node->id;

                    //Save the first node as the "from" node
                    if (first_node)
                    {
                        first_node = false;
                        e.from = current_node->id;
                    }
                }

                if (reader.get_name() == "tag")
                {
                    str k, v;
                    get_attribute(k, reader, "k");
                    if (k == "highway")
		    {
		        is_road = false;
                        get_attribute(v, reader, "v");                     
			
			if(n.road_types.find(v) != n.road_types.end()) {
			  //std::cout<<n.roadtypes.size()<<std::endl;
			  is_road = true;
			}
			
// 			if (v=="motorway" or v=="trunk" or v=="primary" or v=="secondary" or v=="tertiary" or v == "unclassified" or v == "residential" /*or v=="motorway_link" or v=="trunk_link" or v=="primary_link"*/ )
//                             is_road = true;
                        
			//Temporary: only looking at highways.  Here is where roads can be disabled.
                        //if (v == "cycleway" or v == "footway" or v == "service" or v == "steps" or v=="dedicated footway" or v == "pedestrian" or v == "track" or v == "path" or v == "unclassified")
                            //is_road = false;

                        e.highway_class = v;
                    }
                    
                    if (k == "oneway"){
                        get_attribute(v, reader, "v");
                        if (v == "yes")
                            e.oneway = 1;
                        else
                            e.oneway = 0;
                    }

                }
            }
        }
        while(!is_closing_element(reader, "way"));

	
        if (!is_road)
            n.edge_hash.erase(e.id);
	
    }

    static inline void xml_read_nodes(network &n, xmlpp::TextReader &reader)
    {
        read_skip_comment(reader);
        read_map_no_container(n, n.nodes, reader, "node");
    }

    static inline void xml_read_edges(network &n, xmlpp::TextReader &reader)
    {
        read_skip_comment(reader);
        read_map_no_container(n, n.edge_hash, reader, "way");
    }

    static inline void xml_read_center(network &n, const char* filename)
    {
        xmlpp::TextReader reader(filename);
        do
        {
            try
            {
                read_skip_comment(reader);
            }
            catch(xml_eof &e)
            {
                break;
            }
            if (reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if (reader.get_name() == "bounds")
                {
                    float minlat, minlon, maxlat, maxlon;
                    get_attribute(minlat, reader, "minlat");
                    get_attribute(minlon, reader, "minlon");
                    get_attribute(maxlat, reader, "maxlat");
                    get_attribute(maxlon, reader, "maxlon");

                    n.center[0] = (maxlon - minlon)/2.0 + minlon;
                    n.center[1] = (maxlat - minlat)/2.0 + minlat;

		    //// latitude from left to right is large value to small value
		    //// longtitude from top to bottom is small value to large value
                    n.topleft[0] = minlon;
                    n.topleft[1] = maxlat;
                    n.bottomright[0] = maxlon;
                    n.bottomright[1] = minlat;
		    std::cout<<"Bounds: "<<"minlon="<<minlon<<", maxlon="<<maxlon<<", minlat="<<minlat<<", maxlat="<<maxlat<<std::endl;
                    std::cout<<"Center: "<<"ctrlon="<<n.center[0]<<", ctrlat="<<n.center[1]<<std::endl;
                }
            }
        }while(1);
    }

    static inline void xml_read_nodes(network &n, const char *filename)
    {
        xmlpp::TextReader reader(filename);
        xml_read_nodes(n, reader);

        reader.close();
    }

    static inline void xml_read_edges(network &n, const char *filename)
    {
        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);
        xml_read_edges(n, reader);

        reader.close();
    }
    
    
    void load_config_file(network &n, const char *config_file) {
      
      std::ifstream ifile(config_file);
	if (!ifile) {
	  std::cerr<<"No config osm file found!"<<std::endl;
	  exit(0);
	}
	
        xmlpp::TextReader reader(config_file);
	
        try
        {
            read_skip_comment(reader);
        }
        catch(xml_eof &e)
        {
             std::cerr<<"xml file reading failed!"<<std::endl;
	     exit(0);
        }
        
         if(!is_opening_element(reader, "config"))
            throw xml_error(reader, "The opening element should be \"config\".");

       
        get_attribute(n.lane_width, reader, "lane_width");
	if(n.lane_width <= 0.0f)
	  throw xml_error(reader, "Invalid lane width!");	
	std::cout<<"Config osm: lane width = "<<n.lane_width<<std::endl;
	
        get_attribute(n.road_remove_threshold, reader, "road_remove_threshold");
	if(n.road_remove_threshold < 0.0f)
	  throw xml_error(reader, "Invalid road remove threshold!");	
	std::cout<<"Config osm: road remove threshold = "<<n.road_remove_threshold<<std::endl;
	
        get_attribute(n.node_culling_threshold, reader, "node_culling_threshold");
	if(n.node_culling_threshold < 0.0f)
	  throw xml_error(reader, "Invalid node culling threshold!");	
	std::cout<<"Config osm: node culling threshold = "<<n.node_culling_threshold<<std::endl;
	    
	
        while(!is_closing_element(reader, "config"))
        {
            read_skip_comment(reader);
		  if(reader.get_node_type() == xmlpp::TextReader::Element)
		  {
		    if(reader.get_name() == "highway")
		    {
			std::string roadtype(reader.get_attribute("type"));
			if(roadtype.size() > 0) {
			  //std::cout<<roadtype<<std::endl;
			  n.road_types.insert(roadtype);
			}
			else {
			  std::cerr<<"Missing highway types in the config file!"<<std::endl;
			  exit(0);
			}
			  		
			std::string speedStr(reader.get_attribute("speed"));
			if(speedStr.size() > 0) {
			  float speed = std::stof(speedStr);
			  n.road_speeds.insert(std::make_pair(roadtype, speed));
			}
			else {
			  std::cerr<<"Missing speed value of certain types highway in the config file!"<<std::endl;
			}
			
			std::string nolanesStr(reader.get_attribute("lanes"));
			if(nolanesStr.size() > 0) {
			  int nolanes = std::stoi(nolanesStr);
			  n.road_nolanes.insert(std::make_pair(roadtype, nolanes));
			}  
			else {
			  std::cerr<<"Missing lanes value of certain types highway in the config file!"<<std::endl;
			}
			
		    
		    }
		  }
	}

        reader.close(); 
    }

    network load_xml_network(const char *osm_file, const char *config_file)
    {
        network n;
        load_config_file(n, config_file);
        
	std::cout<<"Start parsing osm file......"<<std::endl;
	
	//// Read "bounds" which contains unique longtitude and latitude for the current osm file. 
        xml_read_center(n, osm_file); 
	
	//// Read ALL nodes and store them in the internal node data structure (readed info: id, xy[0]=lon, xy[1]=lat, xy[2]=0).
        xml_read_nodes(n, osm_file); 
	
	//// Read ALL ways with certain types road (e.g. highway=motorway); storing all nodes, from node, to node, node-edge relationship and oneway
	xml_read_edges(n, osm_file);

        return n;
    }

    void calc_hwy_lengths(network& n, const char* filename)
    {
        std::cout << "Calculating road lengths.\n";

        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);

#ifdef HAVE_PROJ4
        projPJ pj_latlong, pj_merc;
        pj_latlong = pj_init_plus("+proj=latlong +ellps=clrk66");
        pj_merc = pj_init_plus("+proj=merc +ellps=clrk66 +lat_ts=37.5 +units=m");
#endif

        std::ofstream fout;
        fout.open("road_lengths.txt");

        unsigned long long ways = 0;
        float total_dist = 0;

        while(1)
        {
            read_skip_comment(reader);

            if(reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if(reader.get_name() == "way")
                {
                    ways++;

                    edge e;

                    const str id(reader.get_attribute("id"));

                    e.id = id;

                    bool first_node = true;
                    bool is_road = false;
                    do
                    {
                        read_skip_comment(reader);

                        if (reader.get_node_type() == xmlpp::TextReader::Element)
                        {
                            if (reader.get_name() == "nd")
                            {
                                str n_id;
                                node* current_node;
                                get_attribute(n_id, reader, "ref");
                                if (is_def<node>(n.nodes, n_id))
                                {
                                    current_node = retrieve<node>(n.nodes, n_id);

                                    e.shape.push_back(current_node);
                                }
                            }

                            if (reader.get_name() == "tag")
                            {
                                str k, v;
                                get_attribute(k, reader, "k");

                                if (k== "highway")
                                {
                                    get_attribute(v, reader, "v");

                                    if (v == "motorway")
                                    {
                                        is_road = true;
                                    }

                                    e.highway_class = v;
                                }
                            }
                        }
                    }
                    while(!is_closing_element(reader, "way"));

                    if ((is_road) and (e.shape.size() > 1))
                    {
                        double x,y;
                        double deg_to_rad = M_PI / 180.0;

#ifdef HAVE_PROJ4
                        x = e.shape[0]->xy[0] * deg_to_rad;
                        y = e.shape[0]->xy[1] * deg_to_rad;
                        pj_transform(pj_latlong, pj_merc, 1, 1, &x, &y, NULL);
#else
			x = merc_x(e.shape[0]->xy[0]);
			y = merc_y(e.shape[0]->xy[1]);		
#endif
                        float lx, ly;
                        lx = x;
                        ly = y;

                        float dist = 0;

                        // Project nodes.
                        for (int i = 1; i < e.shape.size(); i++)
                        {
#ifdef HAVE_PROJ4
			  x = e.shape[0]->xy[0] * deg_to_rad;
			  y = e.shape[0]->xy[1] * deg_to_rad;
			  pj_transform(pj_latlong, pj_merc, 1, 1, &x, &y, NULL);
#else
			  x = merc_x(e.shape[0]->xy[0]);
			  y = merc_y(e.shape[0]->xy[1]);		
#endif

                            dist += sqrt(pow(x - lx, 2) + pow(y - ly, 2));
                            lx = x;
                            ly = y;
                        }

                        total_dist += dist;

                        std::cout << dist << " is dist for road " << e.id << ".\n";
                        fout << dist << " is dist for road " << e.id << ".\n";
                        std::cout << total_dist << " is the dist thus far.\n";
                    }
                }
            }
        }
        std::cout << "End of ways.\n";
        reader.close();
    }


    void xml_read_some_nodes(network &n, const char* filename)
    {
        printf("Reading some nodes.\n");

        xmlpp::TextReader reader(filename);

        read_skip_comment(reader);

        unsigned int nodes = 0;

        while(1)
        {
            try
            {
                read_skip_comment(reader);
            }
            catch(xml_eof &e)
            {
                return;
            }

            if(reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if(reader.get_name() == "node")
                {
                    const str id(reader.get_attribute("id"));

                    node no;

                    get_attribute(no.id,    reader, "id");
                    get_attribute(no.xy[0], reader, "lon");
                    get_attribute(no.xy[1], reader, "lat");
                    no.xy[2] = 0.0f;

                    if (!out_of_bounds(no.xy))
                    {
                        strhash<node>::type::iterator vp(n.nodes.find(id));
                        if(vp == n.nodes.end())
                            vp = n.nodes.insert(vp, std::make_pair(id, strhash<node>::type::value_type::second_type()));
                        vp->second.id = vp->first;

                        n.nodes[id] = no;

                        nodes++;

                        if (nodes % 100 == 0)
                            printf("%i nodes read.\n", nodes);
                    }
                }
            }
        }
        reader.close();
    }


    void print_lengths(const char* osm_file)
    {
        printf("SANE\n");

        network n;

        //        xml_read_center(n, osm_file);

        xml_read_nodes(n, osm_file);
        //        xml_read_some_nodes(n, osm_file);
        calc_hwy_lengths(n, osm_file);
    }
}

