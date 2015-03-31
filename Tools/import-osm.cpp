#include <osm_network.hpp>
#include <hwm_network.hpp>


void construct_osm_network(osm::network &onet, char* osmFileName, char* configFileName) { 
    onet = osm::load_xml_network(osmFileName, configFileName);
    onet.populate_edges_from_hash(); // load all edges	
    onet.remove_duplicate_nodes();
    onet.clip_roads_to_bounds(); // some edges will be removed due to shortage of in-bounds nodes
    onet.compute_edge_types(); // assign speedlimit and number of lanes to edges
    onet.compute_node_degrees();
    onet.scale_and_translate(); // the provided lat and lon values seems like in no use	
    onet.split_into_road_segments();		
    onet.remove_motorway_intersections();
    onet.join_logical_roads();
    onet.assign_highway_heights();			
    onet.remove_small_roads();	
    onet.join_logical_roads();
    onet.compute_node_heights();
    onet.create_ramps(); // only for motorway_link
    onet.create_intersections();
    onet.remove_small_roads_agree();
    onet.populate_edge_hash_from_edges();
}


int main(int argc, char *argv[])
{
    if(argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <input osm file> <imput config file> <output file>" << std::endl;
        return 1;
    }

    osm::network onet;
    construct_osm_network(onet, argv[1], argv[2]);
  
    hwm::network net(hwm::from_osm("test", 0.5f, onet));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.xml_write(argv[3]);

    try
    {
	net.check();
	std::cerr << "Road network checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
	std::cerr << "Road network doesn't check out: " << e.what() << std::endl;
	exit(1);
    }
    
     hwm::network_aux net_aux(net);
     net_aux.network_obj("test-obj.obj", 0.01);
     std::cerr << "Obj model exported successfully" << std::endl;
    
    return 0;
}


