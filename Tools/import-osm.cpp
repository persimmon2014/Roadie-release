/*
 * import-osm.cpp
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


