/*
 * hwm_xml_read.cpp
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
#include "xml_util.hpp"

void polyline_road::xml_read(xmlpp::TextReader &reader, const vec3f &scale)
{
    assert(is_opening_element(reader, "line_rep"));

    read_to_open(reader, "points");
    
    std::cout<<"abcd"<<std::endl;

    do
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Text ||
           reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
        {
	    //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	  	    std::string res(reader.get_value());
	    std::stringstream s1(res);
	    std::string line;
	    while(getline(s1, line, '\n')) {
	      
	      std::istringstream s2(line);
	      vec3f pos;
	      s2 >> pos[0];
	      s2 >> pos[1];
	      s2 >> pos[2];
	      
	      if((std::abs(pos[0] - 0) < 1e-6 && std::abs(pos[1] - 0) < 1e-6 && std::abs(pos[2] - 0) < 1e-6))
		continue;
	      
	      points_.push_back(vec3f(pos*scale));
	      
	    }
        }
    }
    while(!is_closing_element(reader, "points"));

    if(!initialize())
        throw xml_error(reader, "Failed to initialize polyine");

    read_to_close(reader, "line_rep");
}

void arc_road::xml_read(xmlpp::TextReader &reader, const vec3f &scale)
{
    assert(is_opening_element(reader, "arc_line_rep"));

    read_to_open(reader, "points");

    do
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Text ||
           reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
        {
	  
	    //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	    
	    std::string res(reader.get_value());
	    std::stringstream s1(res);
	    std::string line;
	    while(getline(s1, line, '\n')) {
	      
	      std::istringstream s2(line);
	      vec3f pos;
	      s2 >> pos[0];
	      s2 >> pos[1];
	      s2 >> pos[2];
	      
	      if((std::abs(pos[0] - 0) < 1e-6 && std::abs(pos[1] - 0) < 1e-6 && std::abs(pos[2] - 0) < 1e-6))
		continue;
	      
	      points_.push_back(vec3f(pos*scale));
	      
	    }
        }
    }
    while(!is_closing_element(reader, "points"));

    read_to_open(reader, "radii");

    while(!is_closing_element(reader, "radii"))
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Text ||
           reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
        {
	  
	    //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	    
	    std::string res(reader.get_value());
	    std::stringstream s1(res);
	    std::string line;
	    while(getline(s1, line, '\n')) {
	      
	      std::istringstream s2(line);
	      float rad;
	      s2 >> rad;
	      
	      //maybe also need to test this
// 	      if((std::abs(pos[0] - 0) < 1e-6 && std::abs(pos[1] - 0) < 1e-6 && std::abs(pos[2] - 0) < 1e-6))
// 		continue;
	      
	      radii_.push_back(rad*scale[0]);
	      
	    }
        }
    }

    if(!initialize_from_points_radii(points_, radii_))
        throw xml_error(reader, "Failed to initialize arc_road!");

    read_to_close(reader, "arc_line_rep");
}

void arc_road::xml_read_as_poly(xmlpp::TextReader &reader, const vec3f &scale)
{
  assert(is_opening_element(reader, "line_rep"));

    read_to_open(reader, "points");

    do
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Text ||
           reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
        {
	  
	  
	    //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	    
	    std::string res(reader.get_value());
	    std::stringstream s1(res);
	    std::string line;
	    while(getline(s1, line, '\n')) {
	      
	      std::istringstream s2(line);
	      vec3f pos;
	      s2 >> pos[0];
	      s2 >> pos[1];
	      s2 >> pos[2];
	      
	      if((std::abs(pos[0] - 0) < 1e-6 && std::abs(pos[1] - 0) < 1e-6 && std::abs(pos[2] - 0) < 1e-6))
		continue;
	      
	      points_.push_back(vec3f(pos*scale));
	      
	    }
        }
    }
    while(!is_closing_element(reader, "points"));

    if(!initialize_from_polyline(0.7f, points_))
        throw xml_error(reader, "Failed to initialize arc_road");

    read_to_close(reader, "line_rep");
}

template <class T>
template <class C>
void partition01<T>::xml_read(C &n, xmlpp::TextReader &reader, const str &tag)
{
    read_to_open(reader, "interval");

    if(is_closing_element(reader, "interval"))
        return;

    read_to_open(reader, "base");
    read_to_open(reader, tag);

    T elt0;
    elt0.xml_read(n, reader);

    read_to_close(reader, "base");
    while(!is_closing_element(reader, "interval"))
    {
        read_skip_comment(reader);

        if(is_opening_element(reader, "divider"))
        {
            float div;
            get_attribute(div, reader, "value");

            read_to_open(reader, tag);

            T elt;
            elt.xml_read(n, reader);

            insert(div, elt);

            read_to_close(reader, "divider");
        }
    }

    if(!empty() || !elt0.empty())
        insert(0.0f, elt0);
}


namespace hwm
{
    template <class T>
    static inline T* retrieve(typename strhash<T>::type &m, const str &id)
    {
        typedef typename strhash<T>::type val;
        typename strhash<T>::type::iterator entry(m.find(id));
        if(entry == m.end())
            m.insert(entry, std::make_pair(id, T()));

        return &(m[id]);
    }

    void intersection::state::xml_read(xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "state"));

        get_attribute(duration, reader, "duration");

        while(!is_closing_element(reader, "state"))
        {
            read_skip_comment(reader);
            if(is_opening_element(reader, "lane_pair"))
            {
                size_t in_id, out_id;
                get_attribute(in_id, reader, "in_id");
                get_attribute(out_id, reader, "out_id");
                //state_pairs.insert(state_pair(in_id, out_id));
		state_pairs.push_back(state_pair(in_id, out_id));

                read_to_close(reader, "lane_pair");
            }
        }
    }

    void lane::road_membership::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road_membership"));

        str ref;
        get_attribute(ref, reader, "parent_road_ref");
        get_attribute(interval[0], reader, "interval_start");
        get_attribute(interval[1], reader, "interval_end");
        get_attribute(lane_position, reader, "lane_position");

        parent_road = retrieve<road>(n.roads, ref);

        read_to_close(reader, "road_membership");
    }

    void lane::adjacency::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane_adjacency"));

        str ref;
        int acount = 0;

        if (get_attribute(ref, reader, "lane_ref"))
           ++acount;

        if (get_attribute(neighbor_interval[0], reader, "interval_start"))
           ++acount;

        if (get_attribute(neighbor_interval[1], reader, "interval_end"))
           ++acount;

        if(acount == 3)
            neighbor = retrieve<lane>(n.lanes, ref);
        else if(acount == 0)
            neighbor = 0;
        else
            throw xml_error(reader, "Lane adjacency missing attributes");

        read_to_close(reader, "lane_adjacency");
    }

    void lane::terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "dead_end"));
        read_to_close(reader, tag);
    }

    void lane::intersection_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "intersection_ref"));

        str ref;
        get_attribute(ref, reader, "ref");

        adjacent_intersection = retrieve<intersection>(n.intersections, ref);
        intersect_in_ref = -1;

        if(!adjacent_intersection->id.empty())
        {
            const std::vector<lane*> &cont(tag == "start" ? adjacent_intersection->outgoing : adjacent_intersection->incoming);
            size_t pos = 0;
            while(cont[pos] != parent)
            {
                if(pos >= cont.size()) {
		    throw xml_error(reader, std::string("Lane "+parent->id+" reports that it is incident on intersection "+adjacent_intersection->id+", but intersection does not")); 
		}
                ++pos;
            }
            intersect_in_ref = pos;
        }

        read_to_close(reader, tag);
    }

    void lane::lane_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        get_attribute(ref, reader, "ref");

        adjacent_lane = retrieve<lane>(n.lanes, ref);

        read_to_close(reader, tag);
    }

    void road::xml_read(const vec3f &scale, xmlpp::TextReader &reader)
    {
        // read road from xml-network
        assert(is_opening_element(reader, "road"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id) {
	    throw xml_error(reader, std::string("Id mismatch: had " +id + ", read " + read_id));
	}
        get_attribute(name, reader, "name");

        while(1)
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "line_rep"))
            {
                rep.xml_read_as_poly(reader, scale);
                break;
            }
            else if(is_opening_element(reader, "arc_line_rep"))
            {
                rep.xml_read(reader, scale);
                break;
            }
        }

        read_to_close(reader, "road");
    }

    inline static lane::terminus *terminus_helper(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, tag));

        lane::terminus *res = 0;

        while(!res)
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "dead_end"))
                res = new lane::terminus();
            else if(is_opening_element(reader, "intersection_ref"))
                res = new lane::intersection_terminus();
            else if(is_opening_element(reader, "lane_ref"))
                res = new lane::lane_terminus();
        }

        res->xml_read(n, parent, reader, tag);

        return res;
    }

    void lane::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id)
            throw xml_error(reader, "SNA?");//viboost::str(boost::format("Id mismatch: had %s, read %s") % id % read_id));

        get_attribute(speedlimit, reader, "speedlimit");

        active = true;

        bool have_start     = false;
        bool have_end       = false;
        bool have_road_int  = false;
        bool have_adjacency = false;
        while(!is_closing_element(reader, "lane"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "start"))
            {
                start = terminus_helper(n, this, reader, "start");
                have_start = true;
            }
            else if(is_opening_element(reader, "end"))
            {
                have_end = true;
                end = terminus_helper(n, this, reader, "end");
            }
            else if(is_opening_element(reader, "road_intervals"))
            {
                road_memberships.xml_read(n, reader, "road_membership");
                read_to_close(reader, "road_intervals");
                have_road_int = true;
            }
            else if(is_opening_element(reader, "adjacency_intervals"))
            {
                bool have_left  = false;
                bool have_right = false;
                while(!is_closing_element(reader, "adjacency_intervals"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "left"))
                    {
                        left.xml_read(n, reader, "lane_adjacency");
                        have_left = true;
                        read_to_close(reader, "left");
                    }
                    else if(is_opening_element(reader, "right"))
                    {
                        right.xml_read(n, reader, "lane_adjacency");
                        have_right = true;
                        read_to_close(reader, "right");
                    }
                }
                if(!have_left) {
		    throw xml_error(reader, std::string("No left adjacency in lane " +id + "!"));
		}
                if(!have_right) {
		    throw xml_error(reader, std::string("No right adjacency in lane " +id + "!"));
		}
                have_adjacency = true;
            }
        }

        if(!have_start) {
	    throw xml_error(reader, std::string("No start terminus in lane " +id + "!"));
	}
        if(!have_end) {
	    throw xml_error(reader, std::string("No end terminus in lane " +id + "!"));
	}
        if(!have_road_int) {
	    throw xml_error(reader, std::string("No road memberships in lane " +id + "!"));
	}
        if(!have_adjacency) {
	    throw xml_error(reader, std::string("No adjacencies in lane " +id + "!"));
	}
    }

    static inline void xml_incident_read(network &n, std::vector<lane*> &lv, xmlpp::TextReader &reader, bool incoming, const str &intersect_id)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        size_t loc;
        get_attribute(ref, reader, "ref");
        get_attribute(loc, reader, "local_id");

        if(lv.size() <= loc)
            lv.resize(loc+1);

        assert(lv.size() > loc);
        lv[loc] = retrieve<lane>(n.lanes, ref);

        if(!lv[loc]->id.empty())
        {
            lane::intersection_terminus *term = dynamic_cast<lane::intersection_terminus*>(incoming ? lv[loc]->end : lv[loc]->start);

            if(!term->adjacent_intersection || term->adjacent_intersection->id != intersect_id) {
		throw xml_error(reader, std::string("Intersection " +intersect_id+" reports that it is incident on lane " +lv[loc]->id+", but lane does not"));
	    }
            term->intersect_in_ref = loc;
        }

        read_to_close(reader, "lane_ref");
    }

    void intersection::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "intersection"));
        assert(id != str());
        str in_id;
        get_attribute(in_id, reader, "id");
        if(id != in_id) {
	    throw xml_error(reader, std::string("Id mismatch: had " + id+", read " + in_id));
	}

        read_to_open(reader, "incident");

        while(!is_closing_element(reader, "incident"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "incoming"))
            {
                while(!is_closing_element(reader, "incoming"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        xml_incident_read(n, incoming, reader, true, id);
                }
            }
            else if(is_opening_element(reader, "outgoing"))
            {
                while(!is_closing_element(reader, "outgoing"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        xml_incident_read(n, outgoing, reader, false, id);
                }
            }
        }

        read_to_open(reader, "states");

        while(!is_closing_element(reader, "states"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "state"))
            {
                size_t read_id;
                get_attribute(read_id, reader, "id");

                if(states.size() <= read_id)
                    states.resize(read_id+1);

                states[read_id].xml_read(reader);
            }
        }
    }

    network load_xml_network(const char *filename, const vec3f &scale)
    {
        network n;
	
	std::ifstream ifile(filename);
	if (!ifile) {
	  std::cerr<<"hwm-network xml file doesn't exist!"<<std::endl;
	  exit(0);
	}
	
        xmlpp::TextReader reader(filename);

        read_skip_comment(reader);

        if(!is_opening_element(reader, "network"))
            throw xml_error(reader, "No network element found!");

        str version;
        get_attribute(version, reader, "version");
        if(version != "1.3")
            throw xml_error(reader, "Invalid network xml version!");

        get_attribute(n.lane_width, reader, "lane_width");
        //std::cout<<"Got lane width: "<<n.lane_width<<std::endl;
	
	if(n.lane_width <= 0.0f)
            throw xml_error(reader, "Invalid lane_width!");

        get_attribute(n.name, reader, "name");

        get_attribute(n.gamma, reader, "gamma");
        if(n.gamma <= 0.0f ||
           n.gamma >= 1.0f)
            throw xml_error(reader, "Invalid gamma!");

	get_attribute(n.center_point[0], reader, "center_x");
	get_attribute(n.center_point[1], reader, "center_y");

        bool have_roads         = false;
        bool have_lanes         = false;
        bool have_intersections = false;
        while(!is_closing_element(reader, "network"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "roads"))
            {
                read_map(scale, n.roads, reader, "road", "roads");
		if(n.roads.size() < 1) {
		  std::cerr<<"Error: No roads exist, check the corresponding xml file!"<<std::endl;
		  exit(0);
		}
                have_roads = true;
            }
            else if(is_opening_element(reader, "lanes"))
            {
                read_map(n, n.lanes, reader, "lane", "lanes");
		if(n.lanes.size() < 1) {
		  std::cerr<<"Error: No lanes exist, check the corresponding xml file!"<<std::endl;
		  exit(0);
		}
                have_lanes = true;
            }
            else if(is_opening_element(reader, "intersections"))
            {
                read_map(n, n.intersections, reader, "intersection", "intersections");
                have_intersections = true;
            }
        }

        reader.close();

        return n;
    }
}

static obj_record xml_read_object(xmlpp::TextReader &reader)
{
    obj_record obj;

    get_attribute(obj.name, reader, "name");
    get_attribute(obj.mesh_name, reader, "mesh");

    while(!is_closing_element(reader, "object"))
    {
        read_skip_comment(reader);

        if(is_opening_element(reader, "matrix"))
        {
            while(!is_closing_element(reader, "matrix"))
            {
                int matrix_pos = 0;
                read_skip_comment(reader);

                if(reader.get_node_type() == xmlpp::TextReader::Text ||
                   reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
                {
		  
		  //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	  	  std::string res(reader.get_value());
		  std::stringstream s1(res);
		  std::string line;
		  while(getline(s1, line, '\n')) {
		    
		    std::istringstream s2(line);
		    while(!s2.eof()) {
		      
		      if(matrix_pos > 16)
			throw xml_error(reader, "Too many elements in matrix!");
                                
		      s2 >> obj.matrix.data()[matrix_pos++];
		      
		    }
		    
		  }
                }
            }
        }
        if(is_opening_element(reader, "bb_points"))
        {
            while(!is_closing_element(reader, "bb_points"))
            {
                read_skip_comment(reader);

                if(reader.get_node_type() == xmlpp::TextReader::Text ||
                   reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
                {
		  
		       //std::cout<<"BE CAREFUL, NEED TEST! (hwm_xml_read.cpp)"<<std::endl;
	  	       std::string res(reader.get_value());
		       std::stringstream s1(res);
		       std::string line;
		       while(getline(s1, line, '\n')) {
			 
			 std::istringstream s2(line);
			 vec3f pos;
			 s2 >> pos[0];
			 s2 >> pos[1];
			 s2 >> pos[2];
		 	
			 if((std::abs(pos[0] - 0) < 1e-6 && std::abs(pos[1] - 0) < 1e-6 && std::abs(pos[2] - 0) < 1e-6))
			   continue;
			
			 obj.box.enclose_point(pos[0], pos[1], pos[2]);
		       }
                }
            }
        }
    }

    return obj;
}

std::vector<obj_record> xml_read_scene(const std::string &filename)
{
    std::vector<obj_record> res;

    xmlpp::TextReader reader(filename);
    read_skip_comment(reader);

    if(!is_opening_element(reader, "scene"))
        throw xml_error(reader, "No scene element found!");

    while(!is_closing_element(reader, "scene"))
    {
        read_skip_comment(reader);

        if(is_opening_element(reader, "objects"))
        {
            while(!is_closing_element(reader, "objects"))
            {
                read_skip_comment(reader);
                if(is_opening_element(reader, "object"))
                    res.push_back(xml_read_object(reader));
            }
        }
    }
    reader.close();
    return res;
}