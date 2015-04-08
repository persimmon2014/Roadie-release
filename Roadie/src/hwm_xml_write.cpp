/*
 * hwm_xml_write.cpp
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
#include <fstream>

void polyline_road::xml_write(xmlpp::Element *elt) const
{
    xmlpp::Element *pr_elt = elt->add_child("line_rep");
    xmlpp::Element *pt_elt = pr_elt->add_child("points");

    for(const vec3f &pt: points_)
    {
	pt_elt->add_child_text(std::string(std::to_string(pt[0]) + " " + std::to_string(pt[1]) + " " + std::to_string(pt[2]) + " 0.0\n"));
    }
}

void arc_road::xml_write_as_poly(xmlpp::Element *elt) const
{
    xmlpp::Element *ar_elt = elt->add_child("line_rep");
    xmlpp::Element *pt_elt = ar_elt->add_child("points");

    for(const vec3f &pt: points_)
    {
	pt_elt->add_child_text(std::string(std::to_string(pt[0]) + " " + std::to_string(pt[1]) + " " + std::to_string(pt[2]) + " 0.0\n"));
    }
}

void arc_road::xml_write(xmlpp::Element *elt) const
{
    xmlpp::Element *ar_elt = elt->add_child("arc_line_rep");
    xmlpp::Element *pt_elt = ar_elt->add_child("points");

    for(const vec3f &pt: points_)
    {
	pt_elt->add_child_text(std::string(std::to_string(pt[0]) + " " + std::to_string(pt[1]) + " " + std::to_string(pt[2]) + " 0.0\n"));
    }

    xmlpp::Element *r_elt = ar_elt->add_child("radii");

    for(float pt: radii_)
    {
	r_elt->add_child_text(std::string(std::to_string(pt) + "\n"));
    }
}

void arc_road::svg_arc_arcs(const str &id, xmlpp::Element *parent) const
{
    if(points_.size() <= 2)
        return;

    xmlpp::Element *circle_group = parent->add_child("g");
    circle_group->set_attribute("id", std::string("id" + id + "_arcs"));

    for(size_t i = 1; i < points_.size()-1; ++i)
    {
        xmlpp::Element *circle = circle_group->add_child("path");
	circle->set_attribute("id", std::string("id" + id + "_arc_" + std::to_string(i-1)));

        circle->set_attribute("d",  svg_arc_arc_path(i));
    }
}

void arc_road::svg_arc_circles(const str &id, xmlpp::Element *parent) const
{
    if(points_.size() <= 2)
        return;

    xmlpp::Element *circle_group = parent->add_child("g");
    circle_group->set_attribute("id", std::string("id" + id + "_circles"));
    
    for(size_t i = 1; i < points_.size()-1; ++i)
    {
        const vec3f c(center(i));
        assert(xisfinite(c[0]));
        xmlpp::Element *circle = circle_group->add_child("circle");
        
	circle->set_attribute("id", std::string("id" + id + "_circles_" + std::to_string(i-1)));

        circle->set_attribute("cx", std::to_string(c[0]));
        circle->set_attribute("cy", std::to_string(c[1]));
        circle->set_attribute("r", std::to_string(radii_[i-1]));
    }
}

template <class T>
void partition01<T>::xml_write(xmlpp::Element *elt, const str &name) const
{
    xmlpp::Element *overall_elt = elt->add_child(name);
    xmlpp::Element *interval_elt = overall_elt->add_child("interval");

    typename partition01<T>::const_iterator pit = begin();
    if(!empty())
    {
        xmlpp::Element *base_elt = interval_elt->add_child("base");
        pit->second.xml_write(base_elt);
        for(++pit; pit != end(); ++pit)
        {
            xmlpp::Element *div_elt = interval_elt->add_child("divider");
            div_elt->set_attribute("value", std::to_string(pit->first));
            pit->second.xml_write(div_elt);
        }
    }
}

namespace hwm
{
    template <class T>
    static inline void xml_write_map(const T &v, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *map_elt = elt->add_child(name);

        typedef typename T::value_type val;
        for(const val &item: v)
        {
            item.second.xml_write(map_elt);
        }
    }

    void road::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *road_elt = elt->add_child("road");
        road_elt->set_attribute("id",   id);
        road_elt->set_attribute("name", name);
        rep.xml_write(road_elt);
    }

    void lane::terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        term_elt->add_child("dead_end");
    }

    void lane::intersection_terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        xmlpp::Element *iref = term_elt->add_child("intersection_ref");
        iref->set_attribute("ref", adjacent_intersection->id);
    }

    void lane::lane_terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        xmlpp::Element *iref = term_elt->add_child("lane_ref");
        iref->set_attribute("ref", adjacent_lane->id);
    }

    void lane::road_membership::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *rm_elt = elt->add_child("road_membership");
        if(!empty())
        {
            rm_elt->set_attribute("parent_road_ref", parent_road->id);
            rm_elt->set_attribute("interval_start", std::to_string(interval[0]));
            rm_elt->set_attribute("interval_end", std::to_string(interval[1]));
            rm_elt->set_attribute("lane_position", std::to_string(lane_position));
        }
    }

    void lane::adjacency::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *la_elt = elt->add_child("lane_adjacency");
        if(!empty())
        {
            la_elt->set_attribute("lane_ref",       neighbor->id);
            la_elt->set_attribute("interval_start", std::to_string(neighbor_interval[0]));
            la_elt->set_attribute("interval_end",   std::to_string(neighbor_interval[1]));
        }
    }

    void lane::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *lane_elt = elt->add_child("lane");
        lane_elt->set_attribute("id", id);
        lane_elt->set_attribute("speedlimit", std::to_string(speedlimit));
        start->xml_write(lane_elt, "start");
        end->xml_write(lane_elt, "end");
        road_memberships.xml_write(lane_elt, "road_intervals");
        xmlpp::Element *adj_elt = lane_elt->add_child("adjacency_intervals");
        left. xml_write(adj_elt, "left");
        right.xml_write(adj_elt, "right");
    }

    template <class T>
    static inline void xml_write_vector(const std::vector<T> &v, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *list_elt = elt->add_child(name);
        size_t count = 0;
        for(const T &item: v)
        {
            xml_write(item, count, list_elt);
            count++;
        }
    }

    static inline void xml_write(const lane *lr, const size_t count, xmlpp::Element *elt)
    {
        xmlpp::Element *lr_elt = elt->add_child("lane_ref");
        lr_elt->set_attribute("ref", lr->id);
        lr_elt->set_attribute("local_id", std::to_string(count));
    }

    static inline void xml_write(const intersection::state &is, const size_t count, xmlpp::Element *elt)
    {
        xmlpp::Element *is_elt = elt->add_child("state");
        is_elt->set_attribute("id", std::to_string(count));
        is_elt->set_attribute("duration", std::to_string(is.duration));

        //const intersection::state::state_pair_in &ip = is.in_pair();
        for(auto sp = is.state_pairs.begin(); sp != is.state_pairs.end(); ++sp)
        {
            xmlpp::Element *ii_elt = is_elt->add_child("lane_pair");
            ii_elt->set_attribute("in_id",  std::to_string(sp->in_idx));
            ii_elt->set_attribute("out_id", std::to_string(sp->out_idx));
        }
    }

    void intersection::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *intersection_elt = elt->add_child("intersection");
        xmlpp::Element *incident_elt     = intersection_elt->add_child("incident");

        intersection_elt->set_attribute("id", id);
        xml_write_vector(incoming, incident_elt, "incoming");
        xml_write_vector(outgoing, incident_elt, "outgoing");
        xml_write_vector(states, intersection_elt, "states");
    }

    void network::xml_write(const char *filename) const
    {
        xmlpp::Document out;

        xmlpp::Element *root(out.create_root_node("network"));
        xml_write(root);

        //std::ostream *out_stream = compressing_ostream(filename);
	//out.write_to_stream_formatted(*out_stream, "utf-8");
	//delete out_stream;
	
	std::filebuf fb;
	fb.open (filename, std::ios::out);
	std::ostream out_stream(&fb);
        out.write_to_stream_formatted(out_stream, "utf-8");
	out_stream.flush();
    }

    void network::xml_write(xmlpp::Element *elt) const
    {
        elt->set_attribute("name",    name);
        elt->set_attribute("version", "1.3");
        elt->set_attribute("gamma", std::to_string(gamma));
        elt->set_attribute("lane_width", std::to_string(lane_width));
        elt->set_attribute("center_x", std::to_string(center_point[0]));
        elt->set_attribute("center_y", std::to_string(center_point[1]));
        xml_write_map(roads,         elt, "roads");
        xml_write_map(lanes,         elt, "lanes");
        xml_write_map(intersections, elt, "intersections");
    }

    void network::svg_write(const char *filename, const int flags) const
    {
        xmlpp::Document out;
        out.set_internal_subset("svg", "-//W3C//DTD SVG 1.1//EN", "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd");
        xmlpp::Element *root(out.create_root_node("svg", "http://www.w3.org/2000/svg"));

        vec3f l(FLT_MAX);
        vec3f h(-FLT_MAX);
        bounding_box(l, h);
        const float wi = h[0] - l[0];
        const float hi = h[1] - l[1];
        float width;
        float height;
        if(wi > hi)
        {
            width = 1;
            height = hi/wi;
        }
        else
        {
            height = 1;
            width = wi/hi;
        }
        float image_scale = 500;

	root->set_attribute("width", std::string(std::to_string(image_scale*width) + "px"));
	root->set_attribute("height", std::string(std::to_string(image_scale*height) + "px"));
	
        root->set_attribute("version", "1.1");

        xmlpp::Element *title = root->add_child("title");
	title->add_child_text(std::string(name + " network"));
	
        xmlpp::Element *desc  = root->add_child("desc");
	desc->add_child_text(std::string(name + " network"));

        xmlpp::Element *flipgroup = root->add_child("g");
	flipgroup->set_attribute("transform", std::string("scale(" + std::to_string(image_scale*width/wi) + "," +std::to_string(image_scale*height/hi) + ") translate(" +std::to_string(h[0]) + "," +std::to_string(h[1]) +") scale(1, -1)")); 

        if(flags & SVG_ROADS)
        {
            xmlpp::Element *roadgroup = flipgroup->add_child("g");
            roadgroup->set_attribute("id", "roads");

            xmlpp::Element *arcgroup = roadgroup->add_child("g");
            arcgroup->set_attribute("id", "arc_roads");
            arcgroup->set_attribute("fill", "none");
            arcgroup->set_attribute("stroke", "black");
            arcgroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *polygroup = roadgroup->add_child("g");
            polygroup->set_attribute("id", "poly_roads");
            polygroup->set_attribute("fill", "none");
            polygroup->set_attribute("stroke", "black");
            polygroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *arc_arcgroup, *arc_circlegroup;
            if(flags & SVG_ARCS)
            {
                arc_arcgroup = arcgroup->add_child("g");
                arc_arcgroup->set_attribute("id", "arcs");
                arc_arcgroup->set_attribute("fill", "none");
                arc_arcgroup->set_attribute("opacity", "1.0");
                arc_arcgroup->set_attribute("stroke", "blue");
            }
            if(flags & SVG_CIRCLES)
            {
                arc_circlegroup = arcgroup->add_child("g");
                arc_circlegroup->set_attribute("id", "circles");
                arc_circlegroup->set_attribute("fill", "none");
                arc_circlegroup->set_attribute("opacity", "1.0");
                arc_circlegroup->set_attribute("stroke", "red");
            }

            for(const road_pair &rp: roads)
            {
                {
                    xmlpp::Element *path = arcgroup->add_child("path");
                    path->set_attribute("d", rp.second.rep.svg_arc_path(vec2f(0.0f, 1.0f), 0.0).stringify());
		    path->set_attribute("id", std::string("id" + rp.first + "_arc"));
                }
                if(flags & SVG_ARCS)
                    rp.second.rep.svg_arc_arcs(rp.first, arc_arcgroup);
                if(flags & SVG_CIRCLES)
                    rp.second.rep.svg_arc_circles(rp.first, arc_circlegroup);
                {
                    xmlpp::Element *path = polygroup->add_child("path");
                    path->set_attribute("d", rp.second.rep.svg_poly_path(vec2f(0.0f, 1.0f), 0.0).stringify());
		    path->set_attribute("id", std::string("id" + rp.first + "_poly"));
                }
            }

            std::tr1::unordered_map<const str, bool, hash<const str> > fict_road_map;
            for(const intersection_pair &ip: intersections)
            {
                for(const intersection::state &s: ip.second.states)
                {
                    //intersection::state::state_pair_in::iterator current = s.in_pair().begin();
                    for(auto current = s.state_pairs.begin(); current != s.state_pairs.end(); ++current)
                    {
                        const intersection::state::state_pair &sp = *current;

                        const lane *in_lane  = ip.second.incoming[sp.in_idx];
                        const lane *out_lane = ip.second.outgoing[sp.out_idx];

			const str id(std::string("id" + in_lane->id + "_to_" + out_lane->id));

                        std::tr1::unordered_map<const str, bool, hash<const str> >::const_iterator f = fict_road_map.find(id);
                        if(f != fict_road_map.end())
                            continue;

                        fict_road_map.insert(std::make_pair(id, true));

                        const arc_road &in_road     = in_lane->road_memberships[1.0]->second.parent_road->rep;
                        const float     in_param    = in_lane->road_memberships[1.0]->second.interval[1];
                        const bool      in_reverse  = in_lane->road_memberships[1.0]->second.interval[0] > in_lane->road_memberships[1.0]->second.interval[1];

                        const arc_road &out_road    = out_lane->road_memberships[0.0]->second.parent_road->rep;
                        const float     out_param   = out_lane->road_memberships[0.0]->second.interval[0];
                        const bool      out_reverse = out_lane->road_memberships[0.0]->second.interval[0] > out_lane->road_memberships[0.0]->second.interval[1];

                        vec3f start_point;
                        vec3f start_tan;
                        vec3f end_point;
                        vec3f end_tan;
                        {
                            const mat4x4f start(in_road. point_frame(in_param,  0,  in_reverse));
                            const mat4x4f end  (out_road.point_frame(out_param, 0, out_reverse));
                            for(int i = 0; i < 3; ++i)
                            {
                                start_point[i] = start(i, 3);
                                start_tan[i]   = start(i, 0);
                                end_point[i]   = end(i, 3);
                                end_tan[i]     = -end(i, 0);
                            }
                        }
                        arc_road ar;
                        ar.initialize_from_polyline(0.0f, from_tan_pairs(start_point,
                                                                         start_tan,
                                                                         end_point,
                                                                         end_tan,
                                                                         2.0));
                        {
                            xmlpp::Element *path = arcgroup->add_child("path");
                            path->set_attribute("d", ar.svg_arc_path(vec2f(0.0f, 1.0f), 0.0).stringify()+"Z");
			    path->set_attribute("id", std::string(id + "_arc"));
                        }
                        if(flags & SVG_ARCS)
                            ar.svg_arc_arcs(id, arc_arcgroup);
                        if(flags & SVG_CIRCLES)
                            ar.svg_arc_circles(id, arc_circlegroup);
                        {
                            xmlpp::Element *path = polygroup->add_child("path");
                            path->set_attribute("d", ar.svg_poly_path(vec2f(0.0f, 1.0f), 0.0).stringify()+"Z");
			    path->set_attribute("id", std::string(id + "_poly"));
                        }
                    }
                }
            }
        }

        if(flags & SVG_LANES)
        {
            xmlpp::Element *lanegroup = flipgroup->add_child("g");
            lanegroup->set_attribute("id", "lanes");

            xmlpp::Element *arcgroup = lanegroup->add_child("g");
            arcgroup->set_attribute("id", "arc_lanes");
            arcgroup->set_attribute("fill", "none");
            arcgroup->set_attribute("stroke", "black");
            arcgroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *polygroup = lanegroup->add_child("g");
            polygroup->set_attribute("id", "poly_lanes");
            polygroup->set_attribute("fill", "none");
            polygroup->set_attribute("stroke", "black");
            polygroup->set_attribute("stroke-width", "0.5");

            for(const lane_pair &lp: lanes)
            {
                {
                    xmlpp::Element *path = arcgroup->add_child("path");
                    path->set_attribute("d", lp.second.svg_arc_path(lane_width).stringify()+"Z");
		    path->set_attribute("id", std::string("id" + lp.first + "_arc"));
                }
                {
                    xmlpp::Element *path = polygroup->add_child("path");
                    path->set_attribute("d", lp.second.svg_poly_path(lane_width).stringify()+"Z");
		    path->set_attribute("id", std::string("id" + lp.first + "_poly"));
                }
            }

            for(const intersection_pair &ip: intersections)
            {
                for(const intersection::state &s: ip.second.states)
                {
                    for(const lane_pair &lp: s.fict_lanes)
                    {
                        {
                            xmlpp::Element *path = arcgroup->add_child("path");
                            path->set_attribute("d", lp.second.svg_arc_path(lane_width).stringify()+"Z");
			    path->set_attribute("id", std::string("id" + lp.first + "_arc"));
                        }
                        {
                            xmlpp::Element *path = polygroup->add_child("path");
                            path->set_attribute("d", lp.second.svg_poly_path(lane_width).stringify()+"Z");
			    path->set_attribute("id", std::string("id" + lp.first + "_poly"));
                        }
                    }
                }
            }
        }

//         std::ostream *out_stream = compressing_ostream(filename);
//         out.write_to_stream_formatted(*out_stream, "utf-8");
//         delete out_stream;
	
	std::filebuf fb;
	fb.open (filename, std::ios::out);
	std::ostream out_stream(&fb);
        out.write_to_stream_formatted(out_stream, "utf-8");
	out_stream.flush();
    }
}

void xml_write_scene(const std::string &filename, const std::vector<obj_record> &objs)
{
    xmlpp::Document out;
    xmlpp::Element *root(out.create_root_node("scene"));

    xmlpp::Element *objects = root->add_child("objects");
    for(const obj_record &obj: objs)
    {
        xmlpp::Element *object = objects->add_child("object");
        object->set_attribute("name", obj.name);
        object->set_attribute("mesh", obj.mesh_name);

        xmlpp::Element *mat = object->add_child("matrix");
	
	mat->add_child_text(std::string(std::to_string(obj.matrix(0,0))+ " " +std::to_string(obj.matrix(0,1))+ " " +std::to_string(obj.matrix(0,2))+ " " +std::to_string(obj.matrix(0,3))+ " " +
					std::to_string(obj.matrix(1,0))+ " " +std::to_string(obj.matrix(1,1))+ " " +std::to_string(obj.matrix(1,2))+ " " +std::to_string(obj.matrix(1,3))+ " " +
					std::to_string(obj.matrix(2,0))+ " " +std::to_string(obj.matrix(2,1))+ " " +std::to_string(obj.matrix(2,2))+ " " +std::to_string(obj.matrix(2,3))+ " " +
					std::to_string(obj.matrix(3,0))+ " " +std::to_string(obj.matrix(3,1))+ " " +std::to_string(obj.matrix(3,2))+ " " +std::to_string(obj.matrix(3,3))));
    }
    
//     std::cout<<"xml_write_scene, hwm_xml_write.cpp"<<filename<<std::endl;
//     std::ostream *out_stream = compressing_ostream(filename);
//     out.write_to_stream_formatted(*out_stream, "utf-8");
//     delete out_stream;
    
    std::filebuf fb;
    fb.open (filename, std::ios::out);
    std::ostream out_stream(&fb);
    out.write_to_stream_formatted(out_stream, "utf-8");
    out_stream.flush();
}
