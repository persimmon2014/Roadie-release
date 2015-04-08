/*
 * hwm_draw.cpp
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

#include "hwm_draw.hpp"

static inline tvmet::XprVector<tvmet::VectorConstReference<float, 3>, 3> cvec3f(const float *mem)
{
    return tvmet::cvector_ref<float,3>(mem);
}

namespace hwm
{
    car_draw::car_draw() : v_vbo(0), n_vbo(0)
    {}

    bool car_draw::initialized() const
    {
        return v_vbo && n_vbo;
    }

    void car_draw::initialize(const float car_width,
                              const float car_length,
                              const float car_height,
                              const float car_rear_axle)
    {
        const float overts[][3] = {{-(car_length-car_rear_axle), -car_width/2, 0.0f},  //0
                                   {              car_rear_axle, -car_width/4, 0.0f},  //1
                                   {              car_rear_axle,  car_width/4, 0.0f},  //2
                                   {-(car_length-car_rear_axle),  car_width/2, 0.0f},  //3

                                   {-(car_length-car_rear_axle), -car_width/2,       car_height},  //4
                                   {              car_rear_axle, -car_width/4, car_height*13/15},  //5
                                   {              car_rear_axle,            0, car_height*13/15},  //6
                                   {-(car_length-car_rear_axle),            0,      car_height}};  //7

        const unsigned int ofaces[6][4] = {{ 7, 6, 5, 4}, // bottom
                                           { 0, 1, 2, 3}, // top
                                           { 1, 5, 4, 0}, // left side
                                           { 0, 3, 7, 4}, // back
                                           { 3, 7, 6, 2}, // right side
                                           { 5, 6, 2, 1}};// front

        std::vector<vec3f>        verts  (24);
        std::vector<vec3f>        normals(24);

        for(int i = 0; i < 6; ++i)
        {
            const vec3f d10 (cvec3f(overts[ofaces[i][1]]) - cvec3f(overts[ofaces[i][0]]));
            const vec3f d20 (cvec3f(overts[ofaces[i][2]]) - cvec3f(overts[ofaces[i][0]]));
            vec3f norm(tvmet::normalize(tvmet::cross(d10, d20)));

            for(int j = 0; j < 4; ++j)
            {
                verts[i*4+j]   = cvec3f(overts[ofaces[i][j]]);
                normals[i*4+j] = norm;
            }
        }

        glGenBuffers(1, &v_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glBufferData(GL_ARRAY_BUFFER, 24*3*sizeof(float), &(verts[0]), GL_STATIC_DRAW);

        glGenBuffers(1, &n_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, n_vbo);
        glBufferData(GL_ARRAY_BUFFER, 24*3*sizeof(float), &(normals[0]), GL_STATIC_DRAW);
        std::cout << glGetError() << std::endl;
        assert(glGetError() == GL_NO_ERROR);
    }

    void car_draw::draw() const
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, n_vbo);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, 0);

        assert(glGetError() == GL_NO_ERROR);
        glDrawArrays(GL_QUADS, 0, 24);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    car_draw::~car_draw()
    {
        if(v_vbo)
            glDeleteBuffers(1, &v_vbo);
        if(n_vbo)
            glDeleteBuffers(1, &n_vbo);
    }

    network_draw::network_draw() : v_vbo(0), f_vbo(0), net(0)
    {}

    bool network_draw::initialized() const
    {
        return net && v_vbo && f_vbo;
    }

    void network_draw::initialize(const network *in_net, const float resolution)
    {
        net = in_net;
        std::vector<vertex> points;
        std::vector<vec3u>  lane_faces;

        std::cout << "Generating network mesh with refinement " << resolution << "..."<<std::endl;

        for(const lane_pair &l: net->lanes)
        {
            lane_vert_starts.push_back(points.size());
            lane_face_starts.push_back(lane_faces.size());

            lane_data_map::iterator it = lanes.find(l.second.id);
            assert(it == lanes.end());

            lane_data ld;
            ld.vert_start = points.size();
            ld.face_start = lane_faces.size();

            l.second.make_mesh(points, lane_faces, net->lane_width, resolution);
            const float inv_len = 1.0f/l.second.length();
            for(size_t i = ld.vert_start; i < points.size(); ++i)
                points[i].tex_coord[0] *= inv_len;

            ld.vert_count = points.size()     -  ld.vert_start;
            ld.face_count = (lane_faces.size() - ld.face_start) * 3;
            ld.face_start *= sizeof(vec3u);
            lanes.insert(it, std::make_pair(l.second.id, ld));

            lane_vert_counts.push_back(points.size()-lane_vert_starts.back());
            lane_face_counts.push_back(lane_faces.size() -lane_face_starts.back());
	   
        }
        for(GLsizei &i: lane_face_counts)
        {
            i *= 3;
        }
        for(size_t &i: lane_face_starts)
        {
            i *= sizeof(vec3u);
        }

        for(const intersection_pair &i: net->intersections)
        {
	  
	   
            intersection_vert_fan_starts.push_back(points.size());

            points.push_back(vertex(i.second.center, vec3f(0.0f, 0.0f, 1.0f),
                                    vec2f(0.0f, 0.0f)));
	    
	    
            for(const vec3f &p: i.second.shape)
            {
                points.push_back(vertex(p, vec3f(0.0, 0.0, 1.0), vec2f(0.0f, 0.0f)));
            }
            points.push_back(vertex(i.second.shape.front(), vec3f(0.0, 0.0, 1.0), vec2f(0.0f, 0.0f)));

            intersection_vert_fan_counts.push_back(points.size()-intersection_vert_fan_starts.back());

            intersection_vert_loop_starts.push_back(intersection_vert_fan_starts.back() + 1);
            intersection_vert_loop_counts.push_back(intersection_vert_fan_counts.back() - 2);

            for(const intersection::state &st: i.second.states)
            {
                //const intersection::state::state_pair_in &ip = st.in_pair();
	      
                for(auto sp = st.state_pairs.begin(); sp != st.state_pairs.end(); ++sp)
                {
                    // Ugh, hack.
                    if (!sp->fict_lane)
                        continue;

                    assert(sp->fict_lane);
		    
		    
                    const lane &fict_lane = *(sp->fict_lane);
		     
		 
                    lane_data_map::iterator it = lanes.find(fict_lane.id);
                    assert(it == lanes.end()); // make sure that fict_lane is not an existing lane

                    lane_data fld;
                    fld.vert_start = points.size();
                    fld.face_start = lane_faces.size();
                    fict_lane.make_mesh(points, lane_faces, net->lane_width, resolution);
                    const float inv_len = 1.0f/fict_lane.length();
                    for(size_t i = fld.vert_start; i < points.size(); ++i)
                        points[i].tex_coord[0] *= inv_len;

                    fld.vert_count = points.size()     - fld.vert_start;
                    fld.face_count = (lane_faces.size() - fld.face_start) * 3;
                    fld.face_start *= sizeof(vec3u);
		    
                    lanes.insert(it, std::make_pair(fict_lane.id, fld));
                }
            }
        }
        
        std::cout << "Done" << std::endl;
       
        std::cout << "Sending " << points.size()*sizeof(vertex) << " bytes of vertex info to GPU...";
        std::cout.flush();
        glGenBuffers(1, &v_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glBufferData(GL_ARRAY_BUFFER, points.size()*sizeof(vertex), &(points[0]), GL_STATIC_DRAW);
        std::cout << "Done" << std::endl;

        std::cout << "Sending " << lane_faces.size()*sizeof(vec3i) << " bytes of index info to GPU...";
        std::cout.flush();
        glGenBuffers(1, &f_vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, lane_faces.size()*sizeof(vec3i), &(lane_faces[0]), GL_STATIC_DRAW);
        std::cout << "Done" << std::endl;

        std::cout << glGetError() << std::endl;
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_lanes_wire()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_LOOP, &(lane_vert_starts[0]), &(lane_vert_counts[0]), lane_vert_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_lanes_solid()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(2, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, tex_coord)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawElements(GL_TRIANGLES, &(lane_face_counts[0]), GL_UNSIGNED_INT, reinterpret_cast<const GLvoid**>(&(lane_face_starts[0])), lane_face_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void network_draw::draw_intersections_wire()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_LOOP, &(intersection_vert_loop_starts[0]), &(intersection_vert_loop_counts[0]), intersection_vert_loop_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_intersections_solid()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_TRIANGLE_FAN, &(intersection_vert_fan_starts[0]), &(intersection_vert_fan_counts[0]), intersection_vert_fan_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void network_draw::draw_fictitious_lanes_wire()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        for(const intersection_pair &i: net->intersections)
        {
            const intersection::state &st = i.second.states[i.second.current_state];
            //const intersection::state::state_pair_in &ip = st.in_pair();
// 	    for(const intersection::state &st: i.second.states)
//             {
	    
	      for(auto sp = st.state_pairs.begin(); sp != st.state_pairs.end(); ++sp)
	      {
		  assert(sp->fict_lane);
		  const lane &fict_lane = *(sp->fict_lane);
		  
		  lane_data_map::iterator it = lanes.find(fict_lane.id);
		  assert(it != lanes.end());

		  const lane_data &fld = it->second;
		  //std::cout<<fict_lane.id<<"-"<<fld.vert_count<<std::endl;
		  glDrawArrays(GL_LINE_LOOP, fld.vert_start, fld.vert_count);
	      }
	    //}
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_fictitious_lanes_solid()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(2, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, tex_coord)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);

        assert(glGetError() == GL_NO_ERROR);
        for(const intersection_pair &i: net->intersections)
        {
            const intersection::state &st = i.second.states[i.second.current_state];
            //const intersection::state::state_pair_in &ip = st.in_pair();
            for(auto sp = st.state_pairs.begin(); sp != st.state_pairs.end(); ++sp)
            {
                assert(sp->fict_lane);
                const lane &fict_lane = *(sp->fict_lane);

                lane_data_map::iterator it = lanes.find(fict_lane.id);
                assert(it != lanes.end());

                const lane_data &fld = it->second;
                assert(glGetError() == GL_NO_ERROR);
                glDrawElements(GL_TRIANGLES, fld.face_count, GL_UNSIGNED_INT, reinterpret_cast<const GLvoid*>(fld.face_start));
            }
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void network_draw::draw_lane_wire(const str &id)
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        lane_data_map::iterator it = lanes.find(id);
        assert(it != lanes.end());

        const lane_data &fld = it->second;
        assert(glGetError() == GL_NO_ERROR);
        glDrawArrays(GL_LINE_LOOP, fld.vert_start, fld.vert_count);

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    void network_draw::draw_lane_solid(const str &id)
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(2, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, tex_coord)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);

        assert(glGetError() == GL_NO_ERROR);
        lane_data_map::iterator it = lanes.find(id);
        assert(it != lanes.end());

        const lane_data &fld = it->second;
        assert(glGetError() == GL_NO_ERROR);
        glDrawElements(GL_TRIANGLES, fld.face_count, GL_UNSIGNED_INT, reinterpret_cast<const GLvoid*>(fld.face_start));

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    network_draw::~network_draw()
    {
        if(v_vbo)
            glDeleteBuffers(1, &v_vbo);
        if(f_vbo)
            glDeleteBuffers(1, &f_vbo);
    }

    network_aux_draw::network_aux_draw() : v_vbo(0), f_vbo(0), neta(0)
    {}

    bool network_aux_draw::initialized() const
    {
        return neta && v_vbo && f_vbo;
    }

    void network_aux_draw::initialize(const network_aux *in_neta, const road_metrics &rm, const float resolution)
    {
        neta = in_neta;
        std::vector<vertex> points;
        std::vector<vec3u>  lc_faces;

        std::cout << "Generating network aux mesh with refinement " << resolution << "..."<<std::endl;

        for(const strhash<network_aux::road_rev_map>::type::value_type &rrm_v: neta->rrm)
        {
            for(partition01<network_aux::road_rev_map::lane_cont>::const_iterator current = rrm_v.second.lane_map.begin();
                current != rrm_v.second.lane_map.end();
                ++current)
            {
                const network_aux::road_rev_map::lane_cont &e = current->second;

                if(e.empty())
                    continue;

                lane_maker lm(e.lane_tex(rm));
                lm.res_scale();
                const std::string lm_id(lm.make_string());
                std::map<std::string, material_group>::iterator cont_group(groups.find(lm_id));
                if(cont_group == groups.end())
                {
                    cont_group = groups.insert(cont_group, std::make_pair(lm_id, material_group()));
                    glGenTextures(1, &(cont_group->second.texture));
                    glBindTexture(GL_TEXTURE_2D, cont_group->second.texture);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

                    // std::cout << "Making new lane texture; res: " << lm.im_res << " scale: " << lm.scale << std::endl;
                    unsigned char *pix = new unsigned char[4*lm.im_res[0]*lm.im_res[1]];
                    lm.draw(pix);
                    for(size_t i = 0; i < lm.im_res[0]*lm.im_res[1]; ++i)
                    {
                        unsigned char temp[4];
                        memcpy(temp, pix + 4*i, sizeof(unsigned char)*4);
                        for(int j = 0; j < 3; ++j)
                            pix[4*i+j] = temp[2-j];
                    }

                    unsigned char *tmp = new unsigned char[4*lm.im_res[0]];
                    for(size_t i = 0; i < lm.im_res[1]/2; ++i)
                    {
                        memcpy(tmp, pix + 4*lm.im_res[0]*i, sizeof(unsigned char)*4*lm.im_res[0]);
                        memcpy(pix + 4*lm.im_res[0]*i, pix + 4*lm.im_res[0]*(lm.im_res[1]-1-i), sizeof(unsigned char)*4*lm.im_res[0]);
                        memcpy(pix + 4*lm.im_res[0]*(lm.im_res[1]-1-i), tmp, sizeof(unsigned char)*4*lm.im_res[0]);
                    }
                    delete[] tmp;

                    gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA8, lm.im_res[0], lm.im_res[1], GL_RGBA, GL_UNSIGNED_BYTE, pix);
                    delete[] pix;
                    glError();
                }

                lc_vert_starts.push_back(points.size());
                cont_group->second.lc_face_starts.push_back(lc_faces.size());

                size_t reverse_start;
                e.make_mesh(points, lc_faces, reverse_start, rrm_v.second.lane_map.containing_interval(current), neta->net.lane_width, resolution);
                const float inv_scale = 1.0/lm.scale[0];
                for(size_t i = lc_vert_starts.back(); i < points.size(); ++i)
                    points[i].tex_coord[0] *= inv_scale;

                lc_vert_counts.push_back(reverse_start   - lc_vert_starts.back());

                lc_vert_starts.push_back(reverse_start);
                lc_vert_counts.push_back(points.size()   - lc_vert_starts.back());

                cont_group->second.lc_face_counts.push_back(lc_faces.size() - cont_group->second.lc_face_starts.back());
            }
        }

        typedef std::pair<const std::string, material_group> mg_pair;
        for(mg_pair &mg: groups)
        {
            for(GLsizei &i: mg.second.lc_face_counts)
            {
                i *= 3;
            }
            for(size_t &i: mg.second.lc_face_starts)
            {
                i *= sizeof(vec3u);
            }
        }

        for(const strhash<network_aux::intersection_geometry>::type::value_type &ip: neta->intersection_geoms)
        {
            typedef std::pair<float, network_aux::point_tan> offs_pt;
            const network_aux::intersection_geometry &ig = ip.second;

            intersection_vert_fan_starts.push_back(points.size());

            points.push_back(vertex(ig.is->center, vec3f(0.0f, 0.0f, 1.0f), vec2f(0.0f, 0.0f)));
            for(size_t i = 0; i < ig.ric.size(); ++i)
            {
                const network_aux::road_store &rs = ig.ric[i];
                for(const offs_pt &op: rs.ipt)
                {
                    points.push_back(vertex(op.second.point, vec3f(0.0f, 0.0f, 1.0f), vec2f(0.0f, 0.0f)));
                }
                intersection_vert_strip_starts.push_back(points.size()-1);
                ig.connecting_arcs[i].extract_line(points, vec2f(0.0f, 1.0f), 0.0f, 0.01);
                intersection_vert_strip_counts.push_back(points.size()-intersection_vert_strip_starts.back());
            }
            points.push_back(points[intersection_vert_fan_starts.back() + 1]);

            intersection_vert_fan_counts.push_back(points.size()-intersection_vert_fan_starts.back());
        }
        std::cout << "Done" << std::endl;

        std::cout << "Sending " << points.size()*sizeof(vertex) << " bytes of vertex info to GPU...";
        std::cout.flush();
        glGenBuffers(1, &v_vbo);
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glBufferData(GL_ARRAY_BUFFER, points.size()*sizeof(vertex), &(points[0]), GL_STATIC_DRAW);
        std::cout << "Done" << std::endl;

        std::cout << "Sending " << lc_faces.size()*sizeof(vec3i) << " bytes of index info to GPU...";
        std::cout.flush();
        glGenBuffers(1, &f_vbo);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, lc_faces.size()*sizeof(vec3i), &(lc_faces[0]), GL_STATIC_DRAW);
        std::cout << "Done" << std::endl;

        assert(glGetError() == GL_NO_ERROR);
    }

    void network_aux_draw::draw_roads_wire()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_STRIP, &(lc_vert_starts[0]), &(lc_vert_counts[0]), lc_vert_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_aux_draw::draw_roads_solid()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(2, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, tex_coord)));

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_vbo);

        typedef std::pair<const std::string, material_group> mg_pair;
        for(mg_pair &mg: groups)
        {
            glBindTexture(GL_TEXTURE_2D, mg.second.texture);

            assert(glGetError() == GL_NO_ERROR);
            glMultiDrawElements(GL_TRIANGLES, &(mg.second.lc_face_counts[0]), GL_UNSIGNED_INT, reinterpret_cast<const GLvoid**>(&(mg.second.lc_face_starts[0])), mg.second.lc_face_starts.size());
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        glDisableClientState(GL_TEXTURE_COORD_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void network_aux_draw::draw_intersections_wire()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_STRIP, &(intersection_vert_strip_starts[0]), &(intersection_vert_strip_counts[0]), intersection_vert_strip_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_aux_draw::draw_intersections_solid()
    {
        glBindBuffer(GL_ARRAY_BUFFER, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, position)));

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(offsetof(vertex, normal)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_TRIANGLE_FAN, &(intersection_vert_fan_starts[0]), &(intersection_vert_fan_counts[0]), intersection_vert_fan_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

    network_aux_draw::~network_aux_draw()
    {
        if(v_vbo)
            glDeleteBuffers(1, &v_vbo);
        if(f_vbo)
            glDeleteBuffers(1, &f_vbo);
    }
}
