/*
 * hwm_draw.hpp
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

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
#include "hwm_network.hpp"

#define glError() { \
	GLenum err = glGetError(); \
	while (err != GL_NO_ERROR) { \
		fprintf(stderr, "glError: %s caught at %s:%u\n", (char *)gluErrorString(err), __FILE__, __LINE__); \
		err = glGetError(); \
	} \
}

namespace hwm
{
    struct car_draw
    {
        car_draw();

        bool initialized() const;

        void initialize(const float car_width,
                        const float car_length,
                        const float car_height,
                        const float car_rear_axle);

        void draw() const;

        ~car_draw();

        GLuint v_vbo;
        GLuint n_vbo;
    };

    struct network_draw
    {
        network_draw();

        bool initialized() const;
        void initialize(const hwm::network *net, const float resolution);

        void draw_lanes_wire();
        void draw_lanes_solid();

        void draw_intersections_wire();
        void draw_intersections_solid();

        void draw_fictitious_lanes_wire();
        void draw_fictitious_lanes_solid();

        void draw_lane_wire(const str &id);
        void draw_lane_solid(const str &id);

        ~network_draw();

        GLuint v_vbo;
        GLuint f_vbo;

        std::vector<GLint>    lane_vert_starts;
        std::vector<GLsizei>  lane_vert_counts;
        std::vector<size_t>   lane_face_starts;
        std::vector<GLsizei>  lane_face_counts;

        std::vector<GLint>    intersection_vert_fan_starts;
        std::vector<GLsizei>  intersection_vert_fan_counts;
        std::vector<GLint>    intersection_vert_loop_starts;
        std::vector<GLsizei>  intersection_vert_loop_counts;

        struct lane_data
        {
            GLint   vert_start;
            GLsizei vert_count;
            size_t  face_start;
            GLsizei face_count;
        };

        typedef strhash<lane_data>::type lane_data_map;
        lane_data_map                    lanes;

        const hwm::network *net;
    };

    struct network_aux_draw
    {
        struct material_group
        {
            GLuint texture;

            std::vector<size_t>   lc_face_starts;
            std::vector<GLsizei>  lc_face_counts;
        };

        network_aux_draw();

        bool initialized() const;
        void initialize(const hwm::network_aux *neta, const road_metrics &rm, const float resolution);

        void draw_roads_wire();
        void draw_roads_solid();

        void draw_intersections_wire();
        void draw_intersections_solid();

        ~network_aux_draw();

        GLuint v_vbo;
        GLuint f_vbo;

        std::vector<GLint>    lc_vert_starts;
        std::vector<GLsizei>  lc_vert_counts;

        std::map<std::string, material_group> groups;

        std::vector<GLint>    intersection_vert_fan_starts;
        std::vector<GLsizei>  intersection_vert_fan_counts;
        std::vector<GLint>    intersection_vert_strip_starts;
        std::vector<GLsizei>  intersection_vert_strip_counts;

        const hwm::network_aux *neta;
    };
}
