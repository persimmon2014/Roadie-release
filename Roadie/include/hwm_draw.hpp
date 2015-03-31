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
