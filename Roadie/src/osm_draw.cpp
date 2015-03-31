#include "osm_network.hpp"
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

static bool               first_for_display = 1;
static std::vector<vec3f> colors;

namespace osm
{
    void network::draw_network()
    {
        glColor3f(0,0,0);

        int i = 0;
        if (first_for_display)
        {
            for(size_t i = 0; i < edges.size(); ++i)
            {
                colors.push_back(vec3f(drand48(), drand48(), drand48()));
            }
        }

        for(edge &e: edges)
        {
            const shape_t& e_nodes = e.shape;

            glBegin(GL_LINE_STRIP);
            glColor3fv(colors[i].data());

            for(int j = 0; j < static_cast<int>(e_nodes.size()); j++)
            {
                glVertex3fv(e_nodes[j]->xy.data());
            }
            glEnd();

            i++;
        }

        glPointSize(5);
        glColor3f(1,0,0);
        glBegin(GL_POINTS);
        for(strhash<intersection>::type::value_type& i: intersections)
        {
            glVertex3fv(nodes[i.second.id_from_node].xy.data());
        }
        glEnd();

        first_for_display = false;
    }
}
