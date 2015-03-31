#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.H"
#include <Fl/Fl_Box.H>
#include "arcball.hpp"
#include "hwm_network.hpp"
#include "hwm_draw.hpp"

static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(11.35),
                                                          car_pos(0.0f),
                                                          pick_vert(-1),
                                                          glew_state(GLEW_OK+1),
                                                          light_position(50.0, 100.0, 50.0, 1.0),
                                                          tex_(0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

        this->resizable(this);
    }

    void setup_light()
    {
        static const GLfloat amb_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat diff_light_rgba[] = { 0.7, 0.7, 0.7, 1.0 };
        static const GLfloat spec_light_rgba[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat spec_material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat shininess = 100.0;

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
    }

    void init_glew()
    {
        glew_state = glewInit();
        if (GLEW_OK != glew_state)
        {
            /* Problem: glewInit failed, something is seriously wrong. */
            std::cerr << "Error: " << glewGetErrorString(glew_state)  << std::endl;
        }
        std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    }

    void init_textures()
    {
        if(!glIsTexture(tex_))
        {
            glGenTextures(1, &tex_);
            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            glDisable(GL_TEXTURE_2D);
        }
    }

    void draw()
    {
        if (!valid())
        {
	    //glutCreateWindow(0,0,500,500);
            glViewport(0, 0, w(), h());
	    
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 2000.0f, 180000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(1.0, 1.0, 1.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

	
            if(GLEW_OK != glew_state)
                init_glew();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*net->lane_width,
                                      CAR_LENGTH,
                                      1.5f,
                                      CAR_REAR_AXLE);

            if(!network_drawer.initialized())
                network_drawer.initialize(net, 0.01f);

            init_textures();

            setup_light();

            if(net)
            {
                bb[0] = vec3f(FLT_MAX);
                bb[1] = vec3f(-FLT_MAX);
                net->bounding_box(bb[0], bb[1]);
            }
            glEnable(GL_MULTISAMPLE);
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        if(net)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
            //            glColor3f(0.1, 0.2, 0.1);
            glColor3f(1.0, 1.0, 1.0);
            glPushMatrix();
            glTranslatef(0.0, 0.0, bb[0][2]-1.0f);
            glBegin(GL_QUADS);
            glVertex2f(2*bb[0][0], 2*bb[0][1]);
            glVertex2f(2*bb[1][0], 2*bb[0][1]);
            glVertex2f(2*bb[1][0], 2*bb[1][1]);
            glVertex2f(2*bb[0][0], 2*bb[1][1]);
            glEnd();
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);

            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);
            std::vector<vec4f> colors(100);
            for(size_t i = 0; i < colors.size(); ++i)
            {
                colors[i][0] = colors[i][1] = colors[i][2] = static_cast<float>(i)/(colors.size()-1);
                colors[i][3] = 1.0f;
            }

            glTexImage2D (GL_TEXTURE_2D,
                          0,
                          GL_RGBA,
                          100,
                          1,
                          0,
                          GL_RGBA,
                          GL_FLOAT,
                          colors[0].data());

            glColor3f(0.5, 0.5, 0.5);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);
            //            network_drawer.draw_lanes_wire();
            for(const hwm::lane_pair &l: net->lanes)
            {
              network_drawer.draw_lane_wire(l.first);
            }

            glColor3f(1.0, 0.5, 0.5);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);
            network_drawer.draw_fictitious_lanes_wire();

            glDisable(GL_TEXTURE_2D);

            glColor3f(0.5, 0.5, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);
            network_drawer.draw_intersections_wire();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            for(const hwm::lane_pair &l: net->lanes)
            {
                const hwm::lane &la = l.second;

                mat4x4f trans(la.point_frame(car_pos));
                vec3f   this_point(trans(0,3), trans(1,3), trans(2,3));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 1.0, 0.0);

                glDisable(GL_BLEND);
                glEnable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glPushMatrix();
                glMultMatrixf(ttrans.data());
                //                car_drawer.draw();
                glPopMatrix();

                {
                    float p = car_pos;
                    const hwm::lane *left = la.left_adjacency(p);
                    if(left)
                    {
                        glColor3f(0.0, 1.0, 0.0);
                        const vec3f other_point(left->point(p));
                        glEnable(GL_BLEND);
                        glDisable(GL_LIGHTING);
                        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                        glBegin(GL_LINES);
                        glVertex3fv(this_point.data());
                        glVertex3fv(other_point.data());
                        glEnd();
                        glDisable(GL_BLEND);
                        glEnable(GL_LIGHTING);
                        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    }
                }
                {
                    float p = car_pos;
                    const hwm::lane *right = la.right_adjacency(p);
                    if(right)
                    {
                        glColor3f(1.0, 0.0, 0.0);
                        const vec3f other_point(right->point(p));
                        glEnable(GL_BLEND);
                        glDisable(GL_LIGHTING);
                        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
                        glBegin(GL_LINES);
                        glVertex3fv(this_point.data());
                        glVertex3fv(other_point.data());
                        glEnd();
                        glDisable(GL_BLEND);
                        glEnable(GL_LIGHTING);
                        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                    }
                }

            }

            glEnable(GL_BLEND);
            glDisable(GL_LIGHTING);
        }

        glFlush();
        glFinish();
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =   2.0f*x/(w()-1) - 1.0f;
                float fy = -(2.0f*y/(h()-1) - 1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =  2.0f*x/(w()-1)-1.0f;
                float fy = -(2.0f*y/(h()-1)-1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
		           
                    nav.get_click(fx, fy, 1.0f, true);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float scale = std::pow(2.0f, zoom-1.0f);

                    double update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    nav.translate(update);
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float scale = std::pow(1.5f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 17.0f)
                        zoom = 17.0f;
                    else if(zoom < FLT_MIN)
                        zoom = FLT_MIN;

                    std::cout << "zoom: " << zoom << std::endl;

                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case 'n':
                if(net)
                {
                    for(hwm::intersection_pair &i: net->intersections)
                    {
                        i.second.advance_state();
                    }
                }
                break;
            case 't':
                car_pos += 0.02f;
                if(car_pos > 1.0f)
                    car_pos = 1.0f;
                break;
            case 'g':
                car_pos -= 0.02f;
                if(car_pos < 0.0f)
                    car_pos = 0.0f;
                break;
            default:
                break;
            }
            redraw();
            return 1;
        case FL_MOUSEWHEEL:
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    arcball nav;
    float zoom;
    float lastmouse[2];

    float              car_pos;
    hwm::car_draw      car_drawer;
    hwm::network_draw  network_drawer;
    hwm::network      *net;

    vec3f bb[2];

    int    pick_vert;
    GLuint glew_state;
    vec4f  light_position;
    GLuint tex_;
};


void renderFunction()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.0, 1.0, 0.0);
    glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
    glBegin(GL_POLYGON);
        glVertex2f(-0.5, -0.5);
        glVertex2f(-0.5, 0.5);
        glVertex2f(0.5, 0.5);
        glVertex2f(0.5, -0.5);
    glEnd();
    glFlush();
}

int main(int argc, char *argv[])
{
    if(argc < 2)
    {
       std::cerr << "Usage: " << argv[0] << " <input network>" << std::endl;
       return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0f, 1.0f, 1.0f)));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }
    
    fltkview mv(0, 0, 500, 500, "fltk View");
    mv.net = &net;
 
    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH|FL_MULTISAMPLE);
     
    //mv.draw();
    mv.show(1, argv);
    return Fl::run();
}
