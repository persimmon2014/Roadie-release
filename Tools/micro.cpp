#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.H"
#include "arcball.hpp"
#include "hwm_network.hpp"
#include "hwm_draw.hpp"

using namespace std;

static const float LANE_WIDTH = 2.5f;
static const float CAR_LENGTH = 4.5f;
static const float CAR_HEIGHT = 1.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class car
{
public:
    double pos;
    double vel;
    double a_max;
    double a_pref;
    double v_pref;
    double delta;
    double length;
    double accel;
    double dist;

    bool stopping;

    double _last_accel;

    bool operator==(const car& other) const
    {
        //Assumes we only check against cars in our lane.
        return (dist == other.dist);
    }

    car(){
        //All units are functions of meters and/or seconds
        pos = 0;
        vel = 0;
        a_max = 0.73;
        a_pref = 1.67;
        v_pref = 33;
        delta = 4;
        length = 5;
        accel = 0;
        dist = 0;
    }

    car(double p, double v, double _lane_length){
        //All units are functions of meters and/or seconds
        pos = p;
        vel = v;
        a_max = 0.73;
        a_pref = 1.67;
        v_pref = 33;
        delta = 4;
        length = 5;
        accel = 0;
        dist = p * _lane_length;
    }
};


struct micro_lane
{
    float       length;
    deque<car>  cars;
    hwm::lane  *parent_lane;
};

class micro /*: boost::noncopyable*/
{
public:

    double accel_calc(const car &l, const car &f)
    {
        const double s0 = 2;
        const double T = 1.6;

        //if (f.vel < 0) f.vel = 0; //TODO Should this be taken into account?

        const double s_opt = s0 + T*f.vel + (f.vel*(f.vel - l.vel))/2*(sqrt(f.a_max*f.a_pref));
        const double acel =  f.a_max*(1 - pow((f.vel / f.v_pref),f.delta) - pow((s_opt/(l.dist - f.dist - f.length)),2));

        //assert(l.dist - f.dist > 0); //A leader needs to lead.

        return acel;
    }

    void calc_accel_at_isect(const micro_lane &l, car *thisCar, float _free)
    {
        hwm::lane *fict_lane = l.parent_lane->downstream_lane();

        //TODO Have the intersections update their current state
        //Test if this lane is allowed through the intersection.
        if (!fict_lane)
            return;

        const micro_lane *fict_micro = fict_lane->user_data<micro_lane>();

        //Find the next car ahead or the amount of free space (up to the min_for_free_mvmnt limit.
        const double min_for_free_mvmnt = 1000;
        double free_dist = (1 - thisCar->pos) * l.length + _free; //Distance left in lane
        
        do{
            //There are no cars in the intersection lane.
            if (fict_micro->cars.empty())
            {
                free_dist += fict_micro->length;
                if (free_dist > min_for_free_mvmnt)
                {
                    car ghost_car;
                    ghost_car.vel = 0;
                    ghost_car.dist = thisCar->dist + free_dist;

                    thisCar->accel = accel_calc(ghost_car, *thisCar);
                    break;
                }
            }
            else
            {
                //TODO move to function
                car foo_car(fict_micro->cars.front());
                foo_car.dist += free_dist + thisCar->dist;

                //There is a car in the lane
                thisCar->accel = accel_calc(foo_car, *thisCar);
                break;
            }

            //Set up the next intersection lane if it is available, otherwise create a ghost.
            //A lane is available in the intersection.
            if(fict_lane->downstream_lane())
            {
                fict_lane  = fict_lane->downstream_lane();
                fict_micro = fict_lane->user_data<micro_lane>();
            }
            else
            {
                car ghost_car;
                ghost_car.vel = 0;
                ghost_car.dist = thisCar->dist + free_dist;

                thisCar->accel = accel_calc(ghost_car, *thisCar);
                break;
            }
        }while(true);
    }

    void calc_lane_accel(double timestep, hwm::lane &l)
    {
        micro_lane *micro = l.user_data<micro_lane>();

        //Calculate accel for all cars in lanes
        for (int i = 0; i < static_cast<int>(micro->cars.size())-1; i++)
        {
            car &thisCar = micro->cars[i];

            thisCar.stopping = false;
            micro->cars[i].accel = accel_calc(micro->cars[i+1], micro->cars[i]);
        }

        if(!micro->cars.empty())
        {
            //Behavior will be determined by the state of the intersection at the end of the lane	   
            if (l.downstream_lane())
            { 
                car &thisCar = micro->cars.back();
                calc_accel_at_isect(*micro, &thisCar, 0);
                thisCar.stopping = false;
            }
            else //If the lane ends, there is a red light, or the car wants to go a different direction.
            { 
                car &thisCar = micro->cars.back();
                car ghost_car(1, 0, micro->length); // create a ghost car at the end of the lane with velocity as 0

                thisCar.stopping = true;
                thisCar.accel = accel_calc(ghost_car, thisCar);
            }
        }
    }

    void calc_all_accel(double timestep, hwm::lane_map &lanes, hwm::intersection_map &intersections)
    {
        for(hwm::lane_pair& l: lanes)
        {
            calc_lane_accel(timestep, l.second);
        }

        //Repeat for i_lanes
        for(hwm::intersection_pair &ip: intersections)
        {
            for(hwm::intersection::state &current_state: ip.second.states)
            {
                for(hwm::lane_pair &l: current_state.fict_lanes)
                {
                    calc_lane_accel(timestep, l.second);
                }
            }
        }
    }

    void update(double timestep, hwm::lane_map &lanes, hwm::intersection_map &intersections)
    {
        calc_all_accel(timestep, lanes, intersections);

        //Update all cars in lanes
        for(hwm::lane_pair &l: lanes)
        {
	    //std::cout<<"asdf"<<l.second.user_data<micro_lane>()->cars.size()<<std::endl;
            for(car& c: l.second.user_data<micro_lane>()->cars)
            {
                c.dist += c.vel * timestep;
                c.vel += c.accel * timestep;
                c.pos = c.dist / l.second.user_data<micro_lane>()->length;
            }
        }
        for(hwm::intersection_pair &ip: intersections)
        {
            for(hwm::intersection::state &current_state: ip.second.states)
            {
                for(hwm::lane_pair &l: current_state.fict_lanes)
                {
                    for(car& c: l.second.user_data<micro_lane>()->cars)
                    {
                        c.dist += c.vel * timestep;
                        c.vel += c.accel * timestep;
                        c.pos = c.dist / l.second.user_data<micro_lane>()->length;
                    }
                }
            }
        }

        //Remove cars from lanes if they depart and place them in isect_lanes.
        for(hwm::lane_pair& l: lanes)
        {
            micro_lane *micro = l.second.user_data<micro_lane>();
            while(!micro->cars.empty() && micro->cars.back().dist > micro->length)
            {
                car &c = micro->cars.back();               
		//assert(not isnan(c.dist) and  not isinf(c.dist));
                		
		hwm::lane *new_lane = l.second.downstream_lane();
                if (new_lane)
                {
                    micro_lane *new_micro = new_lane->user_data<micro_lane>();
                    //Update position and distance.
                    c.dist -= micro->length;
                    c.pos = (float) c.dist / new_micro->length;
                    new_micro->cars.push_front(micro->cars.back());
                    micro->cars.pop_back();
                }
                else
                {
                    c.dist = micro->length;
                    c.pos = 1;
                    assert(0);
                }
            }
        }

        for(hwm::intersection_pair &ip: intersections)
        {
            for(hwm::intersection::state &current_state: ip.second.states)
            {
                for(hwm::lane_pair &l: current_state.fict_lanes)
                {
                    micro_lane *micro = l.second.user_data<micro_lane>();
                    while(!micro->cars.empty() && micro->cars.back().dist > micro->length)
                    {
                        car &c = micro->cars.back();
			
                        //assert(not isnan(c.dist) and  not isinf(c.dist));

                        hwm::lane *new_lane = l.second.downstream_lane();
                        if (new_lane)
                        {
                            micro_lane *new_micro = new_lane->user_data<micro_lane>();
                            //Update position and distance.
                            c.dist -= micro->length;
                            c.pos = (float) c.dist / new_micro->length;
                            new_micro->cars.push_front(micro->cars.back());
                        }
                        else
                        {
                            c.dist = micro->length;
                            c.pos = 1;
                            assert(0);
                        }
                        micro->cars.pop_back();
                    }
                }
            }
        }
    }

    void newer_settle(double timestep, hwm::lane_map &lanes, hwm::intersection_map &intersections)
    {
        double EPSILON = 1;
        double EPSILON_2 = 0.001;
        double INF = numeric_limits<double>::max();
        double max_accel = EPSILON;
        double last_max_accel = INF;

        for(hwm::lane_pair& l: lanes)
        {
            for(car& c: l.second.user_data<micro_lane>()->cars)
            {
                c._last_accel = INF;
            }
        }

        do
        {
            max_accel = EPSILON;
            for(hwm::lane_pair& l: lanes)
            {
                micro_lane *micro = l.second.user_data<micro_lane>();
                for (deque<car>::reverse_iterator c = micro->cars.rbegin();
                     c != micro->cars.rend();
                     )
                {
                    //Try to settle a single car.  Throw it out if it cannot be settled.
                    do
                    {
                        calc_lane_accel(timestep, l.second);

                        c->vel += c->accel * timestep;
                        c->vel = std::max(c->vel, 0.0);

                        if (std::abs(c->accel) > max_accel)
                            max_accel = std::abs(c->accel);

			
			
			
                        if ((std::abs(c->accel) > std::abs(c->_last_accel))
                            or (c->pos > 1) or ((std::abs(c->accel - c->_last_accel) < EPSILON_2) and (std::abs(c->accel) > EPSILON)))
                        {
			    //std::cout<<std::abs(c->accel - c->_last_accel)<<":"<<(std::abs(c->accel) > EPSILON)<<std::endl;
                            deque<car>::reverse_iterator to_erase = c;
                            c++;
                            micro->cars.erase(to_erase.base());
                            break;
                        }
                        else if (std::abs(c->accel) < EPSILON)
                        {
                            c++;
                            break;
                        }
                        else
                            c->_last_accel = c->accel;
                    }
                    while (true);

                    if (c == micro->cars.rend())
                        break;
                }
            }

            // if (max_accel >= last_max_accel)
            // {
            //     cout << "The cars did not settle." << endl;
            //     exit(0);
            // }

            last_max_accel = max_accel;
            cout << "max: " << max_accel << endl;

        }
        while (max_accel > EPSILON);
    }
};


/******************
Globals for OpenGL Loop
*/
double timestep = 0.033;
hwm::network* hnet;
micro sim;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          car_pos(0.0f),
                                                          pick_vert(-1),
                                                          glew_state(GLEW_OK+1),
                                                          light_position(50.0, 100.0, 50.0, 1.0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;
	
	zoom = 9.0;
	zoomgap = 0.1;
	xshift = 700.0f;
	yshift = -500.0f;
	xshiftgap = 5.0;
	yshiftgap = xshiftgap;

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
        glLightfv( GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv( GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv( GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv( GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
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
    

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 10.0f, 5000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);
            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*hnet->lane_width,
                                      CAR_LENGTH,
                                      CAR_HEIGHT,
                                      CAR_REAR_AXLE);

            if(!network_drawer.initialized())
                network_drawer.initialize(hnet, 0.01f);

            setup_light();

            bb[0] = vec3f(FLT_MAX);
            bb[1] = vec3f(-FLT_MAX);
            hnet->bounding_box(bb[0], bb[1]);
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        //glClearColor(1.0, 1.0, 1.0, 1.0);
        glLoadIdentity();

        glTranslatef(xshift, yshift, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        sim.update(timestep, hnet->lanes, hnet->intersections);

	
	// draw bounding box
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDisable(GL_LIGHTING);
        glColor3f(0.1, 0.2, 0.1);
        glPushMatrix();
        glTranslatef(0.0, 0.0, bb[0][2]-0.05f);
        glBegin(GL_QUADS);
        glVertex2f(bb[0][0], bb[0][1]);
        glVertex2f(bb[1][0], bb[0][1]);
        glVertex2f(bb[1][0], bb[1][1]);
        glVertex2f(bb[0][0], bb[1][1]);
        glEnd();
        glPopMatrix();
	
	
// 	glPushMatrix();
// 	 glTranslatef(840.598083, -332.485718, 0);
// 	 glColor3f(1.0,1.0,0.0);
// 	glutSolidCube(10);
// 	glPopMatrix();

	
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);

        glColor3f(0.5, 0.5, 0.5);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        network_drawer.draw_lanes_wire();

        glColor3f(0.5, 0.5, 1.0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);
        //network_drawer.draw_intersections_wire();

        glColor3f(1.0, 0.5, 0.5);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);
        network_drawer.draw_fictitious_lanes_wire();

	
        for(const hwm::lane_pair &l: hnet->lanes)
        {
            const micro_lane *micro = l.second.user_data<micro_lane>();
            for(const car& c: micro->cars)
            {
                mat4x4f trans(l.second.point_frame(c.pos));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 0.0, 0.0);

                glPushMatrix();
                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();
            }
        }

        for(const hwm::intersection_pair &ip: hnet->intersections)
        {
            for(const hwm::intersection::state &current_state: ip.second.states)
            {
                for(const hwm::lane_pair &l: current_state.fict_lanes)
                {
                    for(const car& c: l.second.user_data<micro_lane>()->cars)
                    {
                        mat4x4f trans(l.second.point_frame(c.pos));
                        mat4x4f ttrans(tvmet::trans(trans));
                        glColor3f(1.0, 0.0, 0.0);

                        glPushMatrix();
                        glMultMatrixf(ttrans.data());
                        car_drawer.draw();
                        glPopMatrix();
                    }
                }
            }
        }

        glDisable(GL_LIGHTING);
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
	      case FL_Page_Up:
		zoom -= zoomgap;
		break;
	      case FL_Page_Down:
		zoom += zoomgap;
		break;
	      case FL_Up:
		yshift -= yshiftgap;
		break;
	      case FL_Down:
		yshift += yshiftgap;
		break;
	      case FL_Left:
		xshift += xshiftgap;
		break;
	      case FL_Right:
		xshift -= xshiftgap;
		break;
	      case FL_Tab:
	      {
		if(hnet)
                {
                    for(hwm::intersection_pair &i: hnet->intersections)
                    {
                        i.second.advance_state();
                    }
                }
		break;
	      }
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
    float zoom, zoomgap;
    float xshift, yshift, xshiftgap, yshiftgap;
    float lastmouse[2];

    float              car_pos;
    hwm::car_draw      car_drawer;
    hwm::network_draw  network_drawer;

    vec3f bb[2];

    int pick_vert;
    GLuint glew_state;
    vec4f light_position;
};

void timerCallback(void*)
{
    Fl::redraw();
    Fl::repeat_timeout(timestep, timerCallback);
}


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <hwm network>" << std::endl;
        return 1;
    }
    
    
    hnet = new hwm::network(hwm::load_xml_network(argv[1]));
    hnet->build_intersections();
    hnet->build_fictitious_lanes();
    hnet->auto_scale_memberships();
    hnet->center();

    try
    {
        hnet->check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    // actual lanes
    for(hwm::lane_pair &l: hnet->lanes)
    {  
        micro_lane *ml = new micro_lane;
        l.second.user_datum = ml;
        ml->parent_lane = &(l.second);
        ml->length = l.second.length();
    }

    // fictious lanes at intersections, their id contain "_fict_lanes"
    for(hwm::intersection_pair &ip: hnet->intersections)
    {
        for(hwm::intersection::state &current_state: ip.second.states)
        {
            for(hwm::lane_pair &l: current_state.fict_lanes)
            {
                micro_lane *ml = new micro_lane;
                l.second.user_datum = ml;
                ml->parent_lane = &(l.second);
                ml->length = l.second.length();
            }
        }
    }

    //Create some sample cars
    int cars_per_lane = 1;
    
    for(hwm::lane_pair _lane: hnet->lanes)
    {
        micro_lane *micro = _lane.second.user_data<micro_lane>();
        if (/*_lane.first == str("lane4") or*/ true)
        {
            double p = 0.2; // starting portion on the lane
            for (int i = 0; i < cars_per_lane; i++)
            { 
	      //std::cout << "lll" << p << std::endl;
                car tmp(p, 33, micro->length); // position, velocity, lane length
                micro->cars.push_back(tmp);        
                p += (15.0 / micro->length); // Cars need a minimal distance spacing
            }
        }
    }

    //Make sure car configuration is numerically stable
    sim.newer_settle(timestep, hnet->lanes, hnet->intersections);

    fltkview mv(0, 0, 500, 500, "fltk View");

    Fl::add_timeout(timestep, timerCallback);

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
