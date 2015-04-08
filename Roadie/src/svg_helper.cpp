/*
 * svg_helper.cpp
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

#include "svg_helper.hpp"

struct endpoint_param
{
    vec2f p0;
    vec2f p1;
    vec2f radii;
    bool  fa;
    bool  fs;
};

static float angle_func(const vec2f &u, const vec2f &v)
{
    const float sign = copysign(1.0f, u[0]*v[1] - u[1]*v[0]);
    return sign*std::acos(tvmet::dot(u, v)/(length(u)*length(v)));
}

struct center_param
{
    void from_endpoint(const endpoint_param &ep)
    {
        radii = ep.radii;
        phi   = 0;
        const vec2f xp((ep.p0 - ep.p1)/2);

        const float sign = (ep.fa != ep.fs) ? 1.0 : -1.0f;
        const float r02  = radii[0]*radii[0];
        const float r12  = radii[1]*radii[1];
        const float fac  = sign*std::sqrt((r02*r12 -
                                          r02*xp[1]*xp[1] -
                                          r12*xp[0]*xp[0])/
                                         (r02*xp[1]*xp[1] +
                                          r12*xp[0]*xp[0]));
        const vec2f cp(fac*vec2f(radii[0]*xp[1]/radii[1],
                                 -radii[1]*xp[0]/radii[0]));
        center           = vec2f(cp + (ep.p0 + ep.p1)/2);

        theta  = angle_func(vec2f(1.0f, 0.0f), vec2f((float)(xp[0] - cp[0])/radii[0],
                                               (float)(xp[1] - cp[1])/radii[1]));
        dtheta = angle_func(vec2f((float)(xp[0] - cp[0])/radii[0],
                                  (float)(xp[1] - cp[1])/radii[1]),
                            vec2f((float)(-xp[0] - cp[0])/radii[0],
                                  (float)(-xp[1] - cp[1])/radii[1]));
        if(!ep.fs && dtheta > 0)
            dtheta -= M_2_PI;
        else if(ep.fs && dtheta < 0)
            dtheta += M_2_PI;
    }

    vec2f center;
    vec2f radii;
    float phi;
    float theta;
    float dtheta;
};

line_segment::line_segment(const vec3f &p0, const vec3f &p1)
{
    points[0] = p0;
    points[1] = p1;
}

void line_segment::reverse()
{
    std::swap(points[0], points[1]);
}

vec3f &line_segment::point0()
{
    return points[0];
}

vec3f &line_segment::point1()
{
    return points[1];
}

path_element *line_segment::copy() const
{
    return new line_segment(points[0], points[1]);
}

#if HAVE_CAIRO
void line_segment::cairo_draw(cairo_t *c, bool start) const
{
    if(start)
        cairo_move_to(c, points[0][0], points[0][1]);
    else
        cairo_line_to(c, points[0][0], points[0][1]);
    cairo_line_to(c, points[1][0], points[1][1]);
}
#endif

str line_segment::stringify(bool start) const
{   
    if(start)
        return std::string("M" + std::to_string(points[0][0]) +"," + std::to_string(points[0][1]) + " L" + std::to_string(points[1][0]) + "," + std::to_string(points[1][1]));
    else
        return std::string("L" + std::to_string(points[1][0]) + "," + std::to_string(points[1][1]));
}

arc_segment::arc_segment(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1)
{
    points[0]   = p0;
    points[1]   = p1;
    offset      = off;
    orientation = o;
    radius      = r;
}

void arc_segment::reverse()
{
    std::swap(points[0], points[1]);
    orientation = !orientation;
    offset *= -1;
}

vec3f &arc_segment::point0()
{
    return points[0];
}

vec3f &arc_segment::point1()
{
    return points[1];
}

path_element *arc_segment::copy() const
{
    return new arc_segment(points[0], radius, offset, orientation, points[1]);
}

#if HAVE_CAIRO
void arc_segment::cairo_draw(cairo_t *c, bool start) const
{
    const float offset_val = orientation ? -offset : offset;

    if(start)
        cairo_move_to(c, points[0][0], points[0][1]);

    endpoint_param ep;
    ep.radii = vec2f(radius+offset_val, radius+offset_val);
    ep.p0    = sub<0,2>::vector(points[0]);
    ep.p1    = sub<0,2>::vector(points[1]);
    ep.fa    = 0;
    ep.fs    = orientation;

    center_param cp;
    cp.from_endpoint(ep);

    if(cp.dtheta > 0)
        cairo_arc(c, cp.center[0], cp.center[1],
                  cp.radii[0], cp.theta, cp.theta + cp.dtheta);
    else
        cairo_arc_negative(c, cp.center[0], cp.center[1],
                           cp.radii[0], cp.theta, cp.theta + cp.dtheta);
}
#endif

str arc_segment::stringify(bool start) const
{
    const float offset_val = orientation ? -offset : offset;
    
    if(start)
        return std::string("M" + std::to_string(points[0][0]) +"," + std::to_string(points[0][1]) + " A" + std::to_string(radius+offset_val) + "," + std::to_string(radius+offset_val) +
	  " 0 0," + std::to_string(orientation) + " " + std::to_string(points[1][0]) + "," + std::to_string(points[1][1]));
    else
        return std::string("A" + std::to_string(radius+offset_val) + "," + std::to_string(radius+offset_val) + " 0 0," + std::to_string(orientation) + " " + std::to_string(points[1][0]) + "," + std::to_string(points[1][1]));
}

path::path()
{
}

path::~path()
{
    for(path_element *p: elements)
    {
        delete p;
    }
}

void path::add_line(const vec3f &p0, const vec3f &p1)
{
    if(distance2(p0, p1) > 1e-6 && (elements.empty() || distance2(elements.back()->point1(), p1) > 1e-6))
        elements.push_back(new line_segment(p0, p1));
}

void path::add_arc(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1)
{
    if(distance2(p0, p1) > 1e-6 && (elements.empty() || distance2(elements.back()->point1(), p1) > 1e-6))
        elements.push_back(new arc_segment(p0, r, off, o, p1));
}

void path::reverse()
{
    std::reverse(elements.begin(), elements.end());
    for(path_element *p: elements)
    {
        p->reverse();
    }
}

void path::append(const path &o)
{
    if(o.elements.empty())
        return;
    if(!elements.empty())
        add_line(elements.back()->point1(), o.elements.front()->point0());

    for(const path_element *pe: o.elements)
    {
        elements.push_back(pe->copy());
    }
}

#if HAVE_CAIRO
void path::cairo_draw(cairo_t *ct, const bool new_path) const
{
    if(elements.empty())
        return;
    std::vector<path_element*>::const_iterator c = elements.begin();
    (*c)->cairo_draw(ct, new_path);
    ++c;
    for(; c != elements.end(); ++c)
        (*c)->cairo_draw(ct, false);
}
#endif

str path::stringify() const
{
    str res;
    if(elements.empty())
        return res;

    std::vector<path_element*>::const_iterator c = elements.begin();
    res.append((*c)->stringify(true));
    ++c;
    for(; c != elements.end(); ++c)
        res.append((*c)->stringify(false));

    return res;
}
