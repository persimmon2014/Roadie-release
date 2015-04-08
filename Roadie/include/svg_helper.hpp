/*
 * svg_helper.hpp
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

#ifndef _SVG_HELPER_HPP_
#define _SVG_HELPER_HPP_

#include "libroad_common.hpp"
#if HAVE_CAIRO
#include <cairo.h>
#endif

struct path_element
{
    virtual void          reverse()                    = 0;
    virtual vec3f        &point0()                     = 0;
    virtual vec3f        &point1()                     = 0;
    virtual path_element *copy() const                 = 0;
#if HAVE_CAIRO
    virtual void          cairo_draw(cairo_t *c,
                                     bool start) const = 0;
#endif
    virtual str           stringify(bool start) const  = 0;
};

struct line_segment : public path_element
{
    line_segment(const vec3f &p0, const vec3f &p1);

    virtual void          reverse();
    virtual vec3f        &point0();
    virtual vec3f        &point1();
    virtual path_element *copy() const;
#if HAVE_CAIRO
    virtual void cairo_draw(cairo_t *c, bool start) const;
#endif
    virtual str stringify(bool start) const;

    vec3f points[2];
};

struct arc_segment : public path_element
{
    arc_segment(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1);

    virtual void          reverse();
    virtual vec3f        &point0();
    virtual vec3f        &point1();
    virtual path_element *copy() const;
#if HAVE_CAIRO
    virtual void cairo_draw(cairo_t *c, bool start) const;
#endif
    virtual str stringify(bool start) const;

    vec3f points[2];
    float offset;
    bool  orientation;
    float radius;
};

struct path
{
    path();
    ~path();

    void add_line(const vec3f &p0, const vec3f &p1);
    void add_arc(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1);
    void reverse();
    void append(const path &o);
#if HAVE_CAIRO
    void cairo_draw(cairo_t *ct, const bool new_path=true) const;
#endif
    str stringify() const;

    std::vector<path_element*> elements;
};
#endif
