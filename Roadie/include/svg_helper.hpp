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
