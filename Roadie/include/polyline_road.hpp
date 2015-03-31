#ifndef _POLYLINE_ROAD_HPP_
#define _POLYLINE_ROAD_HPP_

#include "libroad_common.hpp"
#include "svg_helper.hpp"

struct polyline_road
{
    // member functions
    ~polyline_road();

    /**
     * function purpose
     * @param
     * @return 
     */
    bool   initialize();
    
    vec3f   point      (float t, float offset) const; 
    mat3x3f frame      (float t, float offset) const;
    mat4x4f point_frame(float t, float offset) const;
    
    inline float   length     (float offset)          const;
    inline void    translate(const vec3f &o);    
    inline size_t locate(float t, float offset) const;
    inline size_t locate_scale(float &local, float t, float offset) const;
    inline void   check() const;

    void   xml_read (xmlpp::TextReader &reader, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
    void   xml_write(xmlpp::Element *elt) const;
   
    path svg_poly_path_center(const vec2f &interval, const float offset) const;
    path svg_poly_path       (const vec2f &interval, const float offset) const;

    
    
    // member variables
    float inv_len_; // 1.0 / whole polyroad length
    std::vector<vec3f> points_; // all points
    std::vector<vec3f> normals_; // at each point, from the first point to the second last point; the normal is calculated as the (next_pt - current_pt) and take the normal 
    std::vector<float> clengths_; // the polyroad length, the value at each point is the sum of all lengths of previous points
    std::vector<float> cmitres_;
};
#endif
