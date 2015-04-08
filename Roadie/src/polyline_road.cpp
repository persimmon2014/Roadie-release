/*
 * polyline_road.cpp
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

#include "polyline_road.hpp"
#include "svg_helper.hpp"
#include <iostream>

polyline_road::~polyline_road() {}

float polyline_road::length(const float offset) const
{
    return clengths_.back() - 2*offset*cmitres_.back();
}


vec3f polyline_road::point(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    const float last_cmitre = idx == 0 ? 0 : cmitres_[idx-1];
    const float mitre       = cmitres_[idx] - last_cmitre;

    const float local_t = t*length(offset) - (clengths_[idx] - offset*(cmitres_[idx] + last_cmitre));

    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    return vec3f(points_[idx] + (local_t + offset*mitre)*normals_[idx] - offset*right);
}


mat3x3f polyline_road::frame(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    // Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    mat3x3f res;
    res(0, 0) = normals_[idx][0]; res(0, 1) = right[0]; res(0, 2) = up[0];
    res(1, 0) = normals_[idx][1]; res(1, 1) = right[1]; res(1, 2) = up[1];
    res(2, 0) = normals_[idx][2]; res(2, 1) = right[2]; res(2, 2) = up[2];

    return res;
}

mat4x4f polyline_road::point_frame(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    const float last_cmitre = idx == 0 ? 0 : cmitres_[idx-1];
    const float mitre       = cmitres_[idx] - last_cmitre;

    const float local_t = t*length(offset) - (clengths_[idx] - offset*(cmitres_[idx] + last_cmitre));

    const vec3f point(points_[idx] + (local_t + offset*mitre)*normals_[idx] - offset*right);

    mat4x4f fr;
    fr(0, 0) = normals_[idx][0]; fr(0, 1) = right[0]; fr(0, 2) = up[0]; fr(0, 3) = point[0];
    fr(1, 0) = normals_[idx][1]; fr(1, 1) = right[1]; fr(1, 2) = up[1]; fr(1, 3) = point[1];
    fr(2, 0) = normals_[idx][2]; fr(2, 1) = right[2]; fr(2, 2) = up[2]; fr(2, 3) = point[2];
    fr(3, 0) = 0.0f;             fr(3, 1) = 0.0f;     fr(3, 2) = 0.0f;  fr(3, 3) = 1.0f;
    return fr;
}



void polyline_road::translate(const vec3f &o)
{
    for(vec3f &pt: points_)
    {
        pt += o;
    }
}



bool polyline_road::initialize()
{
    const size_t N = points_.size();
    clengths_.resize(N);
    normals_.resize(N-1);

    clengths_[0] = 0.0f;
    for(size_t i = 1; i < N; ++i)
    {
        normals_[i-1] = points_[i] - points_[i-1];
        const float len = ::length(normals_[i-1]);
        if(len < FLT_EPSILON)
            return false;

	normals_[i-1] /= len;
        clengths_[i] = clengths_[i-1] + len;
        
    }
    inv_len_ = 1.0f/clengths_.back();

    cmitres_.resize(N);

    cmitres_[0] = 0.0f;
    if (N >= 2){
        for(size_t i = 0; i < N - 2; ++i)
        {
            const float dot = tvmet::dot(normals_[i], normals_[i+1]);
            if(std::abs(dot + 1.0f) < FLT_EPSILON)
                return false;
            const float orient = normals_[i][1] * normals_[i+1][0] - normals_[i][0]*normals_[i+1][1];
            const float mitre  = (dot > 1.0f) ? 0.0f : copysign(std::sqrt((1.0f - dot)/(1.0f + dot)), -orient); // the value of tan(theta)? theta is between circle_ctr - cur_pt and next_pt - cur_pt

            cmitres_[i+1] += cmitres_[i] + mitre;
        }
    }


    cmitres_[N-1] = cmitres_[N-2]; // + 0.0f;

    return true;
}

size_t polyline_road::locate(const float t, const float offset) const
{
    const float                              scale_t = t*length(offset);
    const std::vector<float>::const_iterator found   = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    const size_t                             idx     = found - clengths_.begin();
    return (idx > 0) ? idx - 1 : 0;
}

size_t polyline_road::locate_scale(float &local_t, const float offset, const float t) const
{
    const float                              scale_t = t*length(offset);
    const std::vector<float>::const_iterator found   = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    const size_t                             idx     = found - clengths_.begin();
    if(idx > 0)
    {
        local_t = (scale_t - clengths_[idx-1]) / (clengths_[idx] - clengths_[idx-1]);
        return idx - 1;
    }
    else
    {
        local_t = 0.0f;
        return 0;
    }
}

void polyline_road::check() const
{
    if(points_.empty())
        throw std::runtime_error("No points in polyline_road");
    if(points_.size() == clengths_.size())
        throw std::runtime_error("Wrong number of clengths in polyline_road");
    if(points_.size() == cmitres_.size())
        throw std::runtime_error("Wrong number of cmitres in polyline_road");
    if(points_.size() == normals_.size() + 1)
        throw std::runtime_error("Wrong number of normals in polyline_road");
}

path polyline_road::svg_poly_path_center(const vec2f &interval, const float offset) const
{
    return svg_poly_path(interval, offset);
}

path polyline_road::svg_poly_path       (const vec2f &interval, const float offset) const
{
    vec2f range(interval);
    bool reversed = range[0] > range[1];
    if(reversed)
        std::swap(range[0], range[1]);

    const size_t start_idx = std::min(normals_.size()-1, locate(range[0], offset));
    const size_t end_idx   = std::min(normals_.size()-1, locate(range[1], offset));

    path p;

    vec3f last_point;
    {
        const float last_cmitre = start_idx == 0 ? 0 : cmitres_[start_idx-1];
        const float mitre       = cmitres_[start_idx] - last_cmitre;

        const float local_t = range[0]*length(offset) - (clengths_[start_idx] - offset*(cmitres_[start_idx] + last_cmitre));

        const vec3f right(normals_[start_idx][1], -normals_[start_idx][0], 0.0f);
        last_point = points_[start_idx] + (local_t + offset*mitre)*normals_[start_idx] - offset*right;
    }
    for(int c = start_idx + 1; c <= static_cast<int>(end_idx); ++c)
    {
        const float mitre = cmitres_[c] - cmitres_[c-1];

        const vec3f right(normals_[c][1], -normals_[c][0], 0.0f);
        const vec3f new_point(points_[c] + offset*(mitre*normals_[c] - right));
        p.add_line(last_point, new_point);
        last_point = new_point;
    }
    {
        const float last_cmitre = end_idx == 0 ? 0 : cmitres_[end_idx-1];
        const float mitre       = cmitres_[end_idx] - last_cmitre;

        const float local_t = range[1]*length(offset) - (clengths_[end_idx] - offset*(cmitres_[end_idx] + last_cmitre));

        const vec3f right(normals_[end_idx][1], -normals_[end_idx][0], 0.0f);
        p.add_line(last_point, vec3f(points_[end_idx] + (local_t + offset*mitre)*normals_[end_idx] - offset*right));
    }

    if(reversed)
        p.reverse();

    return p;
}
