/*
 * hilbert.hpp
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

#ifndef _HILBERT_HPP_
#define _HILBERT_HPP_

#include "libroad_common.hpp"

struct hilbert
{
    static size_t quadrant(const size_t x, const size_t y, const size_t w)
    {
        // const bool xbig = x >= w;
        // const bool ybig = y >= w;
        // return xbig*(3 - ybig) + !xbig*ybig;

        if(x >= w)
        {
            if( y >= w )
                return 2;

            return 3;
        }
        if( y >= w )
            return 1;
        return 0;
    }

    static void h_update(size_t &x, size_t &y, const size_t r, const size_t w)
    {
        switch(r)
        {
        case 0:
            std::swap(x, y);
            break;
        case 1:
            y -= w;
            break;
        case 2:
            x -= w;
            y -= w;
            break;
        case 3:
            {
                const size_t in_x = x;
                x = w - y - 1;
                y = 2*w - in_x - 1;
            }
            break;
        };
    }

    static size_t order(const float x, const float y)
    {
        return order_num(static_cast<size_t>(std::floor(x*((1LL << 24)-1))), static_cast<size_t>(std::floor(y*((1LL << 24)-1))), 24);
    }

    static double order_norm(const float x, const float y)
    {
        return order_num(static_cast<size_t>(std::floor(x*((1LL << 24)-1))), static_cast<size_t>(std::floor(y*((1LL << 24)-1))), 24)/static_cast<double>((1LL << (2*24)) -1);
    }

    static size_t order_num(size_t x, size_t y, const size_t n)
    {
        if(x == 0 && y == 0)
            return 0;

        size_t z    = 0;
        size_t rmin = static_cast<size_t>(std::floor(std::log(static_cast<double>(std::max(x, y)))/M_LN2)) + 1;
        size_t w    = 1LL << (rmin - 1);
        if((rmin & 1) != (n & 1))
            std::swap(x, y);

        while(rmin > 0)
        {
            const size_t r = quadrant(x, y, w);
            z = 4*z + r;
            h_update(x, y, r, w);
            --rmin;
            w >>= 1LL;
        }
        return z;
    }

    static void order_str(size_t x, size_t y, const size_t n, size_t *z, size_t &nchars)
    {
        nchars = 0;
        if(x == 0 && y == 0)
        {
            z[nchars++] = 0;
            return;
        }
        size_t rmin = static_cast<size_t>(std::floor(std::log(static_cast<double>(std::max(x, y)))/M_LN2l)) + 1;
        size_t w = 1LL << (rmin - 1);
        if((rmin & 1) != (n & 1))
            std::swap(x, y);

        while(rmin > 0)
        {
            const size_t r = quadrant(x, y, w);
            z[nchars++]    = r;
            h_update(x, y, r, w);
            --rmin;
            w            >>= 1LL;
        }
    }
};

#endif
