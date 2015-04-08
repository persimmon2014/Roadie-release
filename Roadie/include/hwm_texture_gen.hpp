/*
 * hwm_texture_gen.hpp
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

#ifndef _HWM_TEXTURE_GEN_HPP_
#define _HWM_TEXTURE_GEN_HPP_

#include "libroad_common.hpp"
#include <cairo.h>

namespace hwm
{
    typedef tvmet::Vector<double, 4> color4d;

    struct lane_op
    {
        virtual std::string rep()             const = 0;
        virtual double      width()           const = 0;
        virtual double      xres()            const = 0;
        virtual double      height()          const = 0;
        virtual double      yres()            const = 0;
        virtual void        draw(cairo_t *cr) const = 0;
    };

    struct xgap : public lane_op
    {
        xgap(double  w_);

        virtual std::string rep()             const;
        virtual double      width()           const;
        virtual double      xres()            const;
        virtual double      height()          const;
        virtual double      yres()            const;
        virtual void        draw(cairo_t *cr) const;

        double w;
    };

    struct center_box : public lane_op
    {
        virtual std::string rep()             const = 0;
        virtual double      width()           const = 0;
        virtual double      xres()            const = 0;
        virtual double      height()          const = 0;
        virtual double      yres()            const = 0;
        virtual void        draw(cairo_t *cr) const = 0;
    };

    struct single_box : public center_box
    {
        single_box(const double   w_,     const double h_,
                   const double   ybase_, const double ylen_,
                   const color4d &c_);

        virtual std::string rep()             const;
        virtual double      width()           const;
        virtual double      xres()            const;
        virtual double      height()          const;
        virtual double      yres()            const;
        virtual void        draw(cairo_t *cr) const;

        double  w, h;
        double  ybase;
        double  ylen;
        color4d c;
    };

    struct double_box : public center_box
    {
        double_box(const double   bw_, const double sep_,
                   const double   h_,  const double ybase_, const double ylen_,
                   const color4d &c_);

        virtual std::string rep()             const;
        virtual double      width()           const;
        virtual double      xres()            const;
        virtual double      height()          const;
        virtual double      yres()            const;
        virtual void        draw(cairo_t *cr) const;

        double  bw, sep, h;
        double  ybase;
        double  ylen;
        color4d c;
    };

    struct lane_maker
    {
        ~lane_maker();

        void add_cbox(center_box *cb);
        void add_xgap(double w);
        void res_scale();
        void draw(unsigned char *pix);
        void draw(const std::string &fname);
        std::string make_string() const;

        vec2u                 im_res;
        vec2d                 scale;
        std::vector<lane_op*> boxes;
    };

    struct tex_db : public std::map<const std::string, size_t>
    {
        typedef std::map<const std::string, size_t> base_t;

        //tex_db(std::ostream &ml, const bf::path &bp);
	tex_db(std::ostream &ml, const std::string &bp);

        //const bf::path    get_filename(const size_t idx) const;
        const std::string    get_filename(const size_t idx) const;
	
	const std::string get_matname(const size_t idx) const;
        const std::string do_tex(lane_maker &lm);

        std::ostream   &mtllib;
        //const bf::path  base_path;
	const std::string base_path;
        size_t          image_count;
    };
}
#endif
