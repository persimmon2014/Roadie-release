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
