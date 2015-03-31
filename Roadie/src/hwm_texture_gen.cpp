#include "hwm_network.hpp"
#include "hwm_texture_gen.hpp"

namespace hwm
{
    static void write_road_mtl(std::ostream      &o,
                               const std::string &ts_name,
                               const std::string &ts_file)
    {
	o << std::string("newmtl " + ts_name + "\n" 
	                  + "ns 96.078431\n"
			  + "ka 0.0  0.0  0.0\n" 
			  + "kd 0.64 0.64 0.64\n"
			  + "ks 0.5  0.5  0.5\n"
			  + "ni 1.0\n"
			  + "d  1.0\n"
			  + "map_kd "+ ts_file +"\n");
    }

    xgap::xgap(double  w_)
        : w(w_)
    {}

    std::string xgap::rep() const
    {
	return std::string("xg" + std::to_string(w));
    }

    double xgap::width() const
    {
        return w;
    }

    double xgap::xres() const
    {
        return w;
    }

    double xgap::height() const
    {
        return 0.0;
    }

    double xgap::yres() const
    {
        return std::numeric_limits<double>::max();
    }

    void xgap::draw(cairo_t *cr) const
    {
    }

    static void aligned_rectangle(cairo_t *cr,
                                  double x, double y,
                                  double w, double h)
    {
        cairo_user_to_device(cr, &x, &y);
        x = std::floor(x);
        y = std::floor(y);
        cairo_device_to_user(cr, &x, &y);

        w += x;
        h += y;
        cairo_user_to_device(cr, &w, &h);
        w = std::ceil(w);
        h = std::ceil(h);
        cairo_device_to_user(cr, &w, &h);

        cairo_rectangle(cr, x, y, w-x, h-y);
    }

    single_box::single_box(const double   w_,     const double h_,
                           const double   ybase_, const double ylen_,
                           const color4d &c_) :
        w(w_), h(h_),
        ybase(ybase_), ylen(ylen_),
        c(c_)
    {}

    std::string single_box::rep() const
    {
	return std::string("sb" + std::to_string(w) + "-" + std::to_string(h) + "-" + std::to_string(ybase) + "-" + std::to_string(ylen) + "-" + std::to_string(c[0]) + "-" + std::to_string(c[1]) + "-" + std::to_string(c[2]) + "-" + std::to_string(c[3])); 
    }

    double single_box::width() const
    {
        return w;
    }

    double single_box::xres() const
    {
        return w;
    }

    double single_box::height() const
    {
        return h;
    }

    double single_box::yres() const
    {
        const double base = ybase > 0.0 ? ybase : std::numeric_limits<double>::max();
        const double top  = (h-ybase-ylen) > 0.0 ? h-ybase-ylen : std::numeric_limits<double>::max();
        return std::min(base, std::min(ylen, top));
    }

    void single_box::draw(cairo_t *cr) const
    {
        cairo_set_source_rgba(cr, c[0], c[1], c[2], c[3]);
        aligned_rectangle(cr, ybase, 0, ylen, w);
        cairo_fill(cr);
    }

    double_box::double_box(const double   bw_, const double sep_,
                           const double   h_,  const double ybase_, const double ylen_,
                           const color4d &c_) :
        bw(bw_), sep(sep_), h(h_),
        ybase(ybase_), ylen(ylen_),
        c(c_)
    {}

    std::string double_box::rep() const
    {
	return std::string("db" + std::to_string(bw) + "-" + std::to_string(sep) + "-" + std::to_string(h) + "-" + std::to_string(ybase) + "-" + std::to_string(ylen) + "-" + std::to_string(c[0]) + "-" + std::to_string(c[1]) + "-" + std::to_string(c[2]) + "-" + std::to_string(c[3])); 
    }

    double double_box::width() const
    {
        return bw*2 + sep;
    }

    double double_box::xres() const
    {
        return std::min(sep, bw);
    }

    double double_box::height() const
    {
        return h;
    }

    double double_box::yres() const
    {
        const double base = ybase > 0.0 ? ybase : std::numeric_limits<double>::max();
        const double top  = (h-ybase-ylen) > 0.0 ? h-ybase-ylen : std::numeric_limits<double>::max();
        return std::min(base, std::min(ylen, top));
    }

    void double_box::draw(cairo_t *cr) const
    {
        cairo_set_source_rgba(cr, c[0], c[1], c[2], c[3]);

        aligned_rectangle(cr, ybase, 0, ylen, bw);
        cairo_fill(cr);

        aligned_rectangle(cr, ybase, sep+bw, ylen, bw);
        cairo_fill(cr);
    }

    lane_maker::~lane_maker()
    {
        for(lane_op *o: boxes)
        {
            delete o;
        }
    }

    void lane_maker::add_cbox(center_box *cb)
    {
        if(!boxes.empty())
        {
            xgap *b = dynamic_cast<xgap*>(boxes.back());
            if(b)
                b->w -= cb->width()*0.5;
        }
        boxes.push_back(cb);
    }

    void lane_maker::add_xgap(double w)
    {
        if(!boxes.empty())
        {
            center_box *b = dynamic_cast<center_box*>(boxes.back());
            if(b)
                w -= b->width()*0.5;
        }
        boxes.push_back(new xgap(w));
    }

    void lane_maker::res_scale()
    {
        double total_w    = 0.0;
        double min_x_feat = std::numeric_limits<double>::max();
        double max_h      = 0.0;
        double min_y_feat = std::numeric_limits<double>::max();
        for(const lane_op *lo: boxes)
        {
            total_w    += lo->width();
            min_x_feat  = std::min(min_x_feat, lo->xres());
            max_h       = std::max(max_h, lo->height());
            min_y_feat  = std::min(min_y_feat, lo->yres());
        }
        if(max_h == 0.0)
            max_h = 1.0;

        vec2u minsize(static_cast<size_t>(std::ceil(max_h/min_y_feat)),
                      static_cast<size_t>(std::ceil(total_w/min_x_feat)));
        im_res = 3*minsize;
        for(int i = 0; i < 2; ++i)
            im_res[i] = std::max(im_res[i], (size_t)(1));
        scale  = vec2d(max_h, total_w);
    }

    void lane_maker::draw(unsigned char *pix)
    {
        cairo_surface_t *cs = cairo_image_surface_create_for_data(pix,
                                                                  CAIRO_FORMAT_ARGB32,
                                                                  im_res[0],
                                                                  im_res[1],
                                                                  sizeof(unsigned char)*4*im_res[0]);
        cairo_t         *cr = cairo_create(cs);

        cairo_set_source_rgba(cr, 0.4, 0.4, 0.4, 1.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
        cairo_scale(cr, im_res[0]/scale[0], im_res[1]/scale[1]);

        for(const lane_op *lo: boxes)
        {
            lo->draw(cr);
            cairo_translate(cr, 0.0, lo->width());
        }

        cairo_destroy(cr);
        cairo_surface_destroy(cs);
    }

    void lane_maker::draw(const std::string &fname)
    {
        res_scale();
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);

        cairo_set_source_rgba(cr, 0, 0, 0, 1.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
        cairo_scale(cr, im_res[0]/scale[0], im_res[1]/scale[1]);

        for(const lane_op *lo: boxes)
        {
            lo->draw(cr);
            cairo_translate(cr, 0.0, lo->width());
        }

        cairo_destroy(cr);
        cairo_surface_write_to_png(cs, fname.c_str());
        cairo_surface_destroy(cs);
    }

    std::string lane_maker::make_string() const
    {
        std::string res;
        for(const lane_op *lo: boxes)
        {
            res += lo->rep() + "!";
        }
        return res;
    }

    lane_maker network_aux::road_rev_map::lane_cont::lane_tex(const road_metrics &rm) const
    {
        lane_maker lm;
        // lm.add_xgap(0.25*lane_width);
        // lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
        //                            0,          line_length+line_gap_length,
        //                            color4d(1.0, 1.0, 1.0, 1.0)));

        const_iterator i = begin();
        const float orient0 = copysign(1, i->second.membership->interval[0] - i->second.membership->interval[1]);
        while(i != end() && orient0 == copysign(1, i->second.membership->interval[0] - i->second.membership->interval[1]))
        {
            lm.add_xgap(rm.lane_width);
            const_iterator next = std::next(i);
            if(next != end() && orient0 == copysign(1, next->second.membership->interval[0] - next->second.membership->interval[1]))
                lm.add_cbox(new single_box(rm.line_width, rm.line_length+rm.line_gap_length,
                                           0, rm.line_length,
                                           color4d(1.0, 1.0, 1.0, 1.0)));
            i = next;
        }
        if(i != end() && !lm.boxes.empty())
        {
            lm.add_cbox(new double_box(rm.line_width, rm.line_sep_width,
                                       rm.line_length+rm.line_gap_length, 0, rm.line_length+rm.line_gap_length,
                                       color4d(1.0, 1.0, 0.0, 1.0)));
        }
        while(i != end())
        {
            lm.add_xgap(rm.lane_width);
            const_iterator next = std::next(i);
            if(next != end())
                lm.add_cbox(new single_box(rm.line_width, rm.line_length+rm.line_gap_length,
                                           0, rm.line_length,
                                           color4d(1.0, 1.0, 1.0, 1.0)));
            i = next;
        }
        // lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
        //                            0,          line_length+line_gap_length,
        //                            color4d(1.0, 1.0, 1.0, 1.0)));
        // lm.add_xgap(0.25*lane_width);

        return lm;
    }

    const std::string network_aux::road_rev_map::lane_cont::write_texture(tex_db &tdb, const road_metrics &rm) const
    {
        lane_maker lm(lane_tex(rm));
        return tdb.do_tex(lm);
    }

    //tex_db::tex_db(std::ostream &ml, const bf::path &bp)
    tex_db::tex_db(std::ostream &ml, const std::string &bp)
        : mtllib(ml), base_path(bp), image_count(0)
    {}

    //const bf::path tex_db::get_filename(const size_t idx) const
    const std::string tex_db::get_filename(const size_t idx) const
    {
	return std::string(base_path + "/" + std::string(get_matname(idx) + ".png"));
    }

    const std::string tex_db::get_matname(const size_t idx) const
    {
	return std::string(std::to_string(idx));
    }

    const std::string tex_db::do_tex(lane_maker &lm)
    {
        const std::string rep_str(lm.make_string());

        base_t::iterator lookup(find(rep_str));
        if(lookup == end())
        {
            const std::string fname(get_filename(image_count));
            lm.draw(fname);
            lookup = insert(lookup, std::make_pair(rep_str, image_count));
            write_road_mtl(mtllib, get_matname(lookup->second), fname);
            ++image_count;
        }

        return get_matname(lookup->second);
    };
}
