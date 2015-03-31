#include "hwm_network.hpp"

namespace hwm
{
    lane::lane() : start(0), end(0), active(false)
    {}

    lane::lane(const lane &l) : id(l.id),
                                road_memberships(l.road_memberships),
                                left(l.left),
                                right(l.right),
                                speedlimit(l.speedlimit),
                                active(l.active),
                                user_datum(l.user_datum)
    {
        start = l.start ? l.start->clone() : 0;
        end   = l.end   ? l.end->clone()   : 0;
    }

    lane::~lane()
    {
        delete start;
        delete end;
    }

    lane::serial_state::serial_state()
    {
    }

    lane::serial_state::serial_state(const lane &l)
        : active(l.active)
    {
    }

    void lane::serial_state::apply(lane &l) const
    {
        l.active = active;
    }

    lane::terminus::~terminus()
    {
    }

    void lane::terminus::update_pointers(network &n)
    {
    }

    lane::terminus* lane::terminus::clone() const
    {
        return new lane::terminus();
    }

    void lane::terminus::check(bool start, const lane *parent) const
    {
        return;
    }

    lane* lane::terminus::incident(bool start) const
    {
        return 0;
    }

  std::vector<hwm::lane*>  lane::terminus::upstream_lanes() const
  {
    return std::vector<hwm::lane*>();
  }

  std::vector<const hwm::lane*>  lane::terminus::downstream_lanes(std::string id) const
  {
    return std::vector<const hwm::lane*>();
  }

    bool lane::terminus::network_boundary() const
    {
        return true;
    }

    lane::intersection_terminus::~intersection_terminus()
    {
    }

    void lane::intersection_terminus::update_pointers(network &n)
    {
        const intersection_map::iterator mine = n.intersections.find(adjacent_intersection->id);
        assert(mine != n.intersections.end());
        assert(mine->first == adjacent_intersection->id);
        adjacent_intersection                 = &(mine->second);
    }

    lane::intersection_terminus* lane::intersection_terminus::clone() const
    {
        return new lane::intersection_terminus(adjacent_intersection, intersect_in_ref);
    }

    void lane::intersection_terminus::check(bool start, const lane *parent) const
    {
        if(!adjacent_intersection || intersect_in_ref < 0)
            throw std::runtime_error("Incomplete intersection_terminus");

        const std::vector<lane*> &cont = start ? adjacent_intersection->outgoing : adjacent_intersection->incoming;
        if(intersect_in_ref >= static_cast<int>(cont.size()))
            throw std::runtime_error("intersection_terminus in_ref beyond containter size");
        else if(cont[intersect_in_ref] != parent)
            throw std::runtime_error("Mismatch in intersection_terminus parent ref and intersection ref");
    }

    lane* lane::intersection_terminus::incident(bool start) const
    {
        assert(adjacent_intersection);
        if(start)
            return adjacent_intersection->upstream_lane(intersect_in_ref);
        else
            return adjacent_intersection->downstream_lane(intersect_in_ref);
    }

  std::vector<lane*> lane::intersection_terminus::upstream_lanes() const
  {
    return adjacent_intersection->upstream_lanes(intersect_in_ref);
  }

  std::vector<const lane*> lane::intersection_terminus::downstream_lanes(std::string id) const
  {
    assert(adjacent_intersection);
    return adjacent_intersection->downstream_lanes(id);
  }


    bool lane::intersection_terminus::network_boundary() const
    {
        return false;
    }

    lane::lane_terminus::~lane_terminus()
    {
    }

    void lane::lane_terminus::update_pointers(network &n)
    {
        const lane_map::iterator mine = n.lanes.find(adjacent_lane->id);
        assert(mine != n.lanes.end());
        assert(mine->first == adjacent_lane->id);
        adjacent_lane = &(mine->second);
    }

    lane::lane_terminus* lane::lane_terminus::clone() const
    {
        return new lane::lane_terminus(adjacent_lane);
    }

    void lane::lane_terminus::check(bool start, const lane *parent) const
    {
        if(!adjacent_lane)
            throw std::runtime_error("Incomplete lane_terminus");

        const lane_terminus *other = dynamic_cast<const lane_terminus*>(start ? adjacent_lane->end : adjacent_lane->start);
        if(!other)
            throw std::runtime_error("No reciprocal lane_terminus");
        else if(other->adjacent_lane != parent)
            throw std::runtime_error("Reciprocal lane_terminus inconsistent");
    }

    lane* lane::lane_terminus::incident(bool start) const
    {
        assert(adjacent_lane);
        return adjacent_lane;
    }

  std::vector<lane*> lane::lane_terminus::upstream_lanes() const
  {
    std::vector<lane*> toReturn;
    toReturn.push_back(adjacent_lane);
    return toReturn;
  }

  std::vector<const hwm::lane*> lane::lane_terminus::downstream_lanes(std::string id) const
  {
    std::vector<const lane*> toReturn;
    toReturn.push_back(adjacent_lane);
    return toReturn;
  }

    bool lane::lane_terminus::network_boundary() const
    {
        return false;
    }

    void lane::road_membership::check() const
    {
        if(empty())
            throw std::runtime_error("Empty road_membership");
        else if(parent_road->id.empty())
            throw std::runtime_error("Bad parent road in road_membership!");
    }

    bool lane::road_membership::empty() const
    {
        return !parent_road;
    }

    float   lane::road_membership::length     () const
    {
        return std::abs(parent_road->rep.length(interval[0], lane_position) -
                        parent_road->rep.length(interval[1], lane_position));
    }

    vec3f   lane::road_membership::point      (float t, const float offset, const vec3f &up) const
    {
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.point(t, lane_position+offset, up);
    }

    vec3f   lane::road_membership::point_theta(float &theta, float t, const float offset, const vec3f &up) const
    {
        const bool reversed = (interval[0] > interval[1]);
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.point_theta(theta, t, lane_position+offset, reversed, up);
    }

    mat3x3f lane::road_membership::frame      (float t, const vec3f &up) const
    {
        const bool reversed = (interval[0] > interval[1]);
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.frame(t, lane_position, reversed, up);
    }

    mat4x4f lane::road_membership::point_frame(float t, const float offset, const vec3f &up) const
    {
        const bool reversed = (interval[0] > interval[1]);
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.point_frame(t, lane_position+offset, reversed, up);
    }

    void lane::adjacency::check() const
    {
        // could enforce symmetry here, but probably not necessary
        if(neighbor && neighbor->id.empty())
            throw std::runtime_error("Adjacency references nonexistent neighbor");
    }

    bool lane::adjacency::empty() const
    {
        return !neighbor;
    }

    void lane::check() const
    {
        if(id.empty())
            throw std::runtime_error("Lane with no id was created!");
        else if(road_memberships.empty())
            throw std::runtime_error("Lane has no road memberships!");

        start->check(true, this);
        end->check(false, this);

        if(speedlimit <= 0.0f)
            throw std::runtime_error("Lane has invalid speedlimit");

        for(const road_membership::intervals::entry &rmie: road_memberships)
        {
            rmie.second.check();
        }

        for(const adjacency::intervals::entry &aie: left)
        {
            aie.second.check();
        }

        for(const adjacency::intervals::entry &aie: right)
        {
            aie.second.check();
        }
    }

    void lane::auto_scale_memberships()
    {
        std::vector<float>  mlengths;
        mlengths.push_back(0);
        for(const road_membership::intervals::entry &rmie: road_memberships)
        {
            mlengths.push_back(rmie.second.length() + mlengths.back());
        }
        const float inv_len = 1.0f/mlengths.back();
        road_membership::intervals new_rm;

        std::vector<float>::iterator current_len = mlengths.begin();
        road_membership::intervals::const_iterator it = road_memberships.begin();
        for(; current_len != mlengths.end() && it != road_memberships.end();
            ++current_len, ++it)
        {
            new_rm.insert(inv_len*(*current_len), it->second);
        }

        road_memberships = new_rm;
    }

    template <typename T>
    static inline T lerp(const T x, const T a, const T b)
    {
        return x*(b-a)+a;
    }

    template <typename T>
    static inline T unlerp(const T x, const T a, const T b)
    {
        return (x-a)/(b-a);
    }

    static void rescale_tex_coords(const std::vector<vertex>::iterator &start, const std::vector<vertex>::iterator &end, const vec2f &dest_range, const vec2f &src_range)
    {
//         BOOST_FOREACH(vertex &v, std::make_pair(start, end))
//         {
//             v.tex_coord[0] = lerp(unlerp(v.tex_coord[0], src_range[0], src_range[1]),
//                                   dest_range[0], dest_range[1]);
//         }
      
        for(auto iter = start; iter != end; iter++)
	{
	  iter->tex_coord[0] = lerp(unlerp(iter->tex_coord[0], src_range[0], src_range[1]),
                                  dest_range[0], dest_range[1]);
	}
    }

    void lane::make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, const float lane_width, const float resolution) const
    {
        const int start_high = static_cast<int>(verts.size());

        std::vector<vertex> new_verts;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            const size_t last = new_verts.size();
            rm.parent_road->rep.extract_center(new_verts, rm.interval, rm.lane_position-lane_width*0.5, resolution);

            const vec2f interval(rm.parent_road->rep.length(rm.interval[0], rm.lane_position),
                                 rm.parent_road->rep.length(rm.interval[1], rm.lane_position));

            vec2f cont_interval(road_memberships.containing_interval(current));
            cont_interval[0] = length()*cont_interval[0];
            cont_interval[1] = length()*cont_interval[1];

            rescale_tex_coords(std::next(new_verts.begin(), last), new_verts.end(), cont_interval, interval);
        }

        verts.insert(verts.end(), new_verts.begin(), new_verts.end());
        new_verts.clear();

        const int end_high = static_cast<int>(verts.size());
        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const size_t last = new_verts.size();
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            rm.parent_road->rep.extract_center(new_verts, rev_interval, rm.lane_position+lane_width*0.5, resolution);

            const vec2f interval(rm.parent_road->rep.length(rm.interval[0], rm.lane_position),
                                 rm.parent_road->rep.length(rm.interval[1], rm.lane_position));

            vec2f cont_interval(road_memberships.containing_interval(current));
            cont_interval[0] = length()*cont_interval[0];
            cont_interval[1] = length()*cont_interval[1];

            rescale_tex_coords(std::next(new_verts.begin(), last), new_verts.end(), cont_interval, interval);
        }

        verts.insert(verts.end(), new_verts.begin(), new_verts.end());
        
        for(auto iter = std::next(verts.begin(), start_high); iter != std::next(verts.begin(), start_high); iter++)
        {
            iter->tex_coord[1] = 0.0f; 
        }
        
	for(auto iter = std::next(verts.begin(), end_high); iter != verts.end(); iter++)
        {
            iter->tex_coord[1] = 1.0f;
        }

        ::make_mesh(faces, verts, vec2i(static_cast<int>(start_high), static_cast<int>(end_high)), vec2i(static_cast<int>(verts.size()), static_cast<int>(end_high)));
    }

    path lane::svg_arc_path(const float lane_width) const
    {
        path res;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            res.append(rm.parent_road->rep.svg_arc_path_center(rm.interval, rm.lane_position-lane_width*0.5));
        }

        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            res.append(rm.parent_road->rep.svg_arc_path_center(rev_interval, rm.lane_position+lane_width*0.5));
        }

        return res;
    }

    path lane::svg_poly_path(const float lane_width) const
    {
        path res;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            res.append(rm.parent_road->rep.svg_poly_path_center(rm.interval, rm.lane_position-lane_width*0.5));
        }

        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            res.append(rm.parent_road->rep.svg_poly_path_center(rev_interval, rm.lane_position+lane_width*0.5));
        }

        return res;
    }

    float lane::length     () const
    {
        float total = 0.0f;
        for(const road_membership::intervals::entry &rmie: road_memberships)
        {
            total += rmie.second.length();
        }

        return total;
    }

    vec3f   lane::point      (float t, const float offset, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point(local, offset, up);
    }

    mat3x3f lane::frame      (float t, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.frame(local, up);
    }

    mat4x4f lane::point_frame(float t, const float offset, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point_frame(local, offset, up);
    }

    vec3f lane::point_theta(float &theta, float t, const float offset, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point_theta(theta, local, offset, up);
    }

    lane::serial_state lane::serial() const
    {
        return serial_state(*this);
    }

    lane *lane::left_adjacency(float &param) const
    {
        float local;
        adjacency::intervals::const_iterator adii = left.find_rescale(param, local);
        if(adii == left.end())
            return 0;

        const adjacency &adj = adii->second;
        if(adj.neighbor)
            param = local * (adj.neighbor_interval[1] - adj.neighbor_interval[0]) + adj.neighbor_interval[0];
        return adj.neighbor;
    }

    lane *lane::right_adjacency(float &param) const
    {
        float local;
        adjacency::intervals::const_iterator adii = right.find_rescale(param, local);
        if(adii == right.end())
            return 0;

        const adjacency &adj = adii->second;
        if(adj.neighbor)
            param = local * (adj.neighbor_interval[1] - adj.neighbor_interval[0]) + adj.neighbor_interval[0];
        return adj.neighbor;
    }

    lane *lane::upstream_lane()   const
    {
        assert(start);
        return start->incident(true);
    }

    lane *lane::downstream_lane() const
    {
        assert(end);
        return end->incident(false);
    }
}
