#include "hwm_network.hpp"

namespace hwm
{
    void network_aux::build_spatial()
    {
        road_space.build(net.lane_width, rrm);
    }

    network_aux::road_spatial::entry::entry() : lc(0)
    {
    }

    network_aux::road_spatial::entry::entry(network_aux::road_rev_map::lane_cont *in_lc, const aabb2d &in_rect) : lc(in_lc), rect(in_rect)
    {
    }

    network_aux::road_spatial::road_spatial() : tree(0)
    {
    }

    network_aux::road_spatial::~road_spatial()
    {
        if(tree)
            delete tree;
    }

    void network_aux::road_spatial::build(float lane_width, strhash<network_aux::road_rev_map>::type &roads)
    {
        if(tree)
            delete tree;

        std::vector<rtree2d::entry> leaves;
        typedef strhash<road_rev_map>::type::value_type rrm_pair;
        for(rrm_pair &rp: roads)
        {
            for(partition01<road_rev_map::lane_cont>::iterator current = rp.second.lane_map.begin(); current != rp.second.lane_map.end(); ++current)
            {
                const vec2f interval(rp.second.lane_map.containing_interval(current));
                const aabb2d rect(current->second.planar_bounding_box(lane_width, interval));
                leaves.push_back(rtree2d::entry(rect, items.size()));
                items.push_back(entry(&(current->second), rect));
            }
        }

        tree = rtree2d::hilbert_rtree(leaves);
    }

    std::vector<network_aux::road_spatial::entry> network_aux::road_spatial::query(const aabb2d &rect) const
    {
        std::vector<entry> res;
        if(tree)
        {
            const std::vector<size_t> q_res(tree->query(rect));
            for(const size_t &i: q_res)
            {
                res.push_back(items[i]);
            }
        }

        return res;
    }
}
