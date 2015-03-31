#include "sumo_network.hpp"

namespace sumo
{
    bool network::check_edge(const edge &e) const
    {
        return (e.to != e.from);
    }

    bool network::check_node(const node &n) const
    {
        return !n.id.empty();
    }

    bool network::check() const
    {
        typedef std::pair<str, node> nmap_pair;
        for(const nmap_pair &np: nodes)
        {
            if(!check_node(np.second))
                return false;
        }

        typedef std::pair<str, edge> emap_pair;
        for(const emap_pair &ep: edges)
        {
            if(!check_edge(ep.second))
                return false;
        }
        return true;
    }
}
