#include "hwm_network.hpp"

namespace hwm
{
    void road::check() const
    {
        if(id.empty())
            throw std::runtime_error("Road with empty id");
        rep.check();
    }

    void road::translate(const vec3f &o)
    {
        rep.translate(o);
    }

    void road::bounding_box(vec3f &low, vec3f &high) const
    {
        return rep.bounding_box(low, high);
    }
}
