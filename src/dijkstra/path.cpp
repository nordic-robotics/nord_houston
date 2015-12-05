#include "path.hpp"

namespace dijkstra
{
    void path::compute_length()
    {
        cost = 0;
        for (iterator it = begin(); (it + 1) != end(); ++it)
        {
            cost += (*it)->distance(*(it + 1));
        }
    }

    float path::length() const
    {
        return cost;
    }
}
