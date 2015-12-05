#include "point.hpp"
#include <cmath>
#include <iostream>

namespace dijkstra
{
    float point::distance(point const* other) const
    {
        auto dx = (other->x - x);
        auto dy = (other->y - y);
        return sqrt(dx * dx + dy * dy);
    }
    float point::distance(const point& other) const
    {
        return distance(&other);
    }

    const std::vector<point*>& point::get_links() const
    {
        return links;
    }

    void point::connect(point* other)
    {
        links.push_back(other);
        //other->links.push_back(this);
    }

    bool operator==(const point& lhs, const point& rhs)
    {
        return lhs.x == rhs.x
             & lhs.y == rhs.y;
    }

    std::ostream& operator<<(std::ostream& os, const point& p)
    {
        os << "(" << p.x << ", " << p.y << ")";
        return os;
    }
}
