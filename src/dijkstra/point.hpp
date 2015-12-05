#pragma once

#include <vector>

namespace dijkstra
{
    struct point
    {
        point() : x(0), y(0) { };
        point(float x, float y) : x(x), y(y) { };

        float distance(point const* other) const;
        float distance(const point& other) const;
        const std::vector<point*>& get_links() const;
        void connect(point* other);

        float x;
        float y;

    private:
        std::vector<point*> links;
    };
}
