#pragma once

#include "point.hpp"
#include <vector>
#include <cmath>

namespace dijkstra
{
    class path : public std::vector<point const*>
    {
    public:
        path() : cost(NAN) { };

        void compute_length();
        float length() const;

    private:
        float cost;
    };
}
