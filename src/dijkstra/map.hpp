#pragma once

#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>
#include "point.hpp"
#include "path.hpp"

namespace std {
    template <> struct hash<std::pair<dijkstra::point const*, dijkstra::point const*>>
    {
        size_t operator()(const std::pair<dijkstra::point const*,
                                          dijkstra::point const*>& var) const
        {
            return hash<dijkstra::point const*>()(var.first)
                ^ (hash<dijkstra::point const*>()(var.second) << 16);
        }
    };
}

namespace dijkstra
{
    class map
    {
    public:
        map() { };
        map(std::vector<point>&& graph) : graph(graph) { };
        map(const std::vector<point>& graph) : graph(graph) { };
        map(const map& other) = delete;
        
        void connect(unsigned int a, unsigned int b);
        void precompute(const std::vector<point>& points);

	const std::vector<point>& get_graph() const ;

        path& find(const point& start_approx, const point& goal_approx);

    private:
        point const* closest(const point& p_approx);
        void explore(point const* start, std::vector<point const*> goals);
        void insert_all(const std::unordered_map<point const*, float>& visited,
                        const std::unordered_map<point const*, point const*>& previous);
        void insert(const path& p);

        std::vector<point> graph;
        std::unordered_map<std::pair<point const*, point const*>, path> paths;
    };
}
