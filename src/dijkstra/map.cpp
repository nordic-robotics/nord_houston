#include "map.hpp"
#include <set>

namespace dijkstra
{
    class priority_compare
    {
    public:
        bool operator()(const std::pair<point const*, float>& a,
                        const std::pair<point const*, float>& b)
        {
            if (b.second < a.second)
                return true;
            if (a.second < b.second)
                return false;

            if (b.first < a.first)
                return true;
            return false;
        }
    };

    void map::connect(unsigned int a, unsigned int b)
    {
        graph[a].connect(&graph[b]);
        graph[b].connect(&graph[a]);
    }

    void map::precompute(const std::vector<point>& points)
    {
        for (auto& p1 : points)
        {
            auto start = closest(p1);
            std::vector<point const*> goals;
            goals.reserve(points.size());
            // find the closest for each goal
            std::transform(points.begin(), points.end(),
                           std::back_inserter(goals),
                           [&](const point& x) { return closest(x); });

            explore(start, goals);
        }
    }

    const std::vector<point>& map::get_graph() const { return graph; }

    path& map::find(const point& start_approx, const point& goal_approx)
    {
        auto start = closest(start_approx);
        auto goal = closest(goal_approx);

        // if it already exists: it's memoized, return it
        auto it = paths.find(std::make_pair(start, goal));
        if (it != paths.end())
        {
            return it->second;
        }

        // otherwise do some exploring
        explore(start, {goal});

        // entry should now exist
        return paths[std::make_pair(start, goal)];
    }

    point const* map::closest(const point& p_approx)
    {
        point const* p = nullptr;
        float min_p = INFINITY;

        for (auto& node : graph)
        {
            float dist = p_approx.distance(node);
            if (dist < min_p)
            {
                min_p = dist;
                p = &node;
            }
        }

        return p;
    }

    void map::explore(point const* start, std::vector<point const*> goals)
    {
        std::set<std::pair<point const*, float>, priority_compare> unvisited;
        std::unordered_map<point const*, float> visited;
        std::unordered_map<point const*, point const*> previous;

        unvisited.emplace(start, 0);
        visited.emplace(start, 0);

        while (!unvisited.empty())
        {
            // pop the smallest one
            auto current = unvisited.begin();

            for (auto link : current->first->get_links())
            {
                //TODO: check if memoized first

                if (visited.find(link) != visited.end())
                    continue;

                auto tentative = current->second + current->first->distance(link);
                auto it = std::find_if(unvisited.begin(), unvisited.end(),
                    [&](std::pair<point const*, float> kvp) {
                        return kvp.first == link;
                    });
                // if the node is in the 'unvisited' map
                if (it != unvisited.end())
                {
                    // use the shorter path of the previous and the current
                    if (tentative < it->second)
                    {
                        unvisited.emplace(link, tentative);
                        previous[link] = current->first;
                    }
                }
                else
                {
                    // if it's not in the map, add it
                    unvisited.emplace(link, tentative);
                    previous[link] = current->first;
                }

                visited[link] = current->second;

                // if this is a goal node
                auto goals_it = std::find(goals.begin(), goals.end(), link);
                if (goals_it != goals.end())
                {
                    // remove it as a goal
                    goals.erase(goals_it);

                    // if it was the last goal
                    if (goals.size() == 0)
                    {
                        // we done yo
                        insert_all(visited, previous);
                        return;
                    }
                }
            }
            unvisited.erase(current);
        }
    }

    void map::insert_all(const std::unordered_map<point const*, float>& visited,
                    const std::unordered_map<point const*, point const*>& previous)
    {
        // inserts all subpaths of a path

        // every node in visited is known to be optimal
        for (auto& node : visited)
        {
            path p;
            p.push_back(node.first);

            auto it = previous.end();
            while ((it = previous.find(p.front())) != previous.end())
            {
                // add one node from the path

                // TODO: don't recompute full path
                // maybe incorporate into push_back and add a pop_front
                p.insert(p.begin(), it->second);
                p.compute_length();
                insert(p);

                path p2 = p;
                while (p2.size() > 2)
                {
                    // and remove on the other end until done
                    p2.pop_back();
                    p2.compute_length();
                    insert(p2);
                }
            }
        }
    }

    void map::insert(const path& p)
    {
        paths[std::make_pair(p.front(), p.back())] = p;
        paths[std::make_pair(p.back(), p.front())] = p;
    }
}
