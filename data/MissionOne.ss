#define classify_range 0.6f

#var float align_range = 0.56f
#var float time_left = 300
#var std::vector<point<2>> path
#var std::vector<point<2>> path_dijkstra
#var std::vector<std::pair<size_t, point<2>>> unknown
#var std::vector<std::string> classified
#var std::vector<std::string> announced
#var nord_messages::PoseEstimate pose
#var point<2> exit_point

#param mission_one_behaviour<MissionOne>& b

selector MissionOne:
    sequence Fuck:
        cond path.size() > 0
        cond b.distance_to(path.front()) < 0.015
        call (path.erase(path.begin()), true)
    sequence Timeout:
        cond time_left < b.distance_to_exit_heuristic()
        cond false
        call (path = b.plan_path(point<2>(pose.x.mean, pose.y.mean), exit_point, true), true)
    sequence Classify:
        cond unknown.size() > 0
        cond b.distance_to(unknown.front().second) < #classify_range
        call b.classify(unknown.front().first)
        cond classified.size() > 0
        call b.announce()
    sequence Dijkstra:
        cond path_dijkstra.size() > 0
        call b.go_to(path_dijkstra.front())
        call (path_dijkstra.erase(path_dijkstra.begin()), true)
    sequence Explore:
        cond path.size() > 0
        call (path_dijkstra = b.plan_path(point<2>(pose.x.mean, pose.y.mean), path.front(), true), true)
        call (path.erase(path.begin()), true)
    sequence GoHome:
        call (path = b.plan_path(point<2>(pose.x.mean, pose.y.mean), exit_point, true), true)
    call false