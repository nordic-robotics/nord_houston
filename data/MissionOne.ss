#define classify_range 0.6f

#var float align_range = 0.56f
#var float time_left = 300
#var std::vector<point<2>> path
#var std::vector<std::pair<size_t, point<2>>> unknown
#var std::vector<std::string> classified
#var std::vector<std::string> announced
#var nord_messages::PoseEstimate pose

#param mission_one_behaviour<MissionOne>& b

selector MissionOne:
    sequence Timeout:
        cond time_left < b.distance_to_exit_heuristic()
        call b.exit()
    sequence Classify:
        cond false
        cond unknown.size() > 0
        cond b.distance_to(unknown.front().second) < #classify_range
        call b.align(unknown.front().second)
        call b.classify(unknown.front().first)
        cond classified.size() > 0
        call b.announce()
    sequence Explore:
        cond path.size() > 0
        call b.go_to(path.front())
        call (path.erase(path.begin()), true)
    call b.exit()
