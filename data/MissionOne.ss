#define classify_range 0.8f

#var float align_range = 0.6f
#var float time_left
#var std::vector<point<2>> path
#var std::vector<point<2>> unknown
#var std::vector<nord_messages::Classification> classified
#var std::vector<nord_messages::Classification> announced
#var nord_messages::PoseEstimate pose

#param mission_one_behaviour<MissionOne>& b

selector MissionOne:
    sequence Timeout:
        cond time_left < b.distance_to_exit_heuristic()
        call b.exit()
    sequence Classify:
        cond unknown.size() > 0
        cond b.distance_to(unknown.front()) < #classify_range
        call b.align(unknown.front())
        call b.classify()
        cond classified.size() > 0
        call b.announce()
    sequence Explore:
        cond path.size() > 0
        call b.go_to(path.front())
        call (path.erase(path.begin()), true)
    call b.exit()
