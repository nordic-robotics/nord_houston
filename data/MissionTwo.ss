#define announce_range 0.4f
#define max_xy_stddev 0.015f
#define max_theta_stddev 0.015f

#var bool unsure = true
#var float time_left = 180
#var std::vector<point<2>> path
#var point<2> exit_point
#var std::vector<std::pair<std::string, point<2>>> unannounced
#var nord_messages::PoseEstimate pose

#param mission_two_behaviour<MissionTwo>& b

selector MissionTwo:
    sequence Unsure:
        cond unsure
        call b.spin(4.0, 1.5)
        sequence CheckSureness:
            cond pose.x.stddev < #max_xy_stddev
            cond pose.y.stddev < #max_xy_stddev
            cond pose.theta.stddev < #max_theta_stddev
            call (unsure = false, true)
            call (path = b.plan_path(point<2>(pose.x.mean, pose.y.mean), exit_point, false), true)
            call b.set_num_particles(10000)
    sequence Timeout:
        cond time_left < b.distance_to_exit_heuristic()
        call (path = b.plan_path(point<2>(pose.x.mean, pose.y.mean), exit_point, true), true)
    sequence Classify:
        cond unannounced.size() > 0
        cond b.distance_to(unannounced.front().second) < #announce_range
        call b.announce(unannounced.front().first)
        call b.record_evidence(unannounced.front().first)
        call (unannounced.erase(unannounced.begin()), true)
    sequence Move:
        cond path.size() > 0
        call b.go_to(path.front())
        call (path.erase(path.begin()), true)
