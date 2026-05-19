#include "SubdividedSchedule.hpp"

SubdividedSchedule::SubdividedSchedule(int num_robots, int makespan) {
    positions.resize(num_robots);
    K = makespan * 2;
}