#pragma once

// Translate an algorithm's output timestamps into bag time.
//
// The benchmark never changes an algorithm's own clock: an algorithm may, for
// example, stamp its outputs with ros::Time::now(), which is wall-clock time
// at processing, and its internal logic may depend on that. The HDMapping result,
// however, must be in bag time so that it can be aligned with the ground truth.
//
// The run script therefore records /clock next to the algorithm's outputs.
// "rosbag play --clock" publishes bag time on /clock, and rosbag record stores
// every message at its wall-clock receive time, so for each /clock message
//     receive time - payload = wall-clock minus bag time
// which is constant while the bag plays at normal rate. Subtracting that one
// constant from every pose and point stamp moves the result into bag time and
// keeps all relative timing, and the point-to-pose association, intact.

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <vector>

struct BagTimeShift
{
    bool apply = false;     // true: subtract offset_ns from every stamp
    int64_t offset_ns = 0;  // wall-clock minus bag time, from /clock
    const char *reason = "";
};

inline int64_t median_ns(std::vector<int64_t> values)
{
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
}

// clock_wall_minus_bag: for each recorded /clock message, its receive time
//                       minus its payload, in nanoseconds.
// stamp_minus_receive:  for each recorded output message (odometry), its
//                       header stamp minus its receive time, in nanoseconds.
inline BagTimeShift decide_bag_time_shift(const std::vector<int64_t> &clock_wall_minus_bag,
                                          const std::vector<int64_t> &stamp_minus_receive)
{
    BagTimeShift shift;
    if (clock_wall_minus_bag.empty())
    {
        shift.reason = "no /clock messages in the recording, timestamps left in the algorithm's time base";
        return shift;
    }
    if (stamp_minus_receive.empty())
    {
        shift.reason = "no output messages in the recording";
        return shift;
    }

    shift.offset_ns = median_ns(clock_wall_minus_bag);
    const int64_t lag_ns = median_ns(stamp_minus_receive);

    // Stamps close to their own wall-clock receive time mean the algorithm
    // works in wall-clock time: translate. Stamps close to bag time (lag near
    // minus the offset) mean they are already in bag time: leave them, so the
    // translation can never be applied twice.
    const long long distance_to_wall = std::llabs(static_cast<long long>(lag_ns));
    const long long distance_to_bag = std::llabs(static_cast<long long>(lag_ns + shift.offset_ns));
    if (distance_to_wall < distance_to_bag)
    {
        shift.apply = true;
        shift.reason = "output stamped in wall-clock time, translated to bag time";
    }
    else
    {
        shift.reason = "output already in bag time, no translation needed";
    }
    return shift;
}
