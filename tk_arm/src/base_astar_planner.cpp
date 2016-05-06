#include "tk_arm/base_astar_planner.h"
#include <deque>
#include <set>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <cmath>

using std::set;
using boost::bind;
using boost::function;
using std::vector;
using std::deque;

namespace tinker {
namespace arm {

BaseAStarPlanner::BaseAStarPlanner() : seq_(0) {
    private_nh_.param("distance_factor", distance_factor_, 0.7);
    private_nh_.param("regularity_factor", regularity_factor_, 0.05);
    private_nh_.param("z_factor", z_factor_, 0.3);
}

nav_msgs::Path BaseAStarPlanner::GetPath(
    const geometry_msgs::Point &start_point,
    const geometry_msgs::Point &target_point, bool &success) {
    success = false;
    vector<vector<vector<bool> > > closed_grids = invalid_map_;
    set<ArmStatePtr, function<bool(const ArmStatePtr &, const ArmStatePtr &)> >
        open_states(bind(&BaseAStarPlanner::CompareState, this, _1, _2));
    GridPoint start_grid = ToGrid(start_point);
    ROS_INFO("Start Grid at %d %d %d", start_grid.x, start_grid.y, start_grid.z);
    ROS_INFO("Start at %f %f %f", start_point.x, start_point.y, start_point.z);
    ROS_INFO("Start at %f %f %f", target_point.x, target_point.y, target_point.z);
    target_grid_ = ToGrid(target_point);
    if (target_grid_.z < min_possible_z_)
        target_grid_.z = min_possible_z_;
    if (target_grid_.z > max_possible_z_)
        target_grid_.z = max_possible_z_;
    ROS_INFO("Target Grid at %d %d %d", target_grid_.x, target_grid_.y, target_grid_.z);
    if (Invalid(start_grid)) {
        ROS_WARN("BaseAStarPlanner: Start grid not valid");
        return nav_msgs::Path();
    }
    open_states.insert(ArmStatePtr(new ArmState(start_grid, ArmStatePtr())));
    while (true) {
        if (open_states.empty()) {
            ROS_WARN("BaseAStarPlanner: Failed to make plan");
            return nav_msgs::Path();
        }
        ArmStatePtr now_state = *open_states.begin();
        if (HasReachedTarget(now_state, target_grid_)) {
            nav_msgs::Path path;
            path.header.seq = seq_++;
            path.header.frame_id = "arm_origin_link";
            path.header.stamp = ros::Time::now();
            deque<ArmStatePtr> route_states;
            while (now_state) {
                route_states.push_front(now_state);
                now_state = now_state->prev;
            }
            BOOST_FOREACH (ArmStatePtr state, route_states) {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position = ToPoint(state->point);
                path.poses.push_back(pose);
            }
            return path;
        }
        open_states.erase(open_states.begin());
        vector<ArmStatePtr> neighbour_states =
            GetNeighbours(now_state, closed_grids);
        BOOST_FOREACH (ArmStatePtr state, neighbour_states) {
            open_states.insert(state);
        }
        closed_grids[now_state->point.x][now_state->point.y][now_state->point
                                                                 .z] = true;
    }
}

bool BaseAStarPlanner::HasReachedTarget(const ArmStatePtr state,
                                        const GridPoint &target_grid) {
    if (state->point.x == target_grid.x && state->point.y == target_grid.y &&
        state->point.z == target_grid.z) {
        return true;
    }
    if (state->point.z == target_grid.z) {
        int x = state->point.x;
        int y = state->point.y;
        int z = state->point.z;
        if (invalid_map_[x - 1][y][z] || invalid_map_[x + 1][y][z] ||
            invalid_map_[x][y - 1][z] || invalid_map_[x][y + 1][z]) {
            return true;
        }
    }
    return false;
}

std::vector<ArmStatePtr> BaseAStarPlanner::GetNeighbours(
    ArmStatePtr now_state, const vector<vector<vector<bool> > > &closed_grids) {
    int x = now_state->point.x;
    int y = now_state->point.y;
    int z = now_state->point.z;
    vector<ArmStatePtr> neighbours;
    for (int i = -1; i < 2; i++) {
        for (int j = -1; j < 2; j++) {
            for (int k = -1; k < 2; k++) {
                if (i == 0 && j == 0 && k == 0) continue;
                if (!closed_grids[x + i][y + j][z + k]) {
                    GridPoint new_grid = {.x = x + i, .y = y + j, .z = z + k};
                    ArmStatePtr new_state =
                        ArmStatePtr(new ArmState(new_grid, now_state));
                    new_state->moved_score = now_state->moved_score + StateIncrementEval(new_state);
                    neighbours.push_back(new_state);
                }
            }
        }
    }
    return neighbours;
}

double BaseAStarPlanner::StateIncrementEval(const ArmStatePtr state) {
    ArmStatePtr prev_state = state->prev;
    double increment = 0;
    if (!prev_state) return increment;
    increment = distance_factor_ * Distance(state->point, prev_state->point);
    ArmStatePtr grand_state = prev_state->prev;
    if (!grand_state) return increment;
    int x1 = state->point.x - prev_state->point.x;
    int y1 = state->point.y - prev_state->point.y;
    int z1 = state->point.z - prev_state->point.z;
    int x2 = prev_state->point.x - grand_state->point.x;
    int y2 = prev_state->point.y - grand_state->point.y;
    int z2 = prev_state->point.z - grand_state->point.z;
    if (y1 * z2 != y2 * z1 || x2 * z1 != x1 * z2 || x1 * y2 != x2 * y1)
        increment += regularity_factor_;
    return increment;
}

double BaseAStarPlanner::StateScorePredict(const ArmStatePtr state,
                                        const GridPoint &target_grid) {
    return distance_factor_ * Distance(state->point, target_grid)
        + z_factor_ * fabs(double(state->point.z - target_grid.z));
}

bool BaseAStarPlanner::CompareState(const ArmStatePtr &lhs, const ArmStatePtr &rhs) {
    return (lhs->moved_score + StateScorePredict(lhs, target_grid_)) < 
        (rhs->moved_score + StateScorePredict(rhs, target_grid_));
}
}
}
