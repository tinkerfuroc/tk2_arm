#ifndef __TINKER_BASE_ARM_PLANNER_H__
#define __TINKER_BASE_ARM_PLANNER_H__

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/functional.hpp>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include "tk_arm/arm_ik.h"
#include <string>

namespace tinker {
namespace arm {

struct ArmState;
typedef boost::shared_ptr<ArmState> ArmStatePtr;

struct GridPoint {
    int x;
    int y;
    int z;
    bool operator<(const GridPoint &rhs) {
        if (x < rhs.x) return true;
        if (x > rhs.x) return false;
        if (y < rhs.y) return true;
        if (y > rhs.y) return false;
        if (z < rhs.z) return true;
        if (z > rhs.z) return false;
        return false;
    }
};

struct ArmState {
public:
    ArmState(const GridPoint &grid_point, ArmStatePtr prev_state)
        : point(grid_point), prev(prev_state), moved_score(0) {}
    GridPoint point;
    ArmStatePtr prev;
    double moved_score;
};

class BaseArmPlanner {
public:
    BaseArmPlanner();
    virtual nav_msgs::Path GetPath(const geometry_msgs::Point &start_point,
                                   const geometry_msgs::Point &target_point,
                                   bool &success) = 0;
    GridPoint ToGrid(const geometry_msgs::Point &point);
    geometry_msgs::Point ToPoint(const GridPoint &grid);
    virtual ~BaseArmPlanner() {}
    bool Invalid(const GridPoint &grid);
    double Distance(const GridPoint &from_grid, const GridPoint &to_grid);

protected:
    std::vector<std::vector<std::vector<bool> > > invalid_map_;
    ros::NodeHandle private_nh_;
    int x_size_;
    int y_size_;
    int z_size_;
    double grid_x_min_;
    double grid_x_max_;
    double grid_y_min_;
    double grid_y_max_;
    double grid_z_min_;
    double grid_z_max_;
    double grid_size_;
    int min_possible_z_;
    int max_possible_z_;
    int min_possible_x_;
    int max_possible_x_;
    std::string map_filename_;
    ArmIK arm_ik_;

private:
    void BuildValidMap();
    void SaveValidMap(const std::string &filename);
    bool LoadValidMap(const std::string &filename);
};
}
}

#endif
