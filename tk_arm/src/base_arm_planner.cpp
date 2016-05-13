#include "tk_arm/base_arm_planner.h"
#include <cmath>
#include <cstdio>

using std::string;
using std::vector;

namespace tinker {
namespace arm {

BaseArmPlanner::BaseArmPlanner() : private_nh_("~") {
    private_nh_.param("x_min", grid_x_min_, 0.3);
    private_nh_.param("x_max", grid_x_max_, 1.);
    private_nh_.param("y_min", grid_y_min_, -1.);
    private_nh_.param("y_max", grid_y_max_, 1.);
    private_nh_.param("z_min", grid_z_min_, -0.5);
    private_nh_.param("z_max", grid_z_max_, 1.5);
    private_nh_.param("grid_size", grid_size_, 0.02);
    private_nh_.param("map_filename", map_filename_,
                      string("arm.map"));
    ROS_ASSERT(grid_x_max_ > grid_x_min_);
    ROS_ASSERT(grid_y_max_ > grid_y_min_);
    ROS_ASSERT(grid_z_max_ > grid_z_min_);
    x_size_ = ceil((grid_x_max_ - grid_x_min_) / grid_size_);
    y_size_ = ceil((grid_y_max_ - grid_y_min_) / grid_size_);
    z_size_ = ceil((grid_z_max_ - grid_z_min_) / grid_size_);
    invalid_map_.resize(x_size_);
    for (int i = 0; i < x_size_; i++) {
        invalid_map_[i].resize(y_size_);
        for (int j = 0; j < y_size_; j++) {
            invalid_map_[i][j].resize(z_size_, false);
        }
    }
    ROS_INFO("Try to load map");
    if (!LoadValidMap(map_filename_)) {
        ROS_INFO("Load map failed, building...");
        BuildValidMap();
        SaveValidMap(map_filename_);
    }
    ROS_INFO("Grid z min max: %d %d", min_possible_z_, max_possible_z_);
}

void BaseArmPlanner::BuildValidMap() {
    min_possible_z_ = z_size_;
    max_possible_z_ = 0;
    vector<bool> valid_z(z_size_, false);
    for (int i = 0; i < x_size_; i++) {
        printf("%d\r", i);
        fflush(stdout);
        for (int j = 0; j < y_size_; j++) {
            for (int k = 0; k < z_size_; k++) {
                GridPoint grid = {.x = i, .y = j, .z = k};
                KDL::JntArray jnt_arr;
                geometry_msgs::Point point = ToPoint(grid);
                if (point.z < ArmIK::kBaseHeightMin + ArmIK::kBaseHeightDiff)
                    point.z -= ArmIK::kBaseHeightMin;
                else if (point.z >
                         ArmIK::kBaseHeightMax + ArmIK::kBaseHeightDiff)
                    point.z -= ArmIK::kBaseHeightMax;
                else
                    point.z = ArmIK::kBaseHeightDiff;
                if (i == 0 || j == 0 || k == 0 || i == x_size_ - 1 ||
                    j == y_size_ - 1 || k == z_size_ - 1)
                    invalid_map_[i][j][k] = true;
                else {
                    invalid_map_[i][j][k] =
                        !arm_ik_.PositionToAngle(point, jnt_arr, 1);
                }
                valid_z[k] = valid_z[k] || (!invalid_map_[i][j][k]);
                if (!invalid_map_[i][j][k]) {
                    max_possible_z_ = k > max_possible_z_ ? k : max_possible_z_;
                    min_possible_z_ = k < min_possible_z_ ? k : min_possible_z_;
                }
            }
        }
    }
    printf("\n");
    for (int i = 0; i < z_size_; i++) {
        bool v = valid_z[i];
        printf("%d", v);
    }
    printf("\n");
}

geometry_msgs::Point BaseArmPlanner::ToPoint(const GridPoint &grid) {
    geometry_msgs::Point point;
    point.x = grid_x_min_ + grid.x * grid_size_;
    point.y = grid_y_min_ + grid.y * grid_size_;
    point.z = grid_z_min_ + grid.z * grid_size_;
    return point;
}

GridPoint BaseArmPlanner::ToGrid(const geometry_msgs::Point &point) {
    GridPoint grid;
    grid.x = (point.x - grid_x_min_) / grid_size_;
    grid.y = (point.y - grid_y_min_) / grid_size_;
    grid.z = (point.z - grid_z_min_) / grid_size_;
    return grid;
}

bool BaseArmPlanner::Invalid(const GridPoint &grid) {
    if (grid.x < 0 || grid.x >= x_size_) return true;
    if (grid.y < 0 || grid.y >= y_size_) return true;
    if (grid.z < 0 || grid.z >= z_size_) return true;
    return invalid_map_[grid.x][grid.y][grid.z];
}

double BaseArmPlanner::Distance(const GridPoint &from_grid,
                                const GridPoint &to_grid) {
    double x1 = from_grid.x;
    double y1 = from_grid.y;
    double z1 = from_grid.z;
    double x2 = to_grid.x;
    double y2 = to_grid.y;
    double z2 = to_grid.z;
    double x = x2 - x1;
    double y = y2 - y1;
    double z = z2 - z1;
    return sqrt(x * x + y * y + z * z) * grid_size_;
}

bool BaseArmPlanner::LoadValidMap(const string &filename) {
    FILE *fp = fopen(filename.c_str(), "r");
    if (!fp) return false;
    int x_size;
    int y_size;
    int z_size;
    double grid_x_min;
    double grid_x_max;
    double grid_y_min;
    double grid_y_max;
    double grid_z_min;
    double grid_z_max;
    double grid_size;
    fread(&grid_x_min, sizeof(double), 1, fp);
    fread(&grid_x_max, sizeof(double), 1, fp);
    fread(&grid_y_min, sizeof(double), 1, fp);
    fread(&grid_y_max, sizeof(double), 1, fp);
    fread(&grid_z_min, sizeof(double), 1, fp);
    fread(&grid_z_max, sizeof(double), 1, fp);
    fread(&grid_size, sizeof(double), 1, fp);
    fread(&x_size, sizeof(int), 1, fp);
    fread(&y_size, sizeof(int), 1, fp);
    fread(&z_size, sizeof(int), 1, fp);
    if (grid_x_min != grid_x_min_ || grid_x_max != grid_x_max_ ||
        grid_y_min != grid_y_min_ || grid_y_max != grid_y_max_ ||
        grid_z_min != grid_z_min_ || grid_z_max != grid_z_max_ ||
        grid_size != grid_size_ || x_size != x_size_ || y_size != y_size_ ||
        z_size != z_size_) 
        return false;
    for (int i = 0; i < x_size_; i++)
        for (int j = 0; j < y_size_; j++)
            for (int k = 0; k < z_size_; k++) {
                bool valid;
                fread(&valid, sizeof(bool), 1, fp);
                invalid_map_[i][j][k] = valid;
            }
    fread(&min_possible_z_, sizeof(int), 1, fp);
    fread(&max_possible_z_, sizeof(int), 1, fp);
    return true;
}

void BaseArmPlanner::SaveValidMap(const string &filename) {
    FILE *fp = fopen(filename.c_str(), "w");
    fwrite(&grid_x_min_, sizeof(double), 1, fp);
    fwrite(&grid_x_max_, sizeof(double), 1, fp);
    fwrite(&grid_y_min_, sizeof(double), 1, fp);
    fwrite(&grid_y_max_, sizeof(double), 1, fp);
    fwrite(&grid_z_min_, sizeof(double), 1, fp);
    fwrite(&grid_z_max_, sizeof(double), 1, fp);
    fwrite(&grid_size_, sizeof(double), 1, fp);
    fwrite(&x_size_, sizeof(int), 1, fp);
    fwrite(&y_size_, sizeof(int), 1, fp);
    fwrite(&z_size_, sizeof(int), 1, fp);
    for (int i = 0; i < x_size_; i++)
        for (int j = 0; j < y_size_; j++)
            for (int k = 0; k < z_size_; k++) {
                bool valid = invalid_map_[i][j][k];
                fwrite(&valid, sizeof(bool), 1, fp);
            }
    fwrite(&min_possible_z_, sizeof(int), 1, fp);
    fwrite(&max_possible_z_, sizeof(int), 1, fp);
    fclose(fp);
}
}
}
