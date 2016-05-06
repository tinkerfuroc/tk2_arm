#ifndef __TINKER_ARM_BASE_ASTAR_PLANNER_H__
#define __TINKER_ARM_BASE_ASTAR_PLANNER_H__

#include <tk_arm/base_arm_planner.h>
#include <set>

namespace tinker {
namespace arm {

class BaseAStarPlanner : public BaseArmPlanner {
public:
    BaseAStarPlanner();
    virtual nav_msgs::Path GetPath(const geometry_msgs::Point &start_point,
                                   const geometry_msgs::Point &target_point,
                                   bool &success);
    virtual ~BaseAStarPlanner() {}

protected:
    virtual bool CompareState(const ArmStatePtr &lhs, const ArmStatePtr &rhs);
    virtual std::vector<ArmStatePtr> GetNeighbours(
        ArmStatePtr now_state,
        const std::vector<std::vector<std::vector<bool> > > &closed_grids);
    // Do not return the whole moved score evaluation but the increment of moved
    // score
    virtual int StateIncrementEval(const ArmStatePtr state);
    virtual int StateScorePredict(const ArmStatePtr state, 
                                  const GridPoint &target_grid);

protected:
    virtual bool HasReachedTarget(const ArmStatePtr state,
                                  const GridPoint &target_grid);

private:
    double distance_factor_;
    double regularity_factor_;
    GridPoint target_grid_;
    int seq_;
};
}
}

#endif
