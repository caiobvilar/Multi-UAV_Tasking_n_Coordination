//
// Created by robor on 6/7/2020.
//

#ifndef AREACOVERAGE_MINIMUMTIME_H
#define AREACOVERAGE_MINIMUMTIME_H

#include "pch.h"
#define RESOLUTION 0.1
#define MAX_ITERATIONS 100

namespace TrajectoryPlanning
{
    using PATH = wykobi::polygon<double, 2>;
    using TRAJECTORIES = std::vector<wykobi::polygon<double, 2>>;
    using VELOCITIES = std::vector<double>;

    class MinimumTime {

    public:
        MinimumTime(const std::vector<POINT> &initPos, const std::vector<double> &maxVel);
        /**
         * @brief the key idea is to maximize velocity to minimize the path
         * @param paths a set of paths (each path consisting a set of waypoints)
         * @return a set of trajectories
         */
        std::vector<wykobi::polygon<double, 2>> solve(const std::vector<wykobi::polygon<double, 2>>& paths, int safe_distance);

        std::vector<double>get_velocities()
        {
            return max_vel_;
        }
    private:
        std::vector<POINT> init_pos_;
        std::vector<double> max_vel_;
        std::vector<PATH> uavs_;
        int safe_distance_;

    protected:
        /**
         * @brief find the minimum distance among initial position of paths and uavs
         * @param paths : a set of polygons
         * @return a map of path index and uav position
         */
        std::unordered_map<int,wykobi::point2d<double>> m_connect_uavs(const POLYGONS &paths);
        /**
         * @brief a set of trajectories is generated using max_vel_ parameter. \par
         * @param paths : a set of paths (each path consisting a set of waypoints)
         */
        void m_evaluate_trajectory(std::vector<wykobi::polygon<double, 2>>& paths);

        /**
         * @brief Given a reference trajectory for robot j, \par
         * here we find the parameter for the trajectory for robot i. \par
         * we fixed index j and evaluate i.
         * @param trajs: a set of trajectories for evaluation
         * @param i: index of robot i
         * @param j: index of robot j
         *
         * @remarks
         * this function invokes two private variables \par
         * 1) max_vel_: a set of default velocities
         * 2) uavs_:ground truth waypoints
         *
         * @note
         * we update the private variable max_vel_ instead of returning a value.
         */

        void m_binary_search(std::vector<wykobi::polygon<double, 2>>& trajs, int i, int j);

        wykobi::polygon<double, 2> m_gen_trajectory(const wykobi::polygon<double, 2>& path, int robot);

        bool m_all_collision_check(TRAJECTORIES &trajs);

        std::vector<double> m_pathLength();

    private:
        /**
         * @brief because of sequential decomposition, we need to checking only one neighbor robot \par
         * This step results significant computation time reduction.
         * @param r : trajectory for robot r
         * @param k : trajectory for robot k
         * @return True if collision else false
         */
        bool collision_check(const wykobi::polygon<double, 2>& r, const wykobi::polygon<double, 2>& k);

        std::tuple<VELOCITIES, TRAJECTORIES> bottom_up_iteration(const std::vector<double>& maxVel);
        std::tuple<VELOCITIES, TRAJECTORIES> top_down_iteration(const std::vector<double>& maxVel);

    };




}


#endif //AREACOVERAGE_MINIMUMTIME_H
