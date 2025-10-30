//
// Created by robor on 6/7/2020.
//

#include "Algorithm/MinimumTime.h"

using namespace TrajectoryPlanning;

MinimumTime::MinimumTime(const std::vector<POINT> &initPos, const std::vector<double> &maxVel)
        : init_pos_(initPos), max_vel_(maxVel) {
    std::cout<<"\n";
}

std::unordered_map<int, wykobi::point2d<double>>
MinimumTime::m_connect_uavs(const POLYGONS &paths) {
    // convert wykobi::point to geom::point which can calculate distance between two points

    std::unordered_map<int,wykobi::point2d<double>> result;
    std::vector<int> index_keeper;
    int path_index = 0;
    for(auto p:paths)
    {
        std::map<double, int> cost_map;
        int uav_index = 0;
        // sorting cost automatically using std::map
        for (auto q:init_pos_)
        {
            double d = distance(p[0], q);
            cost_map[d] = uav_index++;
        }
        //FIXME model as TSP problem
        // find the best uav to connect
        for(auto it = cost_map.begin(); it != cost_map.end(); ++it)
        {
            int uav_index = it->second;
//            std::cout<< "[check_index] "<< uav_index <<" cost "<< it->first <<std::endl;
            if(std::find(index_keeper.begin(),index_keeper.end(),uav_index) != index_keeper.end())
            {
                continue;
            }
//            std::cout<< "[uav_index] "<< uav_index <<" cost "<< it->first <<std::endl;
            result[path_index++] = init_pos_[uav_index];
            index_keeper.push_back(uav_index);
            break;
        }
    }


    return result;
}

std::vector<wykobi::polygon<double, 2>> MinimumTime::solve(const std::vector<wykobi::polygon<double, 2>> &paths, int safe_distance) {
    std::vector<wykobi::polygon<double, 2>>result;
    int N = paths.size();
    safe_distance_ = safe_distance;
    auto appendlist = m_connect_uavs(paths);
    for (int i = 0; i < N; ++i) {
        wykobi::polygon<double, 2> polygon;
        polygon.push_back(appendlist[i]);
        std::copy(paths[i].begin(),paths[i].end(),std::back_inserter(polygon));
        polygon.push_back(appendlist[i]);
        result.push_back(polygon);
    }
    uavs_.clear();
    VELOCITIES velocities1, velocities2, maxVel;
    std::copy(result.begin(),result.end(),std::back_inserter(uavs_));
    std::copy(max_vel_.begin(),max_vel_.end(),std::back_inserter(maxVel));
    auto path_len = m_pathLength();
    std::map<double, TRAJECTORIES > evals;
    std::map<double, VELOCITIES > evals_vel;



    for (int j = 0; j < N; ++j) {
        TRAJECTORIES candidate;

        std::copy(uavs_.begin(),uavs_.end(),std::back_inserter(candidate));

        for (int k = 0; k < N; ++k) {
            if(j==k)continue;
            m_evaluate_trajectory(candidate);
            if(m_all_collision_check(candidate))
            {
                max_vel_.clear();
                std::copy(maxVel.begin(),maxVel.end(),std::back_inserter(max_vel_));
                m_binary_search(candidate,k, j);
                m_evaluate_trajectory(candidate);
                continue;
            }
            else
            {
//                double score = std::accumulate(max_vel_.begin(),max_vel_.end(),0.0);
                double score = 0;
                for (int i = 0; i < 3; ++i) {
                    score += path_len[i]/max_vel_[i];
                }

                evals[score] = candidate;
                evals_vel[score] = max_vel_;
                std::cout<< "[TrajGen]: score "<< score << ": ";
                std::copy(max_vel_.begin(),max_vel_.end(),std::ostream_iterator<double>(std::cout, " "));
                std::cout<<"\n";
            }
        }
    }

    max_vel_.clear();

    // first element in the map is the result
    auto it = evals.begin();
    std::copy(evals_vel[it->first].begin(),evals_vel[it->first].end(),std::back_inserter(max_vel_));
    return it->second;

}

void MinimumTime::m_evaluate_trajectory(std::vector<wykobi::polygon<double, 2>> &paths) {

    //FIXME incorporate dt

    for (int i = 0; i < paths.size(); ++i) {
        double vel = max_vel_[i];
        auto path = paths[i];
        wykobi::polygon<double, 2> traj;
        for (int j = 0; j < path.size() - 1; ++j) {
            auto a = path[j];
            auto b = path[j+1];
            double dist = distance(a, b);

            int step = dist / vel;
            arma::vec xx = arma::linspace(path[j].x, path[j+1].x, step);
            arma::vec yy = arma::linspace(path[j].y, path[j+1].y, step);
            int N = xx.size();

            for (int k = 0; k < N; ++k) {
                traj.push_back(wykobi::make_point(xx(k),yy(k)));
            }

        }
        paths[i] = traj;

    }

}

void MinimumTime::m_binary_search(std::vector<wykobi::polygon<double, 2>> &trajs, int i, int j) {

    auto V_MAX = max_vel_[i];
    int SIZE = V_MAX/RESOLUTION;
    std::vector<double> ivec(SIZE);
    double start = 0.0;
    std::generate(ivec.begin(), ivec.end(), [&start](){start+=RESOLUTION; return start;});
    TRAJECTORIES result;
    std::copy(trajs.begin(),trajs.end(),std::back_inserter(result));
    bool collide = true;
    while(max_vel_[i]>2*RESOLUTION && collide)
    {

        max_vel_[i] -= RESOLUTION;
        m_evaluate_trajectory(result);
        auto r = m_gen_trajectory(uavs_[i], i);
        auto k = trajs[j];
        collide = m_all_collision_check(result);
    }
    trajs.clear();
    std::copy(result.begin(),result.end(),std::back_inserter(trajs));


}

wykobi::polygon<double, 2> MinimumTime::m_gen_trajectory(const wykobi::polygon<double, 2> &path, int robot) {
    wykobi::polygon<double, 2> traj;
    for (int j = 0; j < path.size() - 1; ++j) {
        auto a = path[j];
        auto b = path[j+1];
        double dist = distance(a, b);


        int step = dist / max_vel_[robot];
        arma::vec xx = arma::linspace(path[j].x, path[j+1].x, step);
        arma::vec yy = arma::linspace(path[j].y, path[j+1].y, step);
        int N = xx.size();

        for (int k = 0; k < N; ++k) {
            traj.push_back(wykobi::make_point(xx(k),yy(k)));
        }

    }
    return traj;
}

bool MinimumTime::collision_check(const wykobi::polygon<double, 2> &r, const wykobi::polygon<double, 2> &k)
{
    int max_step = std::max(r.size(), k.size());
    for (int t = 0; t < max_step; ++t) {
        int step1 = std::min(t,int(r.size()));
        int step2 = std::min(t,int(k.size()));
        if(distance(r[step1], r[step2])<=safe_distance_)
            return true;
    }
    return false;
}

std::tuple<VELOCITIES, TRAJECTORIES> MinimumTime::bottom_up_iteration(const std::vector<double> &maxVel) {

    TRAJECTORIES result;
    max_vel_.clear();
    std::copy(uavs_.begin(),uavs_.end(),std::back_inserter(result));
    std::copy(maxVel.begin(),maxVel.end(),std::back_inserter(max_vel_));
    m_evaluate_trajectory(result);
    int j = result.size()-1;
    int count = 0;
    do
    {
        if(collision_check(result[j-1],result[j]))
        {
            m_binary_search(result,j-1, j);
            m_evaluate_trajectory(result);
        } else
            --j;
    }while (j>0 && ++count<MAX_ITERATIONS);
    if(count==MAX_ITERATIONS)
        std::cerr<<"[TrajPlanner]"<<"No solution found \n";

    return std::make_tuple(max_vel_, result);
}

std::tuple<VELOCITIES, TRAJECTORIES> MinimumTime::top_down_iteration(const std::vector<double> &maxVel) {
    TRAJECTORIES result;
    max_vel_.clear();
    std::copy(uavs_.begin(),uavs_.end(),std::back_inserter(result));
    std::copy(maxVel.begin(),maxVel.end(),std::back_inserter(max_vel_));
    m_evaluate_trajectory(result);
    int j = 0;
    int count = 0;
    do
    {
        if(collision_check(result[j+1],result[j]))
        {
            m_binary_search(result,j+1, j);
            m_evaluate_trajectory(result);
        } else
            ++j;
    }while (j<result.size()-1 && ++count<MAX_ITERATIONS);
    if(count==MAX_ITERATIONS)
        std::cerr<<"[TrajPlanner]"<<"No solution found \n";

    return std::make_tuple(max_vel_, result);
}

bool MinimumTime::m_all_collision_check(TRAJECTORIES &trajs) {

    int N = trajs.size();
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if(i != j)
            {
                if (collision_check(trajs[i], trajs[j]))
                    return true;

            }
        }
    }

    return false;
}

std::vector<double> MinimumTime::m_pathLength() {
    std::vector<double> length;
    auto dist = [](wykobi::point2d<double> a,wykobi::point2d<double> b )
    {
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    };
    for (auto & path : uavs_)
    {
        double len = 0;
        int N = path.size();
        for (int i = 0; i < N - 1; ++i) {
            len += dist(path[i], path[i+1]);
        }
        length.push_back(len);
    }
    return length;
}
