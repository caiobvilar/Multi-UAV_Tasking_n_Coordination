//
// Created by robor on 4/22/2020.
//

#include "solver.h"
#include "Algorithm/SeqAreaDecompose.h"
// #include "Algorithm/SweepPath.h"
// #include "Algorithm/MinimumTime.h"
#include <chrono>
#include <Algorithm/HorizontalAreaDecomposition.h>
#include "Algorithm/MinTurnDecomposition.h"
#include "logger.h"
#include "Algorithm/SweepPathCalipers.h"

solver::solver() : solver_thread()
{

    STOP = false;
}

solver::~solver()
{
    STOP = true;
    if (solver_thread.joinable())
        solver_thread.join();
}

void solver::solve()
{
    {
        static std::mutex cout_mutex;
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << "[solver] started" << "\n";
    }
    do
    {
        std::unique_lock<std::mutex> lk(m);
        cv.wait(lk, [=]
                { return ready; });

        ready = false;
        // Manual unlocking is done before notifying, to avoid waking up

        if (!STOP)
        {
            auto t1 = std::chrono::high_resolution_clock::now();
            try
            {
                //          ========================MAIN=====================================

                decompose_areas.clear();
                std::vector<wykobi::polygon<double, 2>> temp_decompose_areas;

                wykobi::polygon<double, 2> roi;
                std::copy(roi_.begin(), roi_.end(), std::back_inserter(roi));

                switch (plannerType)
                {
                case HORIZONTAL:
                {
                    TaskPlanning::HorizontalAreaDecomposition algo(uav_batteries_);
                    temp_decompose_areas = algo.solve(roi);
                    break;
                }
                case TRIANGLE:
                {
                    TaskPlanning::SeqAreaDecompose algo(uav_batteries_);
                    temp_decompose_areas = algo.solve(roi);
                    break;
                }
                case MIN_TURN:
                {
                    TaskPlanning::MinTurnDecomposition algo(uav_batteries_);
                    temp_decompose_areas = algo.solve(roi);
                    break;
                }
                }

                std::copy(temp_decompose_areas.begin(), temp_decompose_areas.end(), std::back_inserter(decompose_areas));
                //                sweep_paths.clear();
                SweepPathCalipers path(sensor_footprints_, roi);
                sweep_paths = path.solve(decompose_areas, plannerType == MIN_TURN);

                //                TrajectoryPlanning::MinimumTime traj(initial_positions_,veolcity_limits_);
                //                sweep_trajs = traj.solve(sweep_paths, safe_d istance_);
                //                veolcity_limits_ = traj.get_velocities();
                //          ================================================================
            }
            catch (std::exception &e)
            {
                std::cerr << e.what() << std::endl;
            }

            auto t2 = std::chrono::high_resolution_clock::now();
            // floating-point duration: no duration_cast needed
            std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
            {
                static std::mutex cout_mutex;
                std::lock_guard<std::mutex> lock(cout_mutex);
                std::cout << "elapsed time: " << fp_ms.count() << "ms\n";
            }

            processed = true;
            logging = true;
        }
        // the waiting thread only to block again (see notify_one for details)
        lk.unlock();
        cv.notify_all();
    } while (!STOP);
    {
        static std::mutex cout_mutex;
        std::lock_guard<std::mutex> lock(cout_mutex);
        std::cout << "[solver] thread terminated" << std::endl;
    }
}

void solver::Start()
{
    solver_thread = std::thread(&solver::solve, this);
}