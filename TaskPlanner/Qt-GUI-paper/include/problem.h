//
// Created by robor on 4/22/2020.
//

#ifndef AREACOVERAGE_PROBLEM_H
#define AREACOVERAGE_PROBLEM_H

#include "pch.h"

enum PLANNER_TYPE
{
    MIN_TURN = 0,
    TRIANGLE,
    HORIZONTAL
};

class problem;
class problem : public std::enable_shared_from_this<problem>
{
    friend class MainWindow;
    friend class logger;

public:
    problem()
    {
        ready = false;
    }
    std::shared_ptr<problem> getPtr()
    {
        return shared_from_this();
    }
    void reset()
    {
        input_.clear();
        roi_.clear();
    }

    void formulate(int num_uavs, int safety)
    {
        safe_distance_ = safety;
        // TODO compute roi from input
        compute_convex_hull(input_);
        if (num_uavs > 0)
            random_init(num_uavs);
        num_uavs_ = num_uavs;
    }

    void add_vertex(double x, double y)
    {
        input_.push_back(make_point(x, y));
    }

private:
    void random_init(int count)
    {
        auto isValid = [&]()
        {
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < initial_positions_.size(); ++i)
            {
                for (size_t j = i + 1; j < initial_positions_.size(); ++j)
                {
                    double d = distance(initial_positions_[i], initial_positions_[j]);
                    min_dist = std::min(min_dist, d);
                }
            }
            std::cout << "[problem ] min dist " << min_dist << "\n";
            return (min_dist > safe_distance_) ? true : false;
        };
        do
        {
            initial_positions_.clear();
            recursive_random_init(count);

        } while (!isValid());
    }
    void recursive_random_init(int count)
    {
        /*
         * randomly place a rectangle within the bounded box
         * and then sample a point from it.
         * Recurse the whole process until finding the desired number of samples
         */
        const std::size_t max_points = 1;
        wykobi::rectangle<double> rectangle;
        wykobi::generate_random_object<double>(0, 0, 100, 100, rectangle);
        std::vector<wykobi::point2d<double>> point_list;
        point_list.reserve(max_points);
        wykobi::generate_random_points(rectangle, max_points, std::back_inserter(point_list));

        if (count > 0)
        {
            // don't sample from the target area
            for (auto &checkPoint : point_list)
            {
                if (!point_in_convex_polygon(checkPoint))
                {
                    initial_positions_.push_back(checkPoint);
                    count--;
                    break;
                }
            }
            recursive_random_init(count);
        }
    }

public:
    friend std::ostream &operator<<(std::ostream &os, const problem & /*problem*/)
    {

        return os;
    }

private:
    void add_batteries(const std::vector<double> &battery)
    {
        uav_batteries_.clear();
        std::copy(battery.begin(), battery.end(), std::back_inserter(uav_batteries_));
    }

    void add_footprints(const std::vector<double> &footprints)
    {
        sensor_footprints_.clear();
        std::copy(footprints.begin(), footprints.end(), std::back_inserter(sensor_footprints_));
    }

    void add_velocities(const std::vector<double> &velocities)
    {
        veolcity_limits_.clear();
        std::copy(velocities.begin(), velocities.end(), std::back_inserter(veolcity_limits_));
    }

    void compute_convex_hull(const POLY &polygon)
    {
        std::vector<wykobi::point2d<double>> hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>(
            polygon.begin(),
            polygon.end(),
            std::back_inserter(hull));
        roi_ = wykobi::make_polygon<double>(hull);
    }

    bool point_in_convex_polygon(const POINT &p)
    {

        // For a point to be inside a convex polygon, it must be on the same side
        // of all edges. For a CCW ordered polygon, the point should be on the left side.

        if (roi_.size() < 3)
        {
            return false; // Not a valid polygon
        }

        bool allSameSide = true;
        bool isNegative = false; // Initialize with a default value
        double threshold = -safe_distance_;

        for (size_t i = 0; i < roi_.size(); ++i)
        {
            auto A = roi_[i];
            auto B = roi_[(i + 1) % roi_.size()];

            // Line equation coefficients: Ax + By + C = 0
            double lineA = B.y - A.y;
            double lineB = A.x - B.x;
            double lineC = B.x * A.y - A.x * B.y;

            // Calculate signed distance
            // Note: We're not dividing by the denominator since we only care about the sign
            double signedDistance = lineA * p.x + lineB * p.y + lineC;
            signedDistance /= std::sqrt(lineA * lineA + lineB * lineB);

            // If this is the first edge, establish the reference side
            if (i == 0)
            {
                // Store if first distance is negative (we'll check if all others match)
                isNegative = (signedDistance < threshold);
                allSameSide = true;

                // For subsequent edges, check if the sign is consistent
            }
            else if ((signedDistance < threshold) != isNegative)
            {
                allSameSide = false;
                break;
            }
        }
        // return wykobi::point_in_convex_polygon(p, roi_);
        return allSameSide;
    }

protected:
    POLY roi_, input_;
    int num_uavs_, safe_distance_;
    std::vector<double> uav_batteries_, sensor_footprints_;

public:
    std::mutex m;
    std::condition_variable cv;
    bool ready, processed;
    std::vector<wykobi::polygon<double, 2>> decompose_areas, sweep_paths, sweep_trajs;
    std::vector<POINT> initial_positions_;
    std::vector<double> veolcity_limits_;
    PLANNER_TYPE plannerType;
};

#endif // AREACOVERAGE_PROBLEM_H
