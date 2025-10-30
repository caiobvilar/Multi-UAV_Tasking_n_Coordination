//
// Created by robor on 5/24/2020.
//

#ifndef AREACOVERAGE_SWEEPPATH_H
#define AREACOVERAGE_SWEEPPATH_H


#include <iostream>
#include <vector>
#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>
#include <armadillo>
#include <armadillo>
#include <numeric>


namespace PathPlanning
{

    class SweepPath {
    public:
        /**
         * @brief Calculate the intermediate distance between sweeping lines using sensor footprint.
         * @param footprints sensor footprints of uavs
         */
        SweepPath(const std::vector<double>&footprints);
        /**
         * @brief The key idea of this class is to generate sweeping path for each polygon from task planner.
         * First we generate sweep lines for a large rectangle area based on the sensor foorprint.
         * Second we rotate the rectangle to comply with a line in polygon.
         * Finally, we use polygon clipping algorithm to clip path from the rectangle for a given polygon
         * @param decomoseArea a set of polygons from task planner
         * @return a set of sweeping path based on the polygons and sensor foot prints of uavs
         */
        std::vector<wykobi::polygon<double, 2>> solve(const std::vector<wykobi::polygon<double, 2>>& decomoseArea);

    private:
        std::vector<double>&footprints_;


    protected:
        /**
         * @brief find angle of each poly line in the polygon
         * @param decomoseArea a set of polygons
         * @return
         */
        std::vector<double> m_get_yaw_angles(const wykobi::polygon<double, 2> &decomosePolygon ) const;
        /**
         * @brief calculate the cost between number of turns and the path length \par
         * our goal is to minimize the number of turns as well as the path length
         * @param decomosePolygon
         * @return length*num_turns
         */
        double m_path_stat(const wykobi::polygon<double, 2> &pathPolygon) const;

        wykobi::polygon<double, 2> m_optimize(const wykobi::polygon<double, 2> &decomosePolygon);
        /**
         * @brief rotate rectangle with respect to a reference line \par
         * First we compute the angle of the line m = atan2((y2-y1)/(x2-x1)) \par
         * Then transform the rectangle as follows \par
         * \remarks
         *      X_hat =  np.matmul(X,R) + x0 \par
         *      center = (X_hat[0] + X_hat[2])/2.0 \par
         *      X_hat = X_hat - center + x0 \par
         *
         * @param bb: bounding box (rectangle)
         * @param ref: reference line (line)
         */
//        void m_rotate_rectangle(geom::rectangle & bb, geom::line &ref);
    };
};



#endif //AREACOVERAGE_SWEEPPATH_H
