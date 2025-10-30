//
// Created by robor on 7/19/2020.
//

#ifndef AREACOVERAGE_SWEEPPATHCALIPERS_H
#define AREACOVERAGE_SWEEPPATHCALIPERS_H
#include <iostream>
#include <vector>
#include <wykobi.hpp>
#include <wykobi_algorithm.hpp>
#include <armadillo>
#include "Algorithm/RotatingCalipers.h"

class SweepPathCalipers {
    using POLY = wykobi::polygon<double,2>;
public:
    /**
     * @brief Calculate the intermediate distance between sweeping lines using sensor footprint.
     * @param footprints sensor footprints of uavs
     */
    SweepPathCalipers(const std::vector<double>&footprints, POLY poly);



    /**
     * @brief The key idea of this class is to generate sweeping path for each polygon from task planner.
     * First we generate sweep lines for a large rectangle area based on the sensor foorprint.
     * Second we rotate the rectangle to comply with a line in polygon.
     * Finally, we use polygon clipping algorithm to clip path from the rectangle for a given polygon
     * @param decomoseArea a set of polygons from task planner
     * @return a set of sweeping path based on the polygons and sensor foot prints of uavs
     */
    std::vector<wykobi::polygon<double, 2>> solve(const std::vector<wykobi::polygon<double, 2>>& decomoseArea, bool oneshot);


protected:
    void m_refine_polygons(std::vector<wykobi::polygon<double, 2>>& polyons);
private:
    const std::vector<double>&footprints_;
    POLY combinedPoly;
};


#endif //AREACOVERAGE_SWEEPPATHCALIPERS_H
