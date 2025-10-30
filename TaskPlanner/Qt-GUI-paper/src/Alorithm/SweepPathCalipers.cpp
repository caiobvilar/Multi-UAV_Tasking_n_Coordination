//
// Created by robor on 7/19/2020.
//

#include "Algorithm/SweepPathCalipers.h"
#include "Algorithm/MinTurnDecomposition.h"
#include <cassert>


using namespace TaskPlanning;
SweepPathCalipers::SweepPathCalipers(const std::vector<double> &footprints, POLY poly) :
footprints_(footprints), combinedPoly(poly) {

}

std::vector<wykobi::polygon<double, 2>>
SweepPathCalipers::solve(const std::vector<wykobi::polygon<double, 2>> &decomoseArea, bool oneshot) {

    assert(decomoseArea.size()>0);
//    std::cout<<"[oneshot]: "<< oneshot << "\n";
    std::vector<wykobi::polygon<double, 2>>result;
    const double delta = 0.17;
    arma::mat22 AD, BC, EF;
    for(auto& target: decomoseArea)
    {
        std::tie(AD, BC) = MinTurnDecomposition::MinRectMat(target);
        int count = 0;
        wykobi::polygon<double, 2> path;
        for (double k = 0; k < 1; k+=delta, ++count) {
            EF = (1-k)*AD + k*BC;
            auto E = wykobi::make_point<double>(EF(0,1), EF(0,0));
            auto F = wykobi::make_point<double>(EF(1,1), EF(1,0));
            if(count%2==0)
            {
                path.push_back(E);
                path.push_back(F);
            } else{
                path.push_back(F);
                path.push_back(E);
            }
        }
        if(!oneshot)
        {
            wykobi::polygon<double,2> clipped_polygon;
            wykobi::algorithm::sutherland_hodgman_polygon_clipper<wykobi::point2d<double>>
                    (target, path, clipped_polygon);
            result.push_back(clipped_polygon);
        }
        else
            result.push_back(path);

    }
    if(oneshot)
        m_refine_polygons(result);
    return result;

}


void SweepPathCalipers::m_refine_polygons(vector<wykobi::polygon<double, 2>> &polyons) {

    std::vector<wykobi::polygon<double, 2>>final_result;
    std::copy(polyons.begin(), polyons.end(), std::back_inserter(final_result));
    polyons.clear();
    for(auto &path : final_result)
    {
        wykobi::polygon<double,2> clipped_polygon;
        wykobi::algorithm::sutherland_hodgman_polygon_clipper<wykobi::point2d<double>>
                (combinedPoly, path, clipped_polygon);
        polyons.emplace_back(clipped_polygon);
    }


    //    auto getIndex = [](const wykobi::point2d<double>& p){
//        return p.x + 100*p.y;
//    };
//    if(polyons.size()<2)return;
//    for(auto& poly:polyons)
//    {
//      int N = poly.size() - 1;
//      if(N>0)
//      if(getIndex(poly[0]) == getIndex(poly[N]))
//      {
//          poly.erase(N);
//      }
//    }

}
