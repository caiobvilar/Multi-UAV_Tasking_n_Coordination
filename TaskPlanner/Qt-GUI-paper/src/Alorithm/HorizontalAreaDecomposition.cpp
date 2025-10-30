//
// Created by robor on 6/20/2020.
//

#include "Algorithm/HorizontalAreaDecomposition.h"
using namespace TaskPlanning;

HorizontalAreaDecomposition::HorizontalAreaDecomposition(const vector<double> &batteries) : MinTurnDecomposition(
        batteries) {}

vector<wykobi::polygon<double, 2>> HorizontalAreaDecomposition::solve(const wykobi::polygon<double, 2> &target) {
    // matrix representation of a rectangle
    /*
     *      D------C
     *      |      |
     *      A------B
     */
    arma::mat22 AD, BC, EF, REF;
    AD(0,0) = 0;    // Ay
    AD(1,0) = 100;    // Dy
    AD(1,1) = 0;    // Dx
    AD(0,1) = 0;    // Ax

    BC(0,0) = 0;    // By
    BC(1,0) = 100;    // Cy
    BC(1,1) = 100;    // Cx
    BC(0,1) = 100;    // Bx

    std::vector<wykobi::polygon<double, 2>>result;
    capabilities_ = m_get_capabilities(abs(wykobi::area(target)));
    m_decompose_area(target, AD, BC, EF, REF, result);
    return result;

}
