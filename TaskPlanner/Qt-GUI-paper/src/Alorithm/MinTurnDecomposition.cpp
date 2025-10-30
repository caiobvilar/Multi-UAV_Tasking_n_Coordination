//
// Created by robor on 7/14/2020.
//

#include "Algorithm/MinTurnDecomposition.h"


using namespace TaskPlanning;
MinTurnDecomposition::MinTurnDecomposition(const vector<double> &batteries) : batteries_(batteries) {

}

std::vector<wykobi::polygon<double, 2>> MinTurnDecomposition::solve(const wykobi::polygon<double, 2> &target) {
    capabilities_ = m_get_capabilities(abs(wykobi::area(target)));

    std::vector<wykobi::polygon<double, 2>>result;
    arma::mat22 AD, BC, EF, REF;
    std::tie(AD, BC) = MinRectMat(target);
    m_decompose_area(target, AD, BC, EF, REF, result);

    return result;
}




void MinTurnDecomposition::m_decompose_area(const wykobi::polygon<double, 2> &target, const arma::mat22 &AD,
                                            const arma::mat22 &BC, arma::mat22 &EF, arma::mat22 &REF,
                                            vector<wykobi::polygon<double, 2>> &result) {
    double total_area = 0;
    REF = AD;
    function<double(double)> f = [&](double theta)
    {
        EF = AD*(1-theta) + theta*BC;
        auto divider = m_get_boundinbox(AD, EF);
        return abs(wykobi::area(m_cut_polygon(divider, target)));
    };
    for (auto& cap : capabilities_)
    {
        total_area += cap;
        double theta = binary_search(total_area, f);
        EF = AD*(1-theta) + theta*BC;
        auto divider = m_get_boundinbox(REF, EF);
        auto polygon = m_cut_polygon(divider, target);
        result.push_back(polygon);
        REF = EF;
        // std::cout<< "[MinTurnDecomposition]  " <<wykobi::area(polygon)<<"\t|cap:= "<<cap <<"\n";
    }
}

wykobi::polygon<double, 2> MinTurnDecomposition::m_get_boundinbox(const arma::mat22 &AD, const arma::mat22& EF) {
    wykobi::polygon<double, 2> divider;

    divider.push_back(wykobi::make_point<double>(EF(0,1),EF(0,0)));
    divider.push_back(wykobi::make_point<double>(EF(1,1),EF(1,0)));

    divider.push_back(wykobi::make_point<double>(AD(1,1),AD(1,0)));
    divider.push_back(wykobi::make_point<double>(AD(0,1),AD(0,0)));
    return divider;
}

wykobi::polygon<double, 2> MinTurnDecomposition::m_cut_polygon(const wykobi::polygon<double, 2> &divider,
                                                               const wykobi::polygon<double, 2> &target) {
    wykobi::polygon<double,2> clipped_polygon;
    wykobi::algorithm::sutherland_hodgman_polygon_clipper<wykobi::point2d<double>>
            (divider, target, clipped_polygon);
//    // std::cout<<"area := "<<wykobi::area(clipped_polygon) <<"\n";
    return clipped_polygon;
}

double MinTurnDecomposition::binary_search(double cap, function<double(double)> &f) {
    double start = 0;
    double RES = 0.00001;
    const int N = 1.0/RES;
    std::vector<double>ivec(N);
    std::generate(ivec.begin(), ivec.end(), [=]()mutable{start+=RES; return start;});
    int l = 0;
    int r = ivec.size()-1;
    while (l <= r) {
        int m = l + (r - l) / 2;
        double pivot = ivec[m];
        double area = f(pivot);
        double diff =  cap - area;
        if(diff ==0)
        {
            return pivot;
        }
        else if (diff > 0)
        {
            l = m + 1;
        }
        else
        {
            r = m - 1;
        }
    }
    return ivec[r];
}

std::vector<double> MinTurnDecomposition::m_get_capabilities(double area) {
    std::vector<double>norm_batteries;
    std::copy(batteries_.begin(), batteries_.end(), std::back_inserter(norm_batteries));
    double sum = std::accumulate(norm_batteries.begin(),norm_batteries.end(),0);
    std::transform(norm_batteries.begin(),norm_batteries.end(),norm_batteries.begin(),[&](double val){ return area*val/sum;});
    return norm_batteries;
}

MinTurnDecomposition::~MinTurnDecomposition() {

}

std::tuple<arma::mat22, arma::mat22> MinTurnDecomposition::MinRectMat(const wykobi::polygon<double, 2> &target)
{

    // finding minimum bounding rectangle
    std::vector<Point> target_;
    for (int i = 0; i < target.size(); ++i) {
        auto point = target[i];
        TaskPlanning::Point p(point.x, point.y);
        target_.emplace_back(p);
    }
    auto minRect = RotatingCalipers::minAreaRect(target_);

    auto dist1 = RotatingCalipers::dist(minRect.rect[0], minRect.rect[1]);
    auto dist2 = RotatingCalipers::dist(minRect.rect[2], minRect.rect[1]);
    // std::cout<<"[MinTurnDecomposition] H x W " <<dist1 <<" x " << dist2 <<"\n";
    int init = (dist1<dist2)?0:1;
    wykobi::polygon<double, 2> min_rect_poly;
    int N = minRect.rect.size();
    for (int j = init; j < N + init; ++j) {
        min_rect_poly.push_back(wykobi::make_point<double>(minRect.rect[j%N].x,minRect.rect[j%N].y));
    }
    // matrix representation of a rectangle
    /*
     *      D------C
     *      |      |
     *      A------B
     */
    arma::mat22 AD, BC;
    AD(0,0) = min_rect_poly[0].y;    // Ay
    AD(1,0) = min_rect_poly[3].y;    // Dy
    AD(1,1) = min_rect_poly[3].x;    // Dx
    AD(0,1) = min_rect_poly[0].x;    // Ax

    BC(0,0) = min_rect_poly[1].y;    // By
    BC(1,0) = min_rect_poly[2].y;    // Cy
    BC(1,1) = min_rect_poly[2].x;    // Cx
    BC(0,1) = min_rect_poly[1].x;    // Bx
    return std::make_tuple(AD, BC);
}