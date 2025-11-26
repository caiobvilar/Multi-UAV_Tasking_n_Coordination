//
// Created by redwan on 3/22/21.
//

#include "algo/TriangularDecomposition.h"
using namespace TaskPlanning;

TriangularDecomposition::TriangularDecomposition(const vector<double> &batteries) : batteries_(batteries)
{
}

vector<wykobi::polygon<double, 2>> TriangularDecomposition::solve(const wykobi::polygon<double, 2> &target)
{
    auto search_poly = m_get_search_space(target);
    auto total_area = wykobi::area(search_poly);
    auto capabilities = m_get_capabilities(total_area);
    vector<wykobi::polygon<double, 2>> result;
    double cum_cap = 0;
    int count = capabilities.size();
    for (auto &cap : capabilities)
    {
        cum_cap += cap;
        wykobi::polygon<double, 2> ref_poly;
        //        cout<< "capabilities "<< cum_cap << endl;
        int divide_index = binary_search(search_poly, cum_cap);
        std::copy(search_poly.begin(), search_poly.begin() + divide_index, back_inserter(ref_poly));
        if (--count > 0)
            result.push_back(ref_poly);
    }
    result.push_back(search_poly);
    // refine decomposed polygon to smaller polygon
    int N = (int)result.size() - 1;
    do
    {
        int last_point_it = (int)result[N - 1].size() - 1;
        wykobi::polygon<double, 2> clipped_polygon;
        std::copy(result[N].begin() + last_point_it, result[N].end(), std::back_inserter(clipped_polygon));
        clipped_polygon.push_back(result[N][0]);
        result[N] = clipped_polygon;
    } while (--N > 0);

    return result;
}

int TriangularDecomposition::binary_search(const wykobi::polygon<double, 2> &search_poly, double x)
{
    int l = 0;
    int r = (int)search_poly.size() - 1;

    while (l <= r)
    {
        int m = l + (r - l) / 2;
        // Check if x is present at mid
        wykobi::polygon<double, 2> proposal;
        std::copy(search_poly.begin(), search_poly.begin() + m, back_inserter(proposal));
        auto area = wykobi::area(proposal);
        //        cout<<area << " | " << x <<endl;
        if (abs(area - x) < 50)
            return m;
        // If x greater, ignore left half
        if (area < x)
            l = m + 1;
        // If x is smaller, ignore right half
        else
            r = m - 1;
    }
    // if we reach here, then element was
    // not present
    return r + 1;
}

std::vector<double> TriangularDecomposition::m_get_capabilities(double area)
{
    std::vector<double> norm_batteries;
    std::copy(batteries_.begin(), batteries_.end(), std::back_inserter(norm_batteries));
    double sum = std::accumulate(norm_batteries.begin(), norm_batteries.end(), 0);
    std::transform(norm_batteries.begin(), norm_batteries.end(), norm_batteries.begin(), [&](double val)
                   { return area * val / sum; });
    return norm_batteries;
}

wykobi::polygon<double, 2> TriangularDecomposition::m_get_search_space(const wykobi::polygon<double, 2> &roi)
{

    // convert polygon to arma vector for interpolation
    arma::vec r(roi.size() - 1), xx(roi.size()), yy(roi.size());
    for (std::size_t i = 0; i < roi.size() - 1; ++i)
    {
        arma::vec2 x{roi[i].x, roi[i].y};
        arma::vec2 y{roi[i + 1].x, roi[i + 1].y};
        auto d = arma::norm(x - y);
        r(i) = d;
        // populate arma vectors
        xx(i) = x(0);
        yy(i) = x(1);
        if (i == (roi.size() - 2))
        {
            xx(i + 1) = y(0);
            yy(i + 1) = y(1);
        }
    }
    // interpolate polygon sides
    int NUM_INT_POINT = 10;
    wykobi::polygon<double, 2> search_poly;
    for (std::size_t i = 0; i < roi.size() - 1; ++i)
    {
        auto d = r(i);
        auto denom = r.min() / d;
        const int num = NUM_INT_POINT / denom;
        arma::vec pxx = arma::linspace<arma::vec>(xx(i), xx(i + 1), num);
        arma::vec pyy = arma::linspace<arma::vec>(yy(i), yy(i + 1), num);
        for (arma::uword j = 0; j < pxx.size(); ++j)
        {
            search_poly.push_back(wykobi::make_point(pxx(j), pyy(j)));
        }
    }
    return search_poly;
}
