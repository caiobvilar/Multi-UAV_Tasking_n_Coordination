//
// Created by robor on 5/21/2020.
//

#include "Algorithm/SeqAreaDecompose.h"

using namespace TaskPlanning;

static inline std::vector<LINE> lines_in_triangle_list(const std::vector<wykobi::triangle<double, 2>> &triangle_list, POINT common_point)
{
    std::vector<LINE> result;
    for (auto &tri : triangle_list)
    {
        std::vector<POINT> tri_points;
        for (int i = 0; i < 3; ++i)
        {

            if (distance(tri[i], common_point) == 0)
                continue;
            tri_points.push_back(tri[i]);
        }
        result.push_back(std::make_pair(tri_points[0], tri_points[1]));
    }
    return result;
}

static std::vector<LINE> decompose_line_by_theta(const LINE &l, const std::vector<double> &theta)
{
    // formula: y = x2*theta + (1-theta)*x1
    std::vector<LINE> result;
    POINT x = l.first;
    double old_theta = 0.0;
    for (const auto &q : theta)
    {
        // TODO validate this operation
        double t = old_theta + q;
        POINT y = l.second * t + x * (1 - t);
        result.push_back(std::make_pair(x, y));
        x = y;
        old_theta = t;
    }
    return result;
}

SeqAreaDecompose::SeqAreaDecompose(const std::vector<double> &battery) : batteries_(battery)
{
}

void SeqAreaDecompose::m_decompose(const wykobi::polygon<double, 2> &target, const std::vector<wykobi::triangle<double, 2>> &triangle_list)
{

    // std::cout << "[Capability]: ";
    double area = wykobi::area(target);
    auto uavs = m_compute_capabilities(area);
    // std::cout << "\n [TriangleArea]: ";
    std::vector<double> triangles(triangle_list.size());
    std::transform(triangle_list.begin(), triangle_list.end(),
                   triangles.begin(), [&](const wykobi::triangle<double, 2> &a)
                   { return abs(wykobi::area(a)); });
    TriangleDecomposition<double> myTi(triangles);
    myTi.solve(uavs);

    // common point among triangles
    auto common_point = m_common_point_in_triangles(triangle_list);
    // separate lines in triangle list
    auto sep_lines = lines_in_triangle_list(triangle_list, common_point);
    assert(sep_lines.size() == triangle_list.size());
    auto decomposition = myTi.get_area_decomposition();
    auto uav_tasks = myTi.get_uav_tasks();
    list<pair<double, LINE>> allocation_lines;
    printf("line segmenting \n");
    for (std::size_t j = 0; j < triangle_list.size(); ++j)
    {
        auto key = int(abs(wykobi::area(triangle_list[j])));
        auto line = sep_lines[j];
        deque<double> allocated_area;
        if (decomposition.find(j) != decomposition.end())
            allocated_area = decomposition[j];
        else
            cerr << "key is not found! " << j << endl;
        // allocate lines to uav
        //        copy(sep_lines.begin(), sep_lines.end(), back_inserter(allocation_lines));
        // convert allocated area to vector
        if (allocated_area.size() == 1)
        {
            allocation_lines.push_back(make_pair(key, line));
            continue;
        }
        vector<double> theta;
        std::copy(allocated_area.begin(), allocated_area.end(), std::back_inserter(theta));
        normalize(theta);
        // auto decomposing sep_line
        auto decomposed_lines = decompose_line_by_theta(line, theta);
        assert(decomposed_lines.size() == allocated_area.size());
        // put all the lines in allocation
        int count = 0;
        for (auto &l : decomposed_lines)
            allocation_lines.push_back(make_pair(allocated_area[count++], l));
    }
    auto get_sum = [](const deque<double> &t)
    { return std::accumulate(t.begin(), t.end(), 0.0); };

    int num_uavs = batteries_.size();
    unordered_map<int, double> task_capacity;
    for (int j = 0; j < num_uavs; ++j)
    {
        task_capacity[j] = get_sum(uav_tasks[j]);
    }
    int uav_id = 0;
    double allocate = 0;
    unordered_map<int, vector<LINE>> uav_allocation;
    printf("allocating %ld lines to uavs \n", allocation_lines.size());
    do
    {
        auto candidate = allocation_lines.front();
        allocate += candidate.first;
        printf("> [uav %d] capacity % lf , allocate = %lf, candidate = %lf \n", uav_id, task_capacity[uav_id], allocate, candidate.first);
        if (task_capacity[uav_id] > allocate && uav_id < num_uavs)
        {
            uav_allocation[uav_id].push_back(candidate.second);
        }
        else if (uav_id < num_uavs)
        {
            // need to repeat the last point
            uav_id += 1;
            //            allocate = 0;
            allocate = -candidate.first;
            allocation_lines.push_front(candidate);
        }
        else
            break;
        allocation_lines.pop_front();
    } while (!allocation_lines.empty() && uav_id < num_uavs);
    printf("allocating polygons \n");
    result_.clear();
    auto convex_hull = [](const POLY &polygon)
    {
        std::vector<wykobi::point2d<double>> hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>(
            polygon.begin(),
            polygon.end(),
            std::back_inserter(hull));
        return wykobi::make_polygon<double>(hull);
    };
    for (int k = 0; k < num_uavs; ++k)
    {
        POLY poly;
        for (auto it = uav_allocation[k].begin(); it != uav_allocation[k].end(); ++it)
        {
            poly.push_back(it->first);
            poly.push_back(it->second);
        }
        poly.push_back(common_point);

        result_.push_back(convex_hull(poly));
    }
    printf("allocating polygons finished \n");
}

std::deque<double> SeqAreaDecompose::m_compute_capabilities(double area)
{

    std::deque<double> uavs(batteries_.size());
    std::vector<double> norm_batteries;
    std::copy(batteries_.begin(), batteries_.end(), std::back_inserter(norm_batteries));
    normalize(norm_batteries);
    std::transform(norm_batteries.begin(), norm_batteries.end(), uavs.begin(), [&](double val)
                   { return area * val; });
    return uavs;
}

POINT SeqAreaDecompose::m_common_point_in_triangles(const std::vector<wykobi::triangle<double, 2>> &triangle_list)
{
    int N = triangle_list.size();
    auto first_tri = triangle_list[0];
    auto last_tri = triangle_list[N - 1];
    for (int i = 0; i < 3; ++i)
    {
        POINT a = make_point(first_tri[i].x, first_tri[i].y);
        for (int j = 0; j < 3; ++j)
        {
            POINT b = make_point(last_tri[j].x, last_tri[j].y);
            if (distance(a, b) == 0)
            {
                // std::cout<<"\n[Common Point]: "<< a << std::endl;
                return a;
            }
        }
    }
    std::cerr << "[Error]: no common point found! \n";
    return make_point<double>(0, 0);
}

std::vector<wykobi::polygon<double, 2>> SeqAreaDecompose::solve(const polygon<double, 2> &target)
{
    // simple polygon triangulation
    std::vector<wykobi::triangle<double, 2>> triangle_list;
    wykobi::algorithm::polygon_triangulate<wykobi::point2d<double>>(target, std::back_inserter(triangle_list));
    m_decompose(target, triangle_list);

    return result_;
}
