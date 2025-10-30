//
// Created by robor on 5/21/2020.
//

#include "Algorithm/SeqAreaDecompose.h"

using namespace TaskPlanning;

static inline std::vector<LINE> lines_in_triangle_list(std::vector<wykobi::triangle<double,2>>& triangle_list, POINT common_point)
{
    std::vector<LINE> result;
    for(auto &tri: triangle_list)
    {
        std::vector<POINT> tri_points;
        for (int i = 0; i < 3; ++i) {

            if(distance(tri[i], common_point) == 0) continue;
            tri_points.push_back(tri[i]);
        }
        result.push_back(std::make_pair(tri_points[0], tri_points[1]));
    }
    return result;
}

static std::vector<LINE> decompose_line_by_theta(  const LINE& l, const std::vector<double>& theta)
{
    // formula: y = x2*theta + (1-theta)*x1
    std::vector<LINE> result;
    POINT x = l.first;
    double old_theta = 0.0;
    for(const auto &q: theta)
    {
        // TODO validate this operation
        double t = old_theta + q;
        POINT y = l.second *t + x*(1-t);
        result.push_back(std::make_pair(x, y));
        x = y;
        old_theta = t;
    }
    return result;

}

SeqAreaDecompose::SeqAreaDecompose(const std::vector<double> &battery):batteries_(battery) {

}

Tree *SeqAreaDecompose::m_decompose(const wykobi::polygon<double, 2> &target, const std::vector<wykobi::triangle<double,2>>& triangle_list) {

    // std::cout << "[Capability]: ";
    double area = wykobi::area(target);
    std::list<int> uavs = m_compute_capabilities(area);
    // std::cout << "\n [TriangleArea]: ";
    Tree *t  = new Tree();
    Node *init_node = new Node(area, triangle_list.size());
    int i = 0;
    for(auto &a:triangle_list)
    {
        // std::cout <<-wykobi::area(a) << "\t";
        init_node->children[i++] = new Node(-wykobi::area(a), 2);
    }
    // std::cout << "\n";

    t->root = init_node;
    m_tree_decomposition(t, uavs);

    return t;
}

std::list<int> SeqAreaDecompose::m_compute_capabilities(double area) {

    std::list<int> uavs(batteries_.size());
    std::vector<double>norm_batteries;
    std::copy(batteries_.begin(), batteries_.end(), std::back_inserter(norm_batteries));
    double sum = std::accumulate(norm_batteries.begin(),norm_batteries.end(),0);
    std::transform(norm_batteries.begin(),norm_batteries.end(),uavs.begin(),[&](double val){ return area*val/sum;});
    return uavs;
}

void SeqAreaDecompose::m_print_decomposition(const Node *node) {

    if(!node) return;
    if(!node->children[0])
    {
        auto parent = node->parent;
        int result = 0;
        printParent(parent, result);
        int value = node->data;
        auto old_data = sub_triangles_[result];
        old_data.push_back(value);
        if(result)
            sub_triangles_[result]= old_data;
        else if(value)
            sub_triangles_[value]= old_data;
        // std::cout<< value<<"\n";
    }
    for(auto &child: node->children)
    {
        m_print_decomposition(child);
    }
}

void SeqAreaDecompose::m_tree_decomposition(Tree *t, std::list<int> &uavs) {
    std::list<Node*>q;
    for(auto &child: t->root->children)
        if(child)
            q.push_back(child);
    int d = 0;
    while(!q.empty())
    {
        Node *node = q.front(); //40
        q.pop_front();
        int a = node->data;
        int u = uavs.front() - d;
//        cout<< uavs.front() <<" "<<a << " " << d <<endl;
        uavs.pop_front();
        d = a - u;
        if(d<0)
        {
            uavs.push_front(-d);
            d = 0;
        }
        else
        {
            node->children[0] = new Node(u, 2, node);
            node->children[1] = new Node(d, 2, node);
        }
        if(!uavs.empty() && d>uavs.front())
        {
            d = 0;
            q.push_front(node->children[1]);
        }
    }

}

POINT SeqAreaDecompose::m_common_point_in_triangles(const std::vector<wykobi::triangle<double,2>>& triangle_list)
{
    int N = triangle_list.size();
    auto first_tri = triangle_list[0];
    auto last_tri = triangle_list[N-1];
    for (int i = 0; i < 3; ++i) {
        POINT a = make_point(first_tri[i].x,first_tri[i].y);
        for (int j = 0; j < 3; ++j) {
            POINT b = make_point(last_tri[j].x,last_tri[j].y);
            if(distance(a,b) == 0)
            {
                // std::cout<<"\n[Common Point]: "<< a << std::endl;
                return a;
            }
        }
    }
    std::cerr <<"[Error]: no common point found! \n";
    return make_point<double>(0,0);
}

SeqAreaDecompose::TASKS SeqAreaDecompose::m_task_allocation(std::vector<wykobi::triangle<double,2>> triangle_list,
                                                            std::unordered_map<int, std::vector<int>>& sub_triangles, double area)
{
    // common point among triangles
    auto common_point = m_common_point_in_triangles(triangle_list);
    // separate lines in triangle list
    auto sep_lines = lines_in_triangle_list(triangle_list, common_point);
    std::vector<std::pair<int, LINE>> tri_sep_lines;
    int id = 0;

    for (const auto& line:sep_lines)
    {
        tri_sep_lines.emplace_back(int(abs(wykobi::area(triangle_list[id++]))), line);
        // std::cout<< line << "\n";
    }

    auto area_allocation = [&]( const LINE& l)
    {
        wykobi::polygon<double, 2> poly;
        poly.push_back(l.first);
        poly.push_back(l.second);
        poly.push_back(common_point);
        return abs(wykobi::area(poly));

    };
    std::list<int> tasks = m_compute_capabilities(area);

    TASKS uav_task;
    uav_task.resize(batteries_.size());
    int select = 0;
    //FIXME find necessary conditions to increment select
    for(auto &sep_line: tri_sep_lines)
    {
        int index = sep_line.first;
        LINE ab = sep_line.second;
        // std::cout << index<<" : ";

        // normalizing children values based on their parent value
        std::vector<double>norm_tri;
        std::copy(sub_triangles[index].begin(), sub_triangles[index].end(), std::back_inserter(norm_tri));
        double sum = std::accumulate(norm_tri.begin(),norm_tri.end(),0); // parent value
        std::transform(norm_tri.begin(),norm_tri.end(),norm_tri.begin(),[&](double val){ return val/sum;}); // normalization
//        std::copy(norm_tri.begin(),norm_tri.end(),std::ostream_iterator<double>(// std::cout, " ")); // print out
        // std::cout<<std::endl;
        // m_decompose line ab and allocate to an uav
        auto sub_lines = decompose_line_by_theta(ab, norm_tri);
        for(auto &l:sub_lines)
        {
            // calculate area allocated for selected uav
            double covered = 0.0;
            for(auto it = uav_task[select].begin(); it != uav_task[select].end();++it)
                covered += area_allocation(*it);
            // when allocated area is less than the uav's capability, add the line
            if(covered< tasks.front())
            {
                uav_task[select].push_back(l);
            }
            else if(!tasks.empty() && select < batteries_.size())
            {
                // move to next uav and append this line for the next uav
                tasks.pop_front();
                ++select;
                uav_task[select].clear();
                uav_task[select].push_back(l);
            }
        }
    }
    return uav_task;
}
std::vector<wykobi::polygon<double, 2>> SeqAreaDecompose::solve(const wykobi::polygon<double, 2> &target) {
    // obtain tree from
    // simple polygon triangulation
    std::vector<wykobi::triangle<double,2>> triangle_list;
    wykobi::algorithm::polygon_triangulate<wykobi::point2d<double >>
            (target, std::back_inserter(triangle_list));

    Tree *t = m_decompose(target, triangle_list);
    // std::cout <<"\n===========================================================================\n";
    sub_triangles_.clear(); //populating subtriangles
    m_print_decomposition(t->root);
    // std::cout <<"\n===========================================================================\n";
    // task allocation
    auto tasks = m_task_allocation(triangle_list,sub_triangles_, abs(wykobi::area(target)));

    // TODO: create polygones for uav_task[id][lines]
    auto common_point = m_common_point_in_triangles(triangle_list);
    return m_tasks_to_polygons(tasks, common_point);
}

std::vector<wykobi::polygon<double, 2>> SeqAreaDecompose::m_tasks_to_polygons(SeqAreaDecompose::TASKS &tasks, const POINT& common_point) {
    std::vector<wykobi::polygon<double, 2>> result;
    auto convex_hull = [](const POLY &polygon)
    {
        std::vector<wykobi::point2d<double>> hull;
        wykobi::algorithm::convex_hull_melkman<wykobi::point2d<double>>
                (
                        polygon.begin(),
                        polygon.end(),
                        std::back_inserter(hull)
                );
        return wykobi::make_polygon<double>(hull);
    };
    for(auto &t:tasks)
    {
        POLY poly;
        for(const auto & line_: t)
        {
            poly.push_back(line_.first);
            poly.push_back(line_.second);
        }
        poly.push_back(common_point);
        result.emplace_back(convex_hull(poly));
    }
    // std::cout<<"[result]: "<<result.size() <<"\n";

    return result;
}

