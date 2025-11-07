//
// Created by Redwan Newaz on 5/21/2020.
//

#ifndef AREACOVERAGE_SEQAREADECOMPOSE_H
#define AREACOVERAGE_SEQAREADECOMPOSE_H

#include "pch.h"
#include "Algorithm/TriangleDecomposition.h"

namespace TaskPlanning
{
    struct Node
    {
        double data; // area
        std::vector<Node *> children;
        Node *parent;
        /**
         * @brief a simple data structure for area decomposition
         * @param val : area of a triangle
         * @param num_child : the first children of root node are directly come from polygon triangulation algorithm.
         * We then iteratively subdivide each triangle into two based on the relative capabilities of uavs.
         * @param parent : our goal is to subdivide and merge each triangle from polygon triangulation algorithm
         * in a such way that enables area (task) allocation for uavs, where each area is a convex polygon.
         * Thus, we need to trace parents to perform area allocation efficiently.
         */
        Node(int val, int num_child, Node *parent = nullptr)
        {
            data = val;
            children.resize(num_child);
            this->parent = parent;
        }
    };

    struct Tree
    {
        Node *root;
    };

    class SeqAreaDecompose
    {
        using TASKS = std::vector<std::vector<LINE>>;

    public:
        /**
         * @brief sequential area decomposition and allocation method
         * @param battery: battery represents flytime for each UAV
         * For the sake of simulation, we use battery to compute relative capabilities of UAVs
         */
        SeqAreaDecompose(const std::vector<double> &battery);

        /**
         * @brief interface for the solver. Ear-Clipping Algorithm is used to decompose_ a target polygon into a set of
         * triangles and then triangles are subdivided as well as merged into smaller triangles based on the relative capabilities of uavs.
         * @param target polygon
         * @return a set of decomposed polygons
         */
        std::vector<wykobi::polygon<double, 2>> solve(const wykobi::polygon<double, 2> &target);

    private:
        std::vector<double> batteries_;
        std::unordered_map<int, std::vector<int>> sub_triangles_;
        POLYGONS result_;

    protected:
        /**
         * @brief main interface of the algorithm
         * @param target: user defined area which is approximated as a convex polygon
         * @return The area is subdivided into a set of triangles based on the relative capabilities of UAVs,
         * we use tree structure to compactly represent the area decomposition. Initially, we decompose_ a convex polygon
         * into a set of non-intersecting triangles. Then we further decompose_ each triangle based on the capabilities of uavs.
         *
         */
        void m_decompose(const wykobi::polygon<double, 2> &target, const std::vector<wykobi::triangle<double, 2>> &triangle_list);
        /**
         * @brief
         * @param area: total area of target polygon
         * @return relative capabilities of uavs: maximum flyable area based on their batteries
         */
        std::deque<double> m_compute_capabilities(double area);

        /**
         * @brief the common point of a set of triangles is computed by compaing first and last
         * triangles in the list. If there is no common point, then default common point is (0,0).
         * @param triangle_list a list of triangles
         * @return common point
         */
        static POINT m_common_point_in_triangles(const std::vector<wykobi::triangle<double, 2>> &triangle_list);
    };

}
#endif // AREACOVERAGE_SEQAREADECOMPOSE_H
