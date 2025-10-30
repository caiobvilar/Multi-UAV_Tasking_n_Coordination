//
// Created by Redwan Newaz on 5/21/2020.
//

#ifndef AREACOVERAGE_SEQAREADECOMPOSE_H
#define AREACOVERAGE_SEQAREADECOMPOSE_H

#include "pch.h"


namespace TaskPlanning
{
    struct Node{
        double data; // area
        std::vector<Node*>children;
        Node* parent;
        /**
          * @brief a simple data structure for area decomposition
          * @param val : area of a triangle
          * @param num_child : the first children of root node are directly come from polygon triangulation algorithm.
          * We then iteratively subdivide each triangle into two based on the relative capabilities of uavs.
          * @param parent : our goal is to subdivide and merge each triangle from polygon triangulation algorithm
          * in a such way that enables area (task) allocation for uavs, where each area is a convex polygon.
          * Thus, we need to trace parents to perform area allocation efficiently.
          */
        Node(int val, int num_child, Node* parent = nullptr)
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

    class SeqAreaDecompose {
        using TASKS = std::vector<std::vector<LINE>>;
    public:
        /**
         * @brief sequential area decomposition and allocation method
         * @param battery: battery represents flytime for each UAV
         * For the sake of simulation, we use battery to compute relative capabilities of UAVs
         */
        SeqAreaDecompose(const std::vector<double>& battery);

        /**
         * @brief interface for the solver. Ear-Clipping Algorithm is used to decompose a target polygon into a set of
         * triangles and then triangles are subdivided as well as merged into smaller triangles based on the relative capabilities of uavs.
         * @param target polygon
         * @return a set of decomposed polygons
         */
        std::vector<wykobi::polygon<double, 2>> solve(const wykobi::polygon<double,2>& target);

    private:
        std::vector<double> batteries_;
        std::unordered_map<int, std::vector<int>> sub_triangles_;


    protected:
        /**
         * @brief main interface of the algorithm
         * @param target: user defined area which is approximated as a convex polygon
         * @return The area is subdivided into a set of triangles based on the relative capabilities of UAVs,
         * we use tree structure to compactly represent the area decomposition. Initially, we decompose a convex polygon
         * into a set of non-intersecting triangles. Then we further decompose each triangle based on the capabilities of uavs.
         *
         */
        Tree * m_decompose(const wykobi::polygon<double,2>& target, const std::vector<wykobi::triangle<double,2>>& triangle_list);
        /**
         * @brief
         * @param area: total area of target polygon
         * @return relative capabilities of uavs: maximum flyable area based on their batteries
         */
        std::list<int> m_compute_capabilities(double area);

        /**
         * @brief the actual area decomposition happens here
         * @param t: an empty tree which is going to be populated with the area of sub-triangles
         * @param uavs relative capabilities of uavs
         */
        static void m_tree_decomposition(Tree *t, std::list<int>& uavs );


        void m_print_decomposition(const Node *node);

        template <typename T>
        void printParent(const Node *node, T& result)
        {
            if(!node)return;
            result = node->data;
            printParent(node->parent, result);
            std::cout<< node->data<< " : ";
        }
        /**
         * @brief the common point of a set of triangles is computed by compaing first and last
         * triangles in the list. If there is no common point, then default common point is (0,0).
         * @param triangle_list a list of triangles
         * @return common point
         */
        static POINT m_common_point_in_triangles(const std::vector<wykobi::triangle<double,2>>& triangle_list);

        /**
         * @brief given a list of decomposed areas as an unordered map form, here we allocate
         * task for uavs based on their relative capabilities. The key idea is to decompose a triangle into
         * an outerline and a common point. We then focus on outer line decomposition based on the tree structure.
         * At this point, we need to store the line segments for each uav only. We calculate a desire polygon with a
         * set of lines and a common point.
         * @param triangle_list a list of triangles from initial polygon triangulation
         * @param sub_triangles an unordered map which compactly represent area decomposition
         * @param area total size of the target area
         * @return tasks for individual uavs. tasks are represented as 2D vector, where
         * first vector represent the index of uavs and second vector is a set of lines.
         * By adding common point to each set of lines of an uav, we can determine the shape of polygon.
         */
        TASKS m_task_allocation(std::vector<wykobi::triangle<double,2>> triangle_list,
                                std::unordered_map<int, std::vector<int>>& sub_triangles,  double area);

        /**
         * @brief each uav tasks is to cover a allocated polygon. Here we compute polygon from a set of
         * sequential lines and a common point aka shared point.
         * @param tasks 2D vector where each uav is associated with a set of lines
         * @param common_point the shared point among polygons
         * @return a set of polygons for visualization and path planning purposes.
         */
        std::vector<wykobi::polygon<double, 2>> m_tasks_to_polygons(TASKS &tasks, const POINT& common_point);
    };

};

#endif //AREACOVERAGE_SEQAREADECOMPOSE_H
