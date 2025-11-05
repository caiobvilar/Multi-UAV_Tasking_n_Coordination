//
// Created by redwan on 8/2/20.
//

#ifndef AREACOVERAGE_TRIANGLEDECOMPOSITION_H
#define AREACOVERAGE_TRIANGLEDECOMPOSITION_H

#include <iostream>
#include <vector>
#include <deque>
#include <numeric>
#include <unordered_map>
#include <iterator>
#include <memory>
#include <cassert>
#include <algorithm>

using namespace std;
template <typename T>
inline void normalize(vector<T> &data)
{
    auto sum = accumulate(data.begin(), data.end(), 0);
    std::transform(data.begin(), data.end(), data.begin(), [&sum](T &val)
                   { return val / sum; });
}

template <typename T>
inline static void printMap(T &task)
{
    for (auto it = task.begin(); it != task.end(); ++it)
    {
        cout << it->first << " : ";
        copy(it->second.begin(), it->second.end(), ostream_iterator<double>(cout, " "));
        cout << endl;
    }
}

template <typename T>
class TriangleDecomposition
{
public:
    using TASK = std::unordered_map<int, std::deque<T>>;
    TriangleDecomposition();
    TriangleDecomposition(const vector<T> &triangles);
    virtual ~TriangleDecomposition();
    TASK get_uav_tasks()
    {
        return uav_task_;
    }
    TASK get_area_decomposition()
    {
        return decompose_;
    }

private:
    TASK uav_task_, decompose_;
    vector<T> triangles_;
    struct Node
    {
        double data; // area
        int id, triangle_id;
        std::vector<std::shared_ptr<Node>> children;
        std::weak_ptr<Node> parent;
        Node(T val, int num_child, std::shared_ptr<Node> parent = nullptr, int triangle_id = 0)
        {
            data = val;
            this->parent = parent;
            children.resize(num_child);
            id = 0;
            this->triangle_id = triangle_id;
        }
    };
    struct Tree
    {
        std::shared_ptr<Node> root;
    };

protected:
    void printParent(std::weak_ptr<Node> node, T &result);
    void m_decomposition(Tree *t, deque<T> uavs);
    void m_print_decomposition(weak_ptr<Node> node);

public:
    void solve(const deque<T> &UAVs);
    void allocate_task(std::weak_ptr<Node> node_, TASK &task, TASK &decompose);
};

template <typename T>
void TriangleDecomposition<T>::allocate_task(std::weak_ptr<Node> node_, TASK &task, TASK &decompose)
{
    auto node = node_.lock();
    if (!node)
        return;
    if (!node->children[0])
    {
        auto parent = node->parent;
        T result = 0;
        printParent(parent, result);
        T value = node->data;
        if (value > 1)
        {
            task[node->id].push_back(value);
            decompose[node->triangle_id].push_back(value);
            printf("%f-> (%d)\n", value, node->id);
        }
    }
    for (auto &child : node->children)
    {
        allocate_task(child, task, decompose);
    }
}

template <typename T>
void TriangleDecomposition<T>::printParent(std::weak_ptr<Node> node_, T &result)
{
    auto node = node_.lock();
    if (!node)
        return;
    result = node->data;
    printParent(node->parent.lock(), result);
    std::cout << node->data << " : ";
}

template <typename T>
void TriangleDecomposition<T>::m_print_decomposition(weak_ptr<Node> nodeWeak)
{
    auto node = nodeWeak.lock();
    if (!node)
        return;
    if (!node->children[0])
    {
        auto parent = node->parent;
        T result = 0;
        printParent(parent, result);
        int value = node->data;
        printf("%d -> (%d)\n", value, node->id);
    }
    for (auto &child : node->children)
    {
        m_print_decomposition(child);
    }
}

template <typename T>
void TriangleDecomposition<T>::m_decomposition(Tree *t, deque<T> uavs)
{

    std::deque<shared_ptr<Node>> q;
    for (auto &child : t->root->children)
        if (child)
            q.push_back(child);

    T d = 0;
    int N = uavs.size();
    while (!q.empty())
    {
        auto node = q.front(); // 40
        q.pop_front();
        auto a = node->data;
        auto u = uavs.front() - d;
        //        cout<< uavs.front() <<" "<<a << " " << d <<endl;
        uavs.pop_front();
        d = a - u;
        if (d < 0)
        {
            uavs.push_front(-d);
            d = 0;
        }
        else
        {
            node->children[0] = make_shared<Node>(u, 2, node, node->triangle_id);
            node->children[1] = make_shared<Node>(d, 2, node, node->triangle_id);
            node->children[0]->id = N - uavs.size() - 1;
            node->children[1]->id = N - uavs.size();
        }
        if (!uavs.empty() && d > uavs.front())
        {
            d = 0;
            q.push_front(node->children[1]);
        }
    }
}

template <typename T>
TriangleDecomposition<T>::TriangleDecomposition(const vector<T> &triangles) : triangles_(triangles)
{
}

template <typename T>
void TriangleDecomposition<T>::solve(const deque<T> &UAVs)
{

    double area = std::accumulate(triangles_.begin(), triangles_.end(), 0);
    shared_ptr<Node> init_node = make_shared<Node>(area, triangles_.size());
    auto t = new Tree();
    int i = 0;
    for (auto &a : triangles_)
    {
        init_node->children[i] = make_shared<Node>(a, 2, nullptr, i);
        ++i;
    }
    t->root = init_node;
    deque<T> uavs;
    copy(UAVs.begin(), UAVs.end(), back_inserter(uavs));
    m_decomposition(t, uavs);
    //    m_print_decomposition(t->root);

    assert(uav_task_.size() == 0);
    assert(decompose_.size() == 0);
    allocate_task(t->root, uav_task_, decompose_);

    printf("Task allocations \n");
    printMap(uav_task_);

    printf("decompose_ allocations \n");
    printMap(decompose_);

    delete t;
}

template <typename T>
TriangleDecomposition<T>::~TriangleDecomposition()
{
}

template <typename T>
TriangleDecomposition<T>::TriangleDecomposition()
{
}

#endif // AREACOVERAGE_TRIANGLEDECOMPOSITION_H
