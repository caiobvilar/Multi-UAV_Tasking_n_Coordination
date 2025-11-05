#ifndef TASKALLOCATION_H
#define TASKALLOCATION_H
#include <vector>
#include <queue>
#include <iostream>
#include <unordered_map>
#include "problem_definition.h"
// borrowed from https://www.geeksforgeeks.org/job-assignment-problem-using-branch-and-bound/

// state space tree node
struct Node
{
    // stores parent node of current node
    // helps in tracing path when answer is found
    Node *parent;

    // contains cost for ancestors nodes
    // including current node
    double pathCost;

    // contains least promising cost
    double cost;

    // contain worker number
    int workerID;

    // contains Job ID
    int jobID;

    // Boolean array assigned will contains
    // info about available jobs
    bool *assigned;
};

class TaskAllocation
{
    using MATRIX = std::vector<std::vector<Point2D>>;

public:
    TaskAllocation(const ProblemDefinition &pdf);

    double findMinCost(int N);

    std::pair<int, Point2D> operator()(int workerID);

    ~TaskAllocation();

private:
    MATRIX m_locmat;
    int m_size;
    double **m_costMatrix;
    std::unordered_map<int, std::pair<int, Point2D>> m_assignments;
    struct comp
    {
        bool operator()(const Node *lhs,
                        const Node *rhs) const
        {
            return lhs->cost > rhs->cost;
        }
    };

protected:
    void populateAssignments(Node *min);
    double polyLength(const std::vector<Point2D> &poly);
    // Function to allocate a new search tree node
    // Here Person x is assigned to job y
    Node *newNode(int x, int y, bool assigned[], Node *parent, int N);

    // Function to calculate the least promising cost
    // of node after worker x is assigned to job y.
    double calculateCost(int x, int j, bool assigned[], int N);
};

#endif // TASKALLOCATION_H
