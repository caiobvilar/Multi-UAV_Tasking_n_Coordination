#include "algo/taskallocation.h"
#include <algorithm>
#include <numeric>
#include <map>
#include <limits>

TaskAllocation::TaskAllocation(const ProblemDefinition &pdf)
{
    int N = m_size = pdf.size();
    m_costMatrix = new double *[N];

    m_locmat.resize(N);
    // pdf contains rois and initial positions of robots in normalized scale i.e., [0 - 1]
    for (int i = 0; i < pdf.size(); ++i)
    {
        m_costMatrix[i] = new double[N];
        m_locmat[i].resize(N);
        for (std::size_t j = 0; j < pdf.size(); ++j)
        {
            auto poly = pdf[j];
            int lastIndex = poly.size() - 1;
            auto firstPointToRobot = pdf.agents[i].first - poly[0];
            auto lastPointToRobot = pdf.agents[i].first - poly[lastIndex];
            //            auto totalCost = polyLength(poly) + std::min(firstPointToRobot, lastPointToRobot) + std::max(firstPointToRobot, lastPointToRobot);
            auto totalCost = polyLength(poly) + firstPointToRobot + lastPointToRobot;
            m_costMatrix[i][j] = totalCost;
            m_locmat[i][j] = (firstPointToRobot < lastPointToRobot) ? poly[0] : poly[lastIndex];
        }
    }

    //    for(int i = 0; i < pdf.size(); ++i)
    //        for(int j = 0; j < pdf.size(); ++j)
    //            std::cout << "[TaskAllocation] costMat = " << cost[i][j] << std::endl;

    std::cout << "[TaskAllocation] mission cost = " << findMinCost(N) << std::endl;
}

double TaskAllocation::polyLength(const std::vector<Point2D> &poly)
{
    if (poly.size() < 2)
        return std::numeric_limits<double>::max();

    double totalDist = 0;
    for (std::size_t i = 1; i < poly.size(); ++i)
    {
        totalDist += poly[i] - poly[i - 1];
    }
    return totalDist;
}

Node *TaskAllocation::newNode(int x, int y, bool assigned[], Node *parent, int N)
{
    Node *node = new Node;
    node->assigned = new bool[4];
    for (int j = 0; j < N; j++)
        node->assigned[j] = assigned[j];
    node->assigned[y] = true;

    node->parent = parent;
    node->workerID = x;
    node->jobID = y;

    return node;
}

double TaskAllocation::calculateCost(int x, int j, bool assigned[], int N)
{

    double cost = 0;

    // to store unavailable jobs
    std::vector<bool> available(N, false);
    // start from next worker
    for (int i = x + 1; i < N; i++)
    {
        int min = std::numeric_limits<int>::max(), minIndex = -1;

        // do for each job
        for (j = 0; j < N; j++)
        {
            // if job is unassigned
            if (!assigned[j] && available[j] &&
                m_costMatrix[i][j] < min)
            {
                // store job number
                minIndex = j;

                // store cost
                min = m_costMatrix[i][j];
            }
        }

        // add cost of next worker
        cost += min;

        // job becomes unavailable
        available[minIndex] = false;
    }

    return cost;
}

// double TaskAllocation::findMinCost(const TaskAllocation::MATRIX &m_costMatrix)
double TaskAllocation::findMinCost(int N)
{

    // Create a priority queue to store live nodes of
    // search tree;
    std::priority_queue<Node *, std::vector<Node *>, comp> pq;

    // initialize heap to dummy node with cost 0
    //        int N = m_costMatrix.size();
    //        std::vector<bool> assigned(N, false);
    std::vector<bool> assigned(N, false);
    assigned[0] = false;
    Node *root = newNode(-1, -1, assigned.data(), NULL, N);
    root->pathCost = root->cost = 0;
    root->workerID = -1;

    // Add dummy node to list of live nodes;
    pq.push(root);

    // Finds a live node with least cost,
    // add its childrens to list of live nodes and
    // finally deletes it from the list.

    while (!pq.empty())
    {
        // Find a live node with least estimated cost
        Node *min = pq.top();

        // The found node is deleted from the list of
        // live nodes
        pq.pop();

        // i stores next worker
        int i = min->workerID + 1;

        // if all workers are assigned a job
        if (i == N)
        {
            populateAssignments(min);
            return min->cost;
        }

        // do for each job
        for (int j = 0; j < N; j++)
        {
            // If unassigned
            if (!min->assigned[j])
            {
                // create a new tree node
                Node *child = newNode(i, j, min->assigned, min, N);

                // cost for ancestors nodes including current node
                child->pathCost = min->pathCost + m_costMatrix[i][j];

                // calculate its lower bound
                child->cost = child->pathCost + calculateCost(i, j, child->assigned, N);
                //              computeCost(m_costMatrix, i, j, child->assigned);

                // Add child to list of live nodes;
                pq.push(child);
            }
        }
    }
    // TODO/ERR
    return std::numeric_limits<double>::quiet_NaN();
}

std::pair<int, Point2D> TaskAllocation::operator()(int workerID)
{
    if (m_assignments.find(workerID) == m_assignments.end())
        return {};
    return m_assignments[workerID];
}

TaskAllocation::~TaskAllocation()
{

    if (m_costMatrix != nullptr)
    {
        for (int i = 0; i < m_size; i++)
            delete[] m_costMatrix[i];
        delete[] m_costMatrix;
    }
}

void TaskAllocation::populateAssignments(Node *min)
{
    if (min->parent == NULL)
        return;

    populateAssignments(min->parent);
    m_assignments[min->workerID] = std::make_pair(min->jobID, m_locmat[min->workerID][min->jobID]);
    //        std::cout << "\n Assign Robot " << char(min->workerID + 'A')
    //             << " to Job " << min->jobID << std::endl;
}
