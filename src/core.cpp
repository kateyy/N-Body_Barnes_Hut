#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <iomanip>
#include <iostream>

#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>

#include "core.h"

constexpr double colorMax = 3E4;
using namespace config;

namespace
{
constexpr BodyIndex_t invalidBodyIdx = std::numeric_limits<BodyIndex_t>::max();
constexpr NodeIndex_t invalidNodeIdx = std::numeric_limits<NodeIndex_t>::max();
}

int64_t timeDiffNanonSecs(timepoint_t start, timepoint_t end)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
}

Node::Node()
    : Node(invalidNodeIdx,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        -1)
{
}

Node::Node(const NodeIndex_t up,
    double sx, double sy, double sz,
    double ex, double ey, double ez,
    int depth)
    : start{ sx, sy, sz }
    , end{ ex, ey, ez }
    , depth{ depth }
    , UNE{ invalidNodeIdx }
    , UNW{ invalidNodeIdx }
    , USE{ invalidNodeIdx }
    , USW{ invalidNodeIdx }
    , DNE{ invalidNodeIdx }
    , DNW{ invalidNodeIdx }
    , DSE{ invalidNodeIdx }
    , DSW{ invalidNodeIdx }
    , UP{ up }
{
}

Node::~Node() = default;

Node::Node(Node &&other)
    : Node()
{
    swap(*this, other);
}

Node& Node::operator=(Node other)
{
    swap(*this, other);
    return *this;
}

void swap(Node &lhs, Node &rhs)
{
    using std::swap;
    swap(lhs.start, rhs.start);
    swap(lhs.end, rhs.end);
    swap(lhs.depth, rhs.depth);
    swap(lhs.UNE, rhs.UNE);
    swap(lhs.UNW, rhs.UNW);
    swap(lhs.USE, rhs.USE);
    swap(lhs.USW, rhs.USW);
    swap(lhs.DNE, rhs.DNE);
    swap(lhs.DNW, rhs.DNW);
    swap(lhs.DSE, rhs.DSE);
    swap(lhs.DSW, rhs.DSW);
    swap(lhs.UP, rhs.UP);
    swap(lhs.m_bodyIndices, rhs.m_bodyIndices);
    swap(lhs.m_centerOfMass, rhs.m_centerOfMass);
}

void Node::reset()
{
    Node null;
    swap(*this, null);
}

void Node::resetNeighborhood()
{
    Node null;
    null.start = start;
    null.end = end;
    null.m_bodyIndices = std::move(m_bodyIndices);
    swap(*this, null);
}

Model::Model()
    : render{ true }
    , visualMode{ true }
    , m_stopRequested{ false }
    , m_outputPositions_f{ nullptr }
    , m_frameLimit{ 0 }
    , m_frameCount{ 0 }
{
}

Model::~Model()
{
    if (m_outputPositions_f) {
        std::fclose(m_outputPositions_f);
    }
}

void Model::resetNodes()
{
    m_nodes.resize(1);
    m_nodes.front().resetNeighborhood();
    m_nodes.front().depth = 0;
}

void Model::divideNode(const NodeIndex_t nodeIndex)
{
    if (nodeIndex == invalidNodeIdx) {
        return;
    }
    Node &node = m_nodes[nodeIndex];
    if (node.bodies_quantity() == 1) {
        m_rootIndices.push_back(nodeIndex);
        return;
    }
    if (node.bodies_quantity() == 0) {
        return;
    }
    const double sx = node.start.x;
    const double sy = node.start.y;
    const double sz = node.start.z;
    const double ex = node.end.x;
    const double ey = node.end.y;
    const double ez = node.end.z;
    const double mx = (sx + ex) / 2.0;
    const double my = (sy + ey) / 2.0;
    const double mz = (sz + ez) / 2.0;
    for (const uint32_t bodyIndex : node.bodies()) {
        const Body *body = &m_bodies[bodyIndex];
        if (body->position.x < sx
            || body->position.x >= ex
            || body->position.y < sy
            || body->position.y >= ey
            || body->position.z < sz
            || body->position.z >= ez)
            continue;
        if (body->position.x < mx) {
            if (body->position.y < my) {
                if (body->position.z < mz) {
                    if (node.UNW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, sy, sz, mx, my, mz, node.depth + 1);
                        node.UNW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.UNW].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (node.DNW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, sy, mz, mx, my, ez, node.depth + 1);
                        node.DNW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.DNW].addBody(indexOfBody(body));
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (node.USW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, my, sz, mx, ey, mz, node.depth + 1);
                        node.USW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.USW].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (node.DSW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, my, mz, mx, ey, ez, node.depth + 1);
                        node.DSW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.DSW].addBody(indexOfBody(body));
                }
            }
        }
        else {      // x >= mx
            if (body->position.y < my) {
                if (body->position.z < mz) {
                    if (node.UNE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, sy, sz, ex, my, mz, node.depth + 1);
                        node.UNE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.UNE].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (node.DNE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, sy, mz, ex, my, ez, node.depth + 1);
                        node.DNE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.DNE].addBody(indexOfBody(body));
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (node.USE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, my, sz, ex, ey, mz, node.depth + 1);
                        node.USE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.USE].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (node.DSE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, my, mz, ex, ey, ez, node.depth + 1);
                        node.DSE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[node.DSE].addBody(indexOfBody(body));
                }
            }
        }
    }
    divideNode(node.UNW);
    divideNode(node.UNE);
    divideNode(node.USW);
    divideNode(node.USE);
    divideNode(node.DNW);
    divideNode(node.DNE);
    divideNode(node.DSW);
    divideNode(node.DSE);
}

void Node::updateCenterOfMass(const std::vector<Body> &allBodies)
{
    m_centerOfMass.position = { 0, 0, 0 };
    m_centerOfMass.mass = 0;
    if (bodies_quantity() >= 1) {
        for (const BodyIndex_t i : m_bodyIndices) {
            const Body& body = allBodies[i];
            const auto totalSpeed = glm::length(body.speed);
            const double relativisticAjust = 1 /
                std::sqrt(1 - (totalSpeed * totalSpeed) / (C * C));
            m_centerOfMass.position += body.position * body.mass * relativisticAjust;
            m_centerOfMass.mass += body.mass * relativisticAjust;
        }
        m_centerOfMass.position /= m_centerOfMass.mass;
    }
}

bool Node::applyForceTo(Body &body) const
{
    Vec3d xyzDist = body.position - (end + start) * 0.5;
    const double distance = glm::length(xyzDist);
    if ((this->end.x - this->start.x) / distance < ALPHA
        || this->bodies_quantity() == 1) {
        xyzDist = m_centerOfMass.position - body.position;
        const double DistanceSquared = glm::length2(xyzDist) + EPS2;
        const double force = K * m_centerOfMass.mass * body.mass / DistanceSquared;
        body.force += xyzDist / std::sqrt(DistanceSquared) * force;
        return true;
    }
    else {
        return false;
    }
}

void Model::forceOverNode(NodeIndex_t nodeIdx, NodeIndex_t downIdx, Body &body, bool inverse)
{
    if (nodeIdx == invalidNodeIdx) {
        return;
    }

    const Node& node = m_nodes[nodeIdx];

    for (const NodeIndex_t childIdx : { node.UNE, node.UNW, node.USE, node.USW,
        node.DNE, node.DNW, node.DSE, node.DSW}) {
        if (childIdx != invalidNodeIdx && (childIdx != downIdx || inverse)) {
            const bool canApproximate = m_nodes[childIdx].applyForceTo(body);
            if (!canApproximate) {
                forceOverNode(childIdx, nodeIdx, body, true);
            }
        }
    }

    if (!inverse)
        forceOverNode(node.UP, nodeIdx, body, false);
    return;
}

void Model::init(const size_t numBodies, const bool writeToFile)
{
    m_frameCount = 0;
    if (writeToFile) {
        m_outputPositions_f = std::fopen("positionData.csv", "wb");
    }
    m_nodes.reserve(numBodies);
    m_rootIndices.reserve(numBodies);
    m_bodies.resize(numBodies);

    m_nodes.emplace_back(invalidNodeIdx,
        -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30,
        SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30,
        0);

    constexpr double radius = 60E3 * LY;

    for (BodyIndex_t i = 0; i < m_bodies.size(); ++i) {
        Body& body = m_bodies[i];
        body.position = glm::ballRand<double>(radius);
        body.force.x = 0;
        body.force.y = 0;
        body.force.z = 0;
        body.mass = double(TOTAL_MASS) / numBodies;
        body.speed.x = 0;
        body.speed.y = 0;
        body.speed.z = 0;
        m_nodes[0].addBody(i);
    }

    m_startTime = std::chrono::high_resolution_clock::now();
}

void Model::run()
{
    if (m_runThread) {
        return;
    }

    auto loop = [this] () {
        while (!m_stopRequested) {
            if (!update()) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    };

    m_runThread = std::make_unique<std::thread>(loop);
}

void Model::pause(const bool doPause)
{
    if (!doPause) {
        m_pauseLock.release();
        return;
    }

    if (m_pauseLock.owns_lock()) {
        return;
    }

    m_pauseLock = std::unique_lock<std::mutex>{ m_dataMutex };
}

void Model::stopAndWait()
{
    if (!m_runThread) {
        return;
    }
    m_stopRequested = true;
    m_runThread->join();
}

bool Model::update()
{
    const std::lock_guard<std::mutex> lock{ m_dataMutex };
    return updateUnlocked();
}

bool Model::updateUnlocked()
{
    const auto startTime = std::chrono::high_resolution_clock::now();

    if (m_frameLimit > 0 && m_frameCount == m_frameLimit) {
        m_endTime = std::chrono::high_resolution_clock::now();
        printf("Simulation time Elapsed = %f\n",
            timeDiffNanonSecs(m_startTime, m_endTime) / 1e9);
        return false;
    }
    m_rootIndices.clear();
    resetNodes();
    divideNode(0);
#pragma omp parallel for
    for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(m_nodes.size()); ++i) {
        m_nodes[i].updateCenterOfMass(m_bodies);
    }
  #pragma omp parallel for
    for (ptrdiff_t ri = 0; ri < static_cast<ptrdiff_t>(m_rootIndices.size()); ++ri) {
        const NodeIndex_t rootIdx = m_rootIndices[ri];
        const Node& root = m_nodes[rootIdx];
  #pragma omp parallel for
        for (ptrdiff_t bodyIdx = 0; bodyIdx < static_cast<ptrdiff_t>(root.bodies().size()); ++bodyIdx) {
            forceOverNode(rootIdx, invalidNodeIdx, m_bodies[root.bodies()[bodyIdx]], false);
        }
    }
    if (m_outputPositions_f) {
        fprintf(m_outputPositions_f, "FF\n");
    }
    #pragma omp parallel for
    for (ptrdiff_t i = 0; i < ptrdiff_t(m_bodies.size()); i++) {
        Body& body = m_bodies[i];
        body.speed += body.force / body.mass;
        body.acel = glm::length(body.speed) / colorMax;
        body.position += body.speed * 50E12;
        body.force = { 0, 0, 0 };
    }
    if (m_outputPositions_f) {
        for (const Body& body : m_bodies) {
            fprintf(m_outputPositions_f, "%i,%i,%i,%i\n",
                int(body.position.x * 10E-16),
                int(body.position.y * 10E-16),
                int(body.position.z * 10E-16),
                int(body.acel));
        }
    }
    m_frameCount++;
    if (PRINT_TIMINGS)
    {
        const auto endTime = std::chrono::high_resolution_clock::now();
        std::cout << "Compute time: " << std::setw(11) << timeDiffNanonSecs(startTime, endTime) / 1000
            << "micro seconds, Frame: " << m_frameCount << std::endl;
    }
    return true;
}

Model::LockedData Model::lockedData()
{
    return LockedData(m_bodies, m_nodes, m_dataMutex);
}

void Model::benchMode() {
    while (updateUnlocked()) {}
}
