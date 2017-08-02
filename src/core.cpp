#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>

#include "core.h"

using namespace config;

namespace
{

// constexpr BodyIndex_t invalidBodyIdx = std::numeric_limits<BodyIndex_t>::max();
constexpr NodeIndex_t invalidNodeIdx = std::numeric_limits<NodeIndex_t>::max();

constexpr double defaultInitRadius = 60E3 * LY;

void initBodiesSphereExt(std::vector<Body> &bodies, const double radius)
{
    const size_t numBodies = bodies.size();
    for (size_t i = 0; i < numBodies; ++i) {
        Body &body = bodies[i];
        body.position = glm::ballRand<double>(radius);
        body.force = { 0, 0, 0 };
        body.mass = double(TOTAL_MASS) / numBodies;
        body.speed = { 0, 0, 0 };
    }
}

void initBodiesSphere(std::vector<Body> &bodies)
{
    initBodiesSphereExt(bodies, defaultInitRadius);
}

void initBodies4Spheres(std::vector<Body> &bodies)
{
    const size_t numBodies = bodies.size();

    initBodiesSphereExt(bodies, defaultInitRadius);

    const Vec3d offset = 1.2 * Vec3d{ defaultInitRadius, defaultInitRadius, 0.0 };

    for (size_t i = 0; i < numBodies / 4; ++i) {
        bodies[i].position += offset;
    }
    for (size_t i = numBodies / 4; i < numBodies / 2; ++i) {
        bodies[i].position += offset * Vec3d(-1.0, 1.0, 0.0);
    }
    for (size_t i = numBodies / 2; i < 3 * numBodies / 4; ++i) {
        bodies[i].position += offset * Vec3d(1.0, -1.0, 0.0);
    }
    for (size_t i = 3 * numBodies / 4; i < numBodies; ++i) {
        bodies[i].position += offset * Vec3d(-1.0, -1.0, 0.0);
    }
}

bool initializeBodies(std::vector<Body> &bodies, std::string scheme)
{
    for (char & c : scheme) {
        if (c >= 'A' && c <= 'Z') {
            c = (c - 'A') + 'a';
        }
    }

    std::map<std::string, std::function<void(std::vector<Body> &)>> schemes = {
        { "sphere", &initBodiesSphere },
        { "4spheres", &initBodies4Spheres },
    };

    if (scheme.empty()) {
        scheme = "sphere";
    }

    const auto it = schemes.find(scheme);
    if (it == schemes.end()) {
        std::cerr << "Invalid body initialization scheme: " << scheme
            << " Available schemes are: ";
        size_t i = 0;
        for (const auto & pair : schemes) {
            std::cerr << pair.first;
            if (++i != schemes.size()) {
                std::cerr << ", ";
            }
        }
        std::cerr << std::endl;
        return false;
    }

    it->second(bodies);
    return true;
}

}

int64_t timeDiffNanoSecs(timepoint_t start, timepoint_t end)
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
    if (m_nodes[nodeIndex].bodies_quantity() == 1) {
        m_rootIndices.push_back(nodeIndex);
        return;
    }
    if (m_nodes[nodeIndex].bodies_quantity() == 0) {
        return;
    }
    const double sx = m_nodes[nodeIndex].start.x;
    const double sy = m_nodes[nodeIndex].start.y;
    const double sz = m_nodes[nodeIndex].start.z;
    const double ex = m_nodes[nodeIndex].end.x;
    const double ey = m_nodes[nodeIndex].end.y;
    const double ez = m_nodes[nodeIndex].end.z;
    const double mx = (sx + ex) / 2.0;
    const double my = (sy + ey) / 2.0;
    const double mz = (sz + ez) / 2.0;
    for (const uint32_t bodyIndex : m_nodes[nodeIndex].bodies()) {
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
                    if (m_nodes[nodeIndex].UNW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, sy, sz, mx, my, mz, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].UNW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].UNW].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (m_nodes[nodeIndex].DNW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, sy, mz, mx, my, ez, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].DNW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].DNW].addBody(indexOfBody(body));
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (m_nodes[nodeIndex].USW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, my, sz, mx, ey, mz, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].USW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].USW].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (m_nodes[nodeIndex].DSW == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            sx, my, mz, mx, ey, ez, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].DSW = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].DSW].addBody(indexOfBody(body));
                }
            }
        }
        else {      // x >= mx
            if (body->position.y < my) {
                if (body->position.z < mz) {
                    if (m_nodes[nodeIndex].UNE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, sy, sz, ex, my, mz, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].UNE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].UNE].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (m_nodes[nodeIndex].DNE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, sy, mz, ex, my, ez, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].DNE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].DNE].addBody(indexOfBody(body));
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (m_nodes[nodeIndex].USE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, my, sz, ex, ey, mz, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].USE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].USE].addBody(indexOfBody(body));
                }
                else {    // z >= mz
                    if (m_nodes[nodeIndex].DSE == invalidNodeIdx) {
                        m_nodes.emplace_back(nodeIndex,
                            mx, my, mz, ex, ey, ez, m_nodes[nodeIndex].depth + 1);
                        m_nodes[nodeIndex].DSE = NodeIndex_t(m_nodes.size() - 1);
                    }
                    m_nodes[m_nodes[nodeIndex].DSE].addBody(indexOfBody(body));
                }
            }
        }
    }
    divideNode(m_nodes[nodeIndex].UNW);
    divideNode(m_nodes[nodeIndex].UNE);
    divideNode(m_nodes[nodeIndex].USW);
    divideNode(m_nodes[nodeIndex].USE);
    divideNode(m_nodes[nodeIndex].DNW);
    divideNode(m_nodes[nodeIndex].DNE);
    divideNode(m_nodes[nodeIndex].DSW);
    divideNode(m_nodes[nodeIndex].DSE);
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
        const double f = K * m_centerOfMass.mass * body.mass / DistanceSquared;
        const Vec3d additionalForce = xyzDist / std::sqrt(DistanceSquared) * f;
        const std::lock_guard<std::mutex> lock{ body.mutex };
        body.force += additionalForce;
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

bool Model::init(const std::string& bodiesInitScheme, const size_t numBodies)
{
    m_frameCount = 0;
    if (!outputFileName.empty()) {
        m_outputPositions_f = std::fopen(outputFileName.c_str(), "wb");
    }

    m_nodes.emplace_back(invalidNodeIdx,
        -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30,
        SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30,
        0);

    if (!inputFileName.empty()) {
        std::ifstream file(inputFileName);
        if (!file.good()) {
            std::cerr << "Invalid input file: " << inputFileName << std::endl;
            return false;
        }
        std::string line;
        Body body;
        while (!file.eof()) {
            std::getline(file, line);
            if (line.empty()) {
                continue;
            }
            std::stringstream lineStream(line);
            lineStream >> body.position.x >> body.position.y >> body.position.z;
            lineStream >> body.mass;
            lineStream >> body.speed.x >> body.speed.y >> body.speed.z;
            if (!lineStream.eof()) {
                std::cerr << "Invalid line in input file: " << line << std::endl;
                return false;
            }
            m_bodies.emplace_back(body);
        }
        m_bodies.shrink_to_fit();
        m_nodes.reserve(m_bodies.size());
        m_rootIndices.reserve(m_bodies.size());

    }
    else {
        m_nodes.reserve(numBodies);
        m_rootIndices.reserve(numBodies);
        m_bodies.resize(numBodies);
        if (!initializeBodies(m_bodies, bodiesInitScheme)) {
            return false;
        }
    }

    for (BodyIndex_t i = 0; i < m_bodies.size(); ++i) {
        m_nodes[0].addBody(i);
    }

    m_startTime = std::chrono::high_resolution_clock::now();

    return true;
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
        m_totalRuntimeSecs = timeDiffNanoSecs(m_startTime, m_endTime) * 1e-9;
        printf("%f\n", m_totalRuntimeSecs);
        return false;
    }
    m_rootIndices.clear();

    /** Build the tree for the current spatial body distribution. */
    resetNodes();
    divideNode(0);

    /**
     * Update body parameters.
     * These loops implicitly swap between old and new body parameters:
     * The previous state is stored in the nodes' centers of mass, which are updated in the first
     * loop. The new state is written to the m_bodies in the second loop (with its nested loops).
     */

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
    #pragma omp parallel for
    for (ptrdiff_t i = 0; i < ptrdiff_t(m_bodies.size()); i++) {
        Body& body = m_bodies[i];
        body.speed += body.force / body.mass;
        body.position += body.speed * 50E12;
        body.force = { 0, 0, 0 };
    }
    exportBodies();
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

bool Model::exportBodies()
{
    if (!m_outputPositions_f) {
        return false;
    }
    for (const Body& body : m_bodies) {
        fprintf(m_outputPositions_f, "%g %g %g %g %g %g %g\n",
            body.position.x, body.position.y, body.position.z,
            body.mass,
            body.speed.x, body.speed.y, body.speed.z);
    }
    return true;
}
