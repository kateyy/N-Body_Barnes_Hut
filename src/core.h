#pragma once

#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

#include <glm/vec3.hpp>

#include "config.h"

using Vec3d = glm::tvec3<double>;


using BodyIndex_t = uint32_t;
using NodeIndex_t = size_t;

/**
 * An entity.
 *
 * Note: The mutex must only be used in contexts where it is guaranteed that the Body will not be
 * copied or moved. The mutex is not copied/moved with the body!
 */
struct Body
{
    Vec3d position;
    Vec3d force;
    Vec3d speed;
    double mass;
    std::mutex mutex;

    friend void swap(Body &lhs, Body &rhs) noexcept
    {
        using std::swap;
        swap(lhs.position, rhs.position);
        swap(lhs.force, rhs.force);
        swap(lhs.speed, rhs.speed);
        swap(lhs.mass, rhs.mass);
    }

    Body() noexcept {}
    ~Body() noexcept {}
    Body(const Vec3d &position, const Vec3d &force, const Vec3d &speed, double mass) noexcept
        : position{ position }, force{ force }, speed{ speed }, mass{ mass } {}
    Body(const Body &other) noexcept
        : position{ other.position }, force{ other.force }, speed{ other.speed }
        , mass{ other.mass }
    {
    }
    Body(Body &&other) noexcept : Body()
    {
        swap(*this, other);
    }
    Body& operator=(Body other) noexcept {
        swap(*this, other);
        return *this;
    }
};


using timepoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
int64_t timeDiffNanonSecs(timepoint_t start, timepoint_t end);


class Node
{
public:
    Node();
    explicit Node(NodeIndex_t up,
        double sx, double sy, double sz,
        double ex, double ey, double ez,
        int depth);
    ~Node();
    Node(Node &&other);
    Node& operator=(Node other);
    friend void swap(Node &lhs, Node &rhs);

    void reset();
    /** Reset neighborhood, preserve bodies, start and end. */
    void resetNeighborhood();

    void addBody(BodyIndex_t index) { m_bodyIndices.push_back(index); }
    const std::vector<BodyIndex_t>& bodies() const { return m_bodyIndices; }
    size_t bodies_quantity() const { return m_bodyIndices.size(); }
    const Body& centerOfMass() const { return m_centerOfMass; };
    void updateCenterOfMass(const std::vector<Body> &allBodies);

    bool applyForceTo(Body &body) const;

    Vec3d start;
    Vec3d end;
    int depth;
    NodeIndex_t UNE;
    NodeIndex_t UNW;
    NodeIndex_t USE;
    NodeIndex_t USW;
    NodeIndex_t DNE;
    NodeIndex_t DNW;
    NodeIndex_t DSE;
    NodeIndex_t DSW;
    NodeIndex_t UP;

private:
    std::vector<BodyIndex_t> m_bodyIndices;
    Body m_centerOfMass;
};


class Model
{
public:
    Model();
    ~Model();

    void setFrameLimit(size_t frameLimit) {
        m_frameLimit = frameLimit;
    }
    bool init(
        const std::string& bodiesInitScheme,
        size_t numBodies);

    /**
     * Run the model in a separate thread, which is managed internally.
     */
    void run();
    size_t frameCount() const {
        return m_frameCount;
    }
    void pause(bool doPause = true);
    void stopAndWait();

    struct LockedData
    {
        LockedData(const std::vector<Body> &bodies, const std::vector<Node> &nodes, std::mutex & mutex)
            : bodies{ bodies }
            , nodes{ nodes }
            , lock{ mutex }
        {
        }
        LockedData(LockedData &&) = default;
        const std::vector<Body> &bodies;
        const std::vector<Node> &nodes;
    private:
        std::unique_lock<std::mutex> lock;
    };

    /**
     * Request locked access to the model data.
     * Access is locked as long as the returned object is not destroyed, so make sure to delete it
     * as soon as you don't require it anymore!
     */
    LockedData lockedData();

    /**
     * Update the model in an infinite loop.
     * This does not lock access to bodies(), so don't try to access model results/states when
     * using the bench mode.
     */
    void benchMode();
    
    /** Write current bodies to the outputFileName. */
    bool exportBodies();


    bool render;
    bool visualMode;
    std::string inputFileName;
    std::string outputFileName;

private:
    bool update();
    bool updateUnlocked();
    void resetNodes();
    void divideNode(NodeIndex_t nodeIdx);
    void forceOverNode(NodeIndex_t nodeIdx, NodeIndex_t downIdx, Body &body, bool inverse);
    NodeIndex_t indexOfNode(const Node *node) const {
        assert(node >= &m_nodes.front());
        const size_t index = node - &m_nodes.front();
        assert(index < m_nodes.size() && &m_nodes[index] == node);
        return static_cast<NodeIndex_t>(index);
    }
    BodyIndex_t indexOfBody(const Body *body) const {
        assert(body >= &m_bodies.front());
        const size_t index = body - &m_bodies.front();
        assert(index < m_bodies.size() && &m_bodies[index] == body);
        return static_cast<BodyIndex_t>(index);
    }

private:
    std::unique_ptr<std::thread> m_runThread;
    std::mutex m_dataMutex;
    std::unique_lock<std::mutex> m_pauseLock;
    bool m_stopRequested;
    std::vector<Node> m_nodes;
    std::vector<NodeIndex_t> m_rootIndices;
    std::vector<Body> m_bodies;
    std::FILE * m_outputPositions_f;
    size_t m_frameLimit;
    size_t m_frameCount;
    timepoint_t m_startTime;
    timepoint_t m_endTime;
};
