#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <utility>

#include <glm/vec3.hpp>

#include "config.h"

#ifdef PERSISTENT_NODE_VECTOR
#include "PersistentVector.hpp"
#endif

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
#if BODY_VALUE_STRIDE && BODY_VALUE_STRIDE > 0
#define PADDING(PREVTYPE, NAME) std::array<char, BODY_VALUE_STRIDE - sizeof(PREVTYPE)> NAME;
#else
#define PADDING(PREVTYPE, NAME)
#endif

    Vec3d position;
    PADDING(Vec3d, padding1)
    Vec3d force;
    PADDING(Vec3d, padding2)
    Vec3d speed;
    PADDING(Vec3d, padding3)
    double mass;
    PADDING(double, padding4)
    std::mutex mutex;
    PADDING(std::mutex, padding5)

    friend void swap(Body &lhs, Body &rhs) noexcept
    {
        using std::swap;
        swap(lhs.position, rhs.position);
        swap(lhs.force, rhs.force);
        swap(lhs.speed, rhs.speed);
        swap(lhs.mass, rhs.mass);
    }

    Body() noexcept {
#if defined(BODY_INFLATE_BYTES) && BODY_INFLATE_BYTES > 0
        std::fill(inflateBytes.begin(), inflateBytes.end(), char(1));
#endif
    }
    ~Body() noexcept {}
    Body(const Vec3d &position, const Vec3d &force, const Vec3d &speed, double mass) noexcept
        : position{ position }, force{ force }, speed{ speed }, mass{ mass } {}
    Body(const Body &other) noexcept
        : position{ other.position }, force{ other.force }, speed{ other.speed }
        , mass{ other.mass }
    {
#if defined(BODY_INFLATE_BYTES) && BODY_INFLATE_BYTES > 0
        std::copy(other.inflateBytes.begin(), other.inflateBytes.end(), inflateBytes.begin());
#endif
    }
    Body(Body &&other) noexcept : Body()
    {
        swap(*this, other);
    }
    Body& operator=(Body other) noexcept {
        swap(*this, other);
        return *this;
    }
#if defined(BODY_INFLATE_BYTES) && BODY_INFLATE_BYTES > 0
    std::array<char, BODY_INFLATE_BYTES> inflateBytes;
#endif
};


using timepoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
int64_t timeDiffNanoSecs(timepoint_t start, timepoint_t end);


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

#ifdef PERSISTENT_NODE_VECTOR
    // Special weak initialization function for PersistentVector, assuming that
    // reset() or the constructor was called before.
    void initializeValues(NodeIndex_t up,
        double sx, double sy, double sz,
        double ex, double ey, double ez,
        int depth) {
        this->UP = up;
        this->start = { sx, sy, sz };
        this->end = { ex, ey, ez };
        this->depth = depth;
    }
#endif

    void reset();
    /** Reset neighborhood, preserve bodies, start and end. */
    void resetNeighborhood();

    void addBody(BodyIndex_t index) { m_bodyIndices.push_back(index); }
    const std::vector<BodyIndex_t>& bodies() const { return m_bodyIndices; }
    size_t bodies_quantity() const { return m_bodyIndices.size(); }
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
#ifdef CENTER_OF_MASS_IS_BODY
    using CenterOfMass = Body;
#else
    struct CenterOfMass {
        Vec3d position;
        double mass;
    };
#endif

private:
    std::vector<BodyIndex_t> m_bodyIndices;
    CenterOfMass m_centerOfMass;
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

#ifdef PERSISTENT_NODE_VECTOR
    using NodeVector = PersistentVector<Node,
        void(Node::*)(NodeIndex_t, double, double, double, double, double, double, int),
        &Node::initializeValues,
        &Node::reset>;
#else
    using NodeVector = std::vector<Node>;
#endif

    struct LockedData
    {
        LockedData(const std::vector<Body> &bodies, const NodeVector &nodes, std::mutex & mutex)
            : bodies{ bodies }
            , nodes{ nodes }
            , lock{ mutex }
        {
        }
        LockedData(LockedData &&) = default;
        const std::vector<Body> &bodies;
        const NodeVector &nodes;
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
    double totalRuntimeSeconds() const { return m_totalRuntimeSecs; }
    double runtimeFirstNFrames() const { return m_excludeNFramesTime; }
    
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
    NodeVector m_nodes;
    std::vector<NodeIndex_t> m_rootIndices;
    std::vector<Body> m_bodies;
    std::FILE * m_outputPositions_f;
    size_t m_frameLimit;
    size_t m_frameCount;
    timepoint_t m_startTime;
    timepoint_t m_endTime;
    double m_totalRuntimeSecs;
    double m_excludeNFramesTime;
};
