#pragma once

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <utility>

#include <glm/vec3.hpp>

using Vec3d = glm::tvec3<double>;

struct Color
{
    float r;
    float g;
    float b;
};

struct Body
{
    Vec3d position;
    Vec3d force;
    Vec3d speed;
    double acel;
    double mass;

    void applyForceFrom(const Body &other);
};


using timepoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
int64_t timeDiffNanonSecs(timepoint_t start, timepoint_t end);


class Node
{
public:
    Node();
    explicit Node(Node *up,
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

    void addBody(Body * body);
    size_t bodies_quantity() const {
        return m_bodies.size();
    }
    const std::vector<Body *> & bodies() const {
        return m_bodies;
    }
    std::vector<Body *> & bodies() {
        return m_bodies;
    }

    const Body& centerOfMass() const {
        return m_centerOfMass;
    };
    void updateCenterOfMass();

    bool applyForceTo(Body &body) const;

    Vec3d start;
    Vec3d end;
    int depth;
    Node *UNE;
    Node *UNW;
    Node *USE;
    Node *USW;
    Node *DNE;
    Node *DNW;
    Node *DSE;
    Node *DSW;
    Node *UP;

private:
    std::vector<Body *> m_bodies;
    Body m_centerOfMass;
};


/* A coloured pixel. */

struct Pixel
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

/* A picture. */
class Bitmap
{
public:
    Bitmap();
    ~Bitmap();
    uint32_t width() const {
        return m_width;
    }
    uint32_t height() const {
        return m_height;
    }
    Pixel & at(uint32_t x, uint32_t y);
    const Pixel& at(uint32_t x, uint32_t y) const;

    bool toPngFile(const std::string & filePath) const;
private:
    uint32_t m_width;
    uint32_t m_height;
    std::unique_ptr<Pixel[]> m_pixels;
};


/**
 * std::vector-style data structure that does not change item memory location on insert/remove.
 *
 * push_back, emplace_back etc. are only allowed to insert as many elements as specified by
 * reserve(). That way, persistent pointers on elements in the storage can be used, as long as the
 * storage is not explicitly resized.
 */
template<typename T>
class FixedStorage
{
public:
    using Data_t = std::vector<T>;
    using iterator = typename Data_t::iterator;
    using const_iterator = typename Data_t::iterator;

    FixedStorage()
        : m_size{ 0 }
        , m_data{}
    {
    }
    ~FixedStorage() noexcept = default;

    iterator begin() noexcept {
        return m_data.begin();
    }
    const_iterator begin() const noexcept {
        return m_data.begin();
    }
    iterator end() noexcept {
        return m_data.begin() + m_size;
    }
    const_iterator end() const noexcept {
        return m_data.begin() + m_size;
    }

    operator const Data_t&() const noexcept {
        return m_data;
    }

    void reserve(const size_t size) {
        if (size <= m_size) {
            return;
        }
        m_data.resize(size);
    }
    void resize(const size_t size) {
        m_size = size;
        m_data.resize(size);
    }
    size_t reservedSize() const noexcept {
        return m_data.size();
    }
    size_t size() const noexcept {
        return m_size;
    }
    void push_back(const T &item) {
        assert(m_size < m_data.size());
        m_data[m_size++] = item;
    }
    void push_back(T &&item) {
        emplace_back(std::move(item));
    }
    template<typename... Params>
    void emplace_back(Params&&... params) {
        assert(m_size < m_data.size());
        m_data[m_size] = T(std::forward<Params>(params)...);
        ++m_size;
    }
    T& operator[](const size_t index) noexcept {
        assert(index < m_size);
        return m_data[index];
    }
    const T& operator[](const size_t index) const noexcept {
        assert(index < m_size);
        return m_data[index];
    }
    T& front() noexcept {
        return operator[](0u);
    }
    const T& front() const noexcept {
        return operator[](0u);
    }
    T& back() noexcept {
        return operator[](m_size - 1);
    }
    const T& back() const noexcept {
        return operator[](m_size - 1);
    }

private:
    size_t m_size;
    Data_t m_data;
};

class Model
{
public:
    Model();
    ~Model();

    void setFrameLimit(size_t frameLimit) {
        m_frameLimit = frameLimit;
    }
    void init(size_t numBodies, bool writeToFile = false);

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


    bool render;
    bool visualMode;

private:
    void update();
    void updateUnlocked();
    void resetNodes();
    void divideNode(Node * node);

private:
    std::unique_ptr<std::thread> m_runThread;
    std::mutex m_dataMutex;
    std::unique_lock<std::mutex> m_pauseLock;
    bool m_stopRequested;
    FixedStorage<Node> m_nodes;
    std::vector<Node *> m_roots;
    FixedStorage<Body> m_bodies;
    std::FILE * m_outputPositions_f;
    size_t m_frameLimit;
    size_t m_frameCount;
    timepoint_t m_startTime;
    timepoint_t m_endTime;
};
