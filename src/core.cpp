#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>

#include <png.h>

#include "config.h"
#include "core.h"

constexpr double colorMax = 3E4;
using namespace config;

int64_t timeDiffNanonSecs(timepoint_t start, timepoint_t end)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
}

/* Write "bitmap" to a PNG file specified by "path"; returns 0 on
   success, non-zero on error. */

Bitmap::Bitmap()
    : m_width{ 0 }
    , m_height{ 0 }
{
}

Bitmap::~Bitmap() = default;

Pixel& Bitmap::at(const uint32_t x, const uint32_t y)
{
    assert(m_pixels && x < m_width && y < m_height);
    return m_pixels[y * m_width + x];
}

const Pixel& Bitmap::at(const uint32_t x, const uint32_t y) const
{
    assert(m_pixels && x < m_width && y < m_height);
    return m_pixels[y * m_width + x];
}

bool Bitmap::toPngFile(const std::string & filePath) const
{
    FILE *fp;
    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    png_byte **row_pointers = NULL;
    bool success = false;
    /* The following number is set by trial and error only. I cannot
       see where it is documented in the libpng manual.
     */
    int pixel_size = 3;
    int depth = 8;

    fp = fopen(filePath.c_str(), "wb");
    if (!fp) {
        return false;
    }

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        goto png_create_write_struct_failed;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        goto png_create_info_struct_failed;
    }

    /* Set up error handling. */

    if (setjmp(png_jmpbuf(png_ptr))) {
        goto png_failure;
    }

    /* Set image attributes. */

    png_set_IHDR(png_ptr,
        info_ptr,
        m_width,
        m_height,
        depth,
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

      /* Initialize rows of PNG. */

    row_pointers = static_cast<png_byte**>(png_malloc(png_ptr, m_height * sizeof(png_byte *)));
    for (uint32_t y = 0; y < m_height; ++y) {
        png_byte *row = static_cast<png_byte*>(png_malloc(png_ptr,
            sizeof(png_byte) * m_width *
            pixel_size));
        row_pointers[y] = row;
        for (uint32_t x = 0; x < m_width; ++x) {
            const Pixel &pixel = at(x, y);
            *row++ = pixel.red;
            *row++ = pixel.green;
            *row++ = pixel.blue;
        }
    }

    /* Write the image data to "fp". */

    png_init_io(png_ptr, fp);
    png_set_rows(png_ptr, info_ptr, row_pointers);
    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

    /* The routine has successfully written the file */
    success = true;

    for (size_t y = 0; y < m_height; y++) {
        png_free(png_ptr, row_pointers[y]);
    }
    png_free(png_ptr, row_pointers);

png_failure:
png_create_info_struct_failed:
    png_destroy_write_struct(&png_ptr, &info_ptr);
png_create_write_struct_failed:
    fclose(fp);
    return success;
}

Node::Node()
    : Node(nullptr,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        -1)
{
}

Node::Node(Node *up,
    double sx, double sy, double sz,
    double ex, double ey, double ez,
    int depth)
    : start{ sx, sy, sz }
    , end{ ex, ey, ez }
    , depth{ depth }
    , UNE{ nullptr }
    , UNW{ nullptr }
    , USE{ nullptr }
    , USW{ nullptr }
    , DNE{ nullptr }
    , DNW{ nullptr }
    , DSE{ nullptr }
    , DSW{ nullptr }
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
    swap(lhs.m_bodies, rhs.m_bodies);
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
    null.m_bodies = std::move(m_bodies);
    swap(*this, null);
}

void Node::addBody(Body *body)
{
    m_bodies.push_back(body);
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
    const size_t reserveSize = m_nodes.reservedSize();
    m_nodes.resize(1);
    m_nodes.reserve(reserveSize);
    m_nodes.front().resetNeighborhood();
    m_nodes.front().depth = 0;
}

void Model::divideNode(Node *node)
{
    if (!node) {
        return;
    }
    if (node->bodies_quantity() == 1) {
        m_roots.push_back(node);
        return;
    }
    if (node->bodies_quantity() == 0) {
        return;
    }
    const double sx = node->start.x;
    const double sy = node->start.y;
    const double sz = node->start.z;
    const double ex = node->end.x;
    const double ey = node->end.y;
    const double ez = node->end.z;
    const double mx = (sx + ex) / 2.0;
    const double my = (sy + ey) / 2.0;
    const double mz = (sz + ez) / 2.0;
    for (Body *body : node->bodies()) {
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
                    if (node->UNW == NULL) {
                        m_nodes.emplace_back(node,
                            sx, sy, sz, mx, my, mz, node->depth + 1);
                        node->UNW = &m_nodes.back();
                    }
                    node->UNW->addBody(body);
                }
                else {    // z >= mz
                    if (node->DNW == NULL) {
                        m_nodes.emplace_back(node,
                            sx, sy, mz, mx, my, ez, node->depth + 1);
                        node->DNW = &m_nodes.back();
                    }
                    node->DNW->addBody(body);
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (node->USW == NULL) {
                        m_nodes.emplace_back(node,
                            sx, my, sz, mx, ey, mz, node->depth + 1);
                        node->USW = &m_nodes.back();
                    }
                    node->USW->addBody(body);
                }
                else {    // z >= mz
                    if (node->DSW == NULL) {
                        m_nodes.emplace_back(node,
                            sx, my, mz, mx, ey, ez, node->depth + 1);
                        node->DSW = &m_nodes.back();
                    }
                    node->DSW->addBody(body);
                }
            }
        }
        else {      // x >= mx
            if (body->position.y < my) {
                if (body->position.z < mz) {
                    if (node->UNE == NULL) {
                        m_nodes.emplace_back(node,
                            mx, sy, sz, ex, my, mz, node->depth + 1);
                        node->UNE = &m_nodes.back();
                    }
                    node->UNE->addBody(body);
                }
                else {    // z >= mz
                    if (node->DNE == NULL) {
                        m_nodes.emplace_back(node,
                            mx, sy, mz, ex, my, ez, node->depth + 1);
                        node->DNE = &m_nodes.back();
                    }
                    node->DNE->addBody(body);
                }
            }
            else {      // y >= my
                if (body->position.z < mz) {
                    if (node->USE == NULL) {
                        m_nodes.emplace_back(node,
                            mx, my, sz, ex, ey, mz, node->depth + 1);
                        node->USE = &m_nodes.back();
                    }
                    node->USE->addBody(body);
                }
                else {    // z >= mz
                    if (node->DSE == NULL) {
                        m_nodes.emplace_back(node,
                            mx, my, mz, ex, ey, ez, node->depth + 1);
                        node->DSE = &m_nodes.back();
                    }
                    node->DSE->addBody(body);
                }
            }
        }
    }
    divideNode(node->UNW);
    divideNode(node->UNE);
    divideNode(node->USW);
    divideNode(node->USE);
    divideNode(node->DNW);
    divideNode(node->DNE);
    divideNode(node->DSW);
    divideNode(node->DSE);
}

void Node::updateCenterOfMass()
{
    m_centerOfMass.position = { 0, 0, 0 };
    m_centerOfMass.mass = 0;
    if (bodies_quantity() >= 1) {
        for (size_t i = 0; i < bodies_quantity(); ++i) {
            const auto totalSpeed = m_bodies[i]->speed.length();
            const double relativisticAjust = 1 /
                std::sqrt(1 - (totalSpeed * totalSpeed) / (C * C));
            m_centerOfMass.position += m_bodies[i]->position
                * m_bodies[i]->mass * relativisticAjust;
            m_centerOfMass.mass += m_bodies[i]->mass * relativisticAjust;
        }
        m_centerOfMass.position /= m_centerOfMass.mass;
    }
}

void Body::applyForceFrom(const Body &other)
{
    const Vec3d xyzDist = other.position - position;
    const double DistanceSquared = xyzDist.lengthSq() + EPS2;
    const double newForce = K * other.mass * mass / DistanceSquared * 0.5;
    const double dist = std::sqrt(DistanceSquared);
    force += xyzDist / std::sqrt(DistanceSquared) * newForce;
}

bool Node::applyForceTo(Body &body) const
{
    Vec3d xyzDist = body.position - (end + start) * 0.5;
    const double distance = xyzDist.length();
    if ((this->end.x - this->start.x) / distance < ALPHA
        || this->bodies_quantity() == 1) {
        xyzDist = m_centerOfMass.position - body.position;
        const double DistanceSquared = xyzDist.lengthSq() + EPS2;
        const double force = K * m_centerOfMass.mass * body.mass / DistanceSquared;
        body.force += xyzDist / std::sqrt(DistanceSquared) * force;
        return true;
    }
    else {
        return false;
    }
}

namespace
{

void forceOverNode(Node *node, Node *down, Body &body, bool inverse)
{
    if (!node) {
        return;
    }
    if (node->UNE && (node->UNE != down || inverse)) {
        const bool canApproximate = node->UNE->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->UNE, node, body, true);
        }
    }
    if (node->UNW && (node->UNW != down || inverse)) {
        const bool canApproximate = node->UNW->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->UNW, node, body, true);
        }
    }
    if (node->USE && (node->USE != down || inverse)) {
        const bool canApproximate = node->USE->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->USE, node, body, true);
        }
    }
    if (node->USW && (node->USW != down || inverse)) {
        const bool canApproximate = node->USW->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->USW, node, body, true);
        }
    }
    if (node->DNE && (node->DNE != down || inverse)) {
        const bool canApproximate = node->DNE->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->DNE, node, body, true);
        }
    }
    if (node->DNW && (node->DNW != down || inverse)) {
        const bool canApproximate = node->DNW->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->DNW, node, body, true);
        }
    }
    if (node->DSE && (node->DSE != down || inverse)) {
        const bool canApproximate = node->DSE->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->DSE, node, body, true);
        }
    }
    if (node->DSW && (node->DSW != down || inverse)) {
        const bool canApproximate = node->DSW->applyForceTo(body);
        if (!canApproximate) {
            forceOverNode(node->DSW, node, body, true);
        }
    }
    if (!inverse)
        forceOverNode(node->UP, node, body, false);
    return;
}

}

void Model::init(const size_t numBodies, const bool writeToFile)
{
    m_frameCount = 0;
    if (writeToFile) {
        m_outputPositions_f = std::fopen("positionData.csv", "wb");
    }
    const size_t maxPossibleNumNodes = std::min(numBodies, size_t(MAX_NODES)) * 10;
    m_nodes.reserve(maxPossibleNumNodes);
    m_roots.reserve(numBodies);
    m_bodies.resize(numBodies);

    m_nodes.emplace_back(nullptr,
        -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30, -SIZE_OF_SIMULATION * 30,
        SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30, SIZE_OF_SIMULATION * 30,
        0);

    for (Body &body : m_bodies) {
        body.position.x =
            (std::rand() % 10000000 / 10000000.0) * 120E3 * LY - 60E3 * LY;
        body.position.y =
            (std::rand() % 10000000 / 10000000.0) * 120E3 * LY - 60E3 * LY;
        body.position.z =
            (std::rand() % 10000000 / 10000000.0) * 120E3 * LY - 60E3 * LY;
        body.force.x = 0;
        body.force.y = 0;
        body.force.z = 0;
        body.mass = double(TOTAL_MASS) / numBodies;
        body.speed.x = 0;
        body.speed.y = 0;
        body.speed.z = 0;
        m_nodes[0].addBody(&body);
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
            update();
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

void Model::update()
{
    const std::lock_guard<std::mutex> lock{ m_dataMutex };
    updateUnlocked();
}

void Model::updateUnlocked()
{
    const auto startTime = std::chrono::high_resolution_clock::now();

    if (m_frameLimit > 0 && m_frameCount == m_frameLimit) {
        m_endTime = std::chrono::high_resolution_clock::now();
        printf("Simulation time Elapsed = %f\n",
            timeDiffNanonSecs(m_endTime, m_startTime) / 1e9);
        exit(0);
    }
    m_roots.clear();
    resetNodes();
    divideNode(&m_nodes.front());
#pragma omp parallel for
    for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(m_nodes.size()); ++i) {
        m_nodes[i].updateCenterOfMass();
    }
  #pragma omp parallel for
    for (ptrdiff_t rootId = 0; rootId < static_cast<ptrdiff_t>(m_roots.size()); ++rootId) {
        Node * root = m_roots[rootId];
  #pragma omp parallel for
        for (ptrdiff_t bodyId = 0; bodyId < static_cast<ptrdiff_t>(root->bodies().size()); ++bodyId) {
            forceOverNode(root, NULL, *root->bodies()[bodyId], false);
        }
    }
    if (m_outputPositions_f) {
        fprintf(m_outputPositions_f, "FF\n");
    }
    for (size_t i = 0; i < m_bodies.size(); i++) {
        m_bodies[i].speed.x += m_bodies[i].force.x / m_bodies[i].mass;
        m_bodies[i].speed.y += m_bodies[i].force.y / m_bodies[i].mass;
        m_bodies[i].speed.z += m_bodies[i].force.z / m_bodies[i].mass;
        m_bodies[i].acel = std::sqrt(m_bodies[i].speed.x * m_bodies[i].speed.x +
            m_bodies[i].speed.y * m_bodies[i].speed.y +
            m_bodies[i].speed.z * m_bodies[i].speed.z) / colorMax;
        if (m_outputPositions_f) {
            fprintf(m_outputPositions_f, "%i,%i,%i,%i\n",
                (int)(m_bodies[i].position.x * 10E-16),
                (int)(m_bodies[i].position.y * 10E-16),
                (int)(m_bodies[i].position.z * 10E-16), (int)m_bodies[i].acel);
        }
        m_bodies[i].position.x += m_bodies[i].speed.x * 50E12;
        m_bodies[i].position.y += m_bodies[i].speed.y * 50E12;
        m_bodies[i].position.z += m_bodies[i].speed.z * 50E12;
        m_bodies[i].force.x = 0;
        m_bodies[i].force.y = 0;
        m_bodies[i].force.z = 0;
    }
    m_frameCount++;
    if (PRINT_TIMINGS)
    {
        const auto endTime = std::chrono::high_resolution_clock::now();
        std::cout << "Compute time: " << std::setw(11) << timeDiffNanonSecs(startTime, endTime) / 1000
            << "micro seconds, Frame: " << m_frameCount << std::endl;
    }
}

Model::LockedData Model::lockedData()
{
    return LockedData(m_bodies, m_nodes, m_dataMutex);
}

void Model::benchMode() {
    while (1) {
        updateUnlocked();
    }
}
