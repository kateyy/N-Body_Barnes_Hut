#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>

#include <GLFW/glfw3.h>

#include <glbinding/gl33core/bitfield.h>
#include <glbinding/gl33core/boolean.h>
#include <glbinding/gl33core/enum.h>
#include <glbinding/gl33core/functions.h>
#include <glbinding/gl33core/types.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <globjects/Buffer.h>
#include <globjects/Program.h>
#include <globjects/Shader.h>
#include <globjects/Uniform.h>
#include <globjects/VertexArray.h>
#include <globjects/VertexAttributeBinding.h>
#include <globjects/base/StaticStringSource.h>

#include "render.h"
#include "core.h"
#include "config.h"

#ifdef OPTION_WITH_PNG_EXPORT
#include "bitmap.h"
#endif

using namespace config;
using namespace gl33core;

namespace
{

constexpr double colorMax = 3E4;

constexpr GLuint primitiveRestartIdx = std::numeric_limits<GLuint>::max();

float Color_list[56][3] = { {204, 0, 0},
{102, 204, 0},
{0, 204, 204},
{102, 0, 204},

{117, 80, 123},
{122, 95, 80},
{85, 122, 80},
{80, 107, 122},

{52, 101, 164},
{163, 52, 156},
{163, 115, 52},
{52, 163, 60},

{115, 210, 22},
{21, 209, 209},
{115, 21, 209},
{209, 21, 21},

{193, 125, 17},
{17, 194, 38},
{17, 85, 194},
{194, 17, 173},

{245, 121, 0},
{0, 245, 0},
{0, 122, 245},
{254, 0, 245},

{237, 212, 0},
{0, 237, 95},
{0, 24, 237},
{237, 0, 142},

{239, 41, 41},
{140, 240, 41},
{41, 240, 240},
{140, 41, 240},

{173, 127, 168},
{173, 155, 127},
{127, 173, 132},
{127, 145, 173},

{114, 159, 207},
{207, 144, 205},
{207, 162, 114},
{114, 207, 115},

{138, 226, 52},
{52, 227, 227},
{140, 52, 227},
{227, 52, 52},

{233, 185, 110},
{109, 232, 123},
{109, 156, 232},
{232, 109, 218},

{252, 175, 62},
{63, 252, 82},
{63, 139, 252},
{252, 63, 234},

{252, 233, 79},
{78, 252, 145},
{78, 99, 252},
{252, 78, 186}
};

/* based on Delphi function by Witold J.Janik */
void
GiveRainbowColor(double position, float *c)
{
  /* if position > 1 then we have repetition of colors it maybe useful    */

  if (position > 1.0) {
    if (position - (int) position == 0.0)
      position = 1.0;
    else
      position = position - (int) position;
  }

  unsigned char nmax = 6;	/* number of color segments */
  double m = nmax * position;

  int n = (int) m;		// integer of m

  double t = position * 0.1;

  switch (n) {
  case 0:{
      c[0] = 255;
      c[1] = t;
      c[2] = 0;
      break;
    };
  case 1:{
      c[0] = 255 - t;
      c[1] = 255;
      c[2] = 0;
      break;
    };
  case 2:{
      c[0] = 0;
      c[1] = 255;
      c[2] = t;
      break;
    };
  case 3:{
      c[0] = 0;
      c[1] = 255 - t;
      c[2] = 255;
      break;
    };
  case 4:{
      c[0] = t;
      c[1] = 0;
      c[2] = 255;
      break;
    };
  case 5:{
      c[0] = 255;
      c[1] = 0;
      c[2] = 255 - t;
      break;
    };
  default:{
      c[0] = 255;
      c[1] = 0;
      c[2] = 0;
      break;
    };

  };				// case
}

struct RenderBody
{
    glm::vec3 position;
    glm::vec3 color;
};

}


Renderer& Renderer::instance()
{
    static Renderer ren;
    return ren;
}

void Renderer::setModel(Model * model)
{
    instance().m_model = model;
}

Renderer::Renderer()
    : m_window{ nullptr }
    , m_model{ nullptr }
    , m_paused{ false }
    , m_nextFrameToRender{ 0u }
    , m_angleX{ 0 }
    , m_angleY{ 0 }
    , m_positionZ{ -10 }
    , m_renderTree{ false }
    , m_enableBlending{ false }
{
}

Renderer::~Renderer() = default;

void Renderer::init(GLFWwindow *window)
{
    instance().m_window = window;
    glfwGetWindowSize(window, &instance().m_winSize.x, &instance().m_winSize.y);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glClearColor(0.1, 0.1, 0.3, 1.0);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPrimitiveRestartIndex(primitiveRestartIdx);

    instance().m_context = std::make_unique<Context>();
}

Renderer::Context::Context()
    : bodiesVBOElementCount{ 0 }
    , vertexShader{ GL_VERTEX_SHADER }
    , fragmentShader{ GL_FRAGMENT_SHADER }
    , nodesVBOElementCount{ 0 }
    , nodesIBOElementCount{ 0 }
{
    static const auto vertexShaderSource = globjects::Shader::sourceFromString(
        R"(#version 330 core 
layout (location = 0) in vec3 a_vertex;
layout (location = 1) in vec3 a_color;
uniform mat4 transform;
out vec3 color;
void main() {
    color = a_color;
    gl_Position = transform * vec4(a_vertex, 1.0);
}
)");
    vertexShader.setSource(vertexShaderSource.get());
    static const auto fragmentShaderSource = globjects::Shader::sourceFromString(
        R"(#version 330 core
layout (location = 0) out vec4 fragColor;
in vec3 color;
void main() {
    fragColor = vec4(color, 0.3);
}
)");
    fragmentShader.setSource(fragmentShaderSource.get());
    program.attach(&vertexShader, &fragmentShader);

    {
        auto vbinding = bodiesVAO.binding(0);
        vbinding->setAttribute(0);
        vbinding->setBuffer(&bodiesVBO, 0, sizeof(RenderBody));
        vbinding->setFormat(3, GL_FLOAT);
        bodiesVAO.enable(0);
        auto cbinding = bodiesVAO.binding(1);
        cbinding->setAttribute(1);
        cbinding->setBuffer(&bodiesVBO, offsetof(RenderBody, color), sizeof(RenderBody));
        cbinding->setFormat(3, GL_FLOAT);
        bodiesVAO.enable(1);
    }

    nodesVAO.bindElementBuffer(&nodesIBO);
    {
        auto vbinding = nodesVAO.binding(0);
        vbinding->setAttribute(0);
        vbinding->setBuffer(&nodesVBO, 0, sizeof(RenderBody));
        vbinding->setFormat(3, GL_FLOAT);
        nodesVAO.enable(0);
        auto cbinding = nodesVAO.binding(1);
        cbinding->setAttribute(1);
        cbinding->setBuffer(&nodesVBO, offsetof(RenderBody, color), sizeof(RenderBody));
        cbinding->setFormat(3, GL_FLOAT);
        nodesVAO.enable(1);
    }
}

Renderer::Context::~Context() = default;

void Renderer::handleResize(GLFWwindow * /*window*/, int w, int h)
{
    instance().m_winSize = { w, h };
    glViewport(0, 0, w, h);
}

void Renderer::handleKeypress(GLFWwindow *window, int key, int scancode, int action, int modes)
{
    instance().handleKeypress_mem(window, key, scancode, action, modes);
}

void Renderer::handleKeypress_mem(GLFWwindow * /*window*/, int key, int /*scancode*/, int action, int mods)
{
    if (action != GLFW_RELEASE) {
        return;
    }

    const bool ctrl_pressed = (mods & GLFW_MOD_CONTROL) != 0;

    auto changePointSize = [] (const bool increase) {
        float pointSize = 1.0f, granularity = 1.f;
        glGetFloatv(GL_POINT_SIZE, &pointSize);
        glGetFloatv(GL_POINT_SIZE_GRANULARITY, &granularity);
        pointSize = pointSize + (increase ? granularity : - granularity);
        std::cout << "Point size: " << pointSize << std::endl;
        glPointSize(pointSize);
    };

    switch (key) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(instance().m_window, true);
        break;
    case GLFW_KEY_LEFT:
        m_angleY += 2;
        break;
    case GLFW_KEY_RIGHT:
        m_angleY -= 2;
        break;
    case GLFW_KEY_UP:
        if (ctrl_pressed) {
            changePointSize(true);
        } else {
            m_angleX += 2;
        }
        break;
    case GLFW_KEY_DOWN:
        if (ctrl_pressed) {
            changePointSize(false);
        } else {
            m_angleX -= 2;
        }
        break;
    case GLFW_KEY_P:
    case GLFW_KEY_F12:
        m_paused = !m_paused;
        m_model->pause(m_paused);
        break;
    case GLFW_KEY_PAGE_UP:
        m_positionZ += 1;
        break;
    case GLFW_KEY_PAGE_DOWN:
        m_positionZ -= 1;
        break;
    case GLFW_KEY_B:
        m_enableBlending = !m_enableBlending;
        if (m_enableBlending) {
            glEnable(GL_BLEND);
        } else {
            glDisable(GL_BLEND);
        }
        break;
    case GLFW_KEY_T:
        m_renderTree = !m_renderTree;
        break;
    case GLFW_KEY_LEFT_CONTROL:
    case GLFW_KEY_RIGHT_CONTROL:
        break;
    default:
        std::cerr << "Unmapped key pressed: " << key << std::endl;
    }

    //angleX += 0.5;
    if (m_angleX > 360)
        m_angleX -= 360;
    if (m_angleY > 360)
        m_angleY -= 360;
    if (m_angleX < 0)
        m_angleX += 360;
    if (m_angleY < 0)
        m_angleY += 360;
}

bool Renderer::drawScene()
{
    return instance().drawScene_mem();
}

bool Renderer::drawScene_mem()
{
    if (!m_model) {
        return false;
    }

    // Skip rendering if the model has no progress yet.
    const auto modelFrame = m_model->frameCount();
    if (modelFrame < m_nextFrameToRender) {
        return false;
    }
    m_nextFrameToRender = modelFrame + 1;  

    auto view = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, m_positionZ));
    view = glm::rotate(view, glm::radians(m_angleX), glm::vec3(1.f, 0.f, 0.f));
    view = glm::rotate(view, glm::radians(m_angleY), glm::vec3(0.f, 1.f, 0.f));
    const auto projection = glm::perspective(45.f,  float(m_winSize.x) / m_winSize.y, 0.01f, 200.f);
    const auto transform = projection * view;

    constexpr double scale = 1.f / (50000.f * LY);

    std::vector<RenderBody> toGpuData;
    std::vector<RenderBody> toGpuNodeData;
    std::vector<GLuint> toGpuNodeIndexes;
    {
        const auto lockedData = m_model->lockedData();
        toGpuData.resize(lockedData.bodies.size());
        for (size_t i = 0; i < lockedData.bodies.size(); ++i) {
            toGpuData[i].position = lockedData.bodies[i].position * scale;
            const float acceleration = float(glm::length(lockedData.bodies[i].speed) / colorMax);
            GiveRainbowColor(acceleration, &toGpuData[i].color[0]);
        }
        if (m_renderTree) {
            GLuint idx = 0;
            for (size_t i = 0; i < lockedData.nodes.size(); ++i) {
                const Node &node = lockedData.nodes[i];
                if (node.bodies_quantity() <= 1) {
                    continue;
                }
                const glm::vec3 color(Color_list[i % 56][0] / 256.0, Color_list[i % 56][1] / 256.0,
                    Color_list[i % 56][2] / 256.0);
                const auto s = node.start * scale;
                const auto e = node.end * scale;
                toGpuNodeData.push_back({ s, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { s.x, e.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { e.x, e.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { e.x, s.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ s, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { s.x, s.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { s.x, e.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { s.x, e.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeIndexes.push_back(primitiveRestartIdx);

                toGpuNodeData.push_back({ { s.x, e.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ e, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { e.x, s.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { s.x, s.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeIndexes.push_back(primitiveRestartIdx);

                toGpuNodeData.push_back({ { e.x, s.y, e.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ { e.x, s.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeIndexes.push_back(primitiveRestartIdx);

                toGpuNodeData.push_back({ { e.x, e.y, s.z }, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeData.push_back({ e, color });
                toGpuNodeIndexes.push_back(idx++);
                toGpuNodeIndexes.push_back(primitiveRestartIdx);
            }
        }
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (toGpuData.empty()) {
        return true;
    }

    m_context->program.setUniform("transform", transform);
    m_context->program.use();

    if (m_context->bodiesVBOElementCount < toGpuData.size()) {
        m_context->bodiesVBO.setData(toGpuData, GL_STATIC_DRAW);
        m_context->bodiesVBOElementCount = toGpuData.size();
    }
    else {
        m_context->bodiesVBO.setSubData(toGpuData, 0);
    }

    m_context->bodiesVAO.drawArrays(GL_POINTS, 0, static_cast<GLsizei>(toGpuData.size()));

    //Tree rendering
    if (m_renderTree) {
        if (m_context->nodesVBOElementCount < toGpuNodeData.size()) {
            m_context->nodesVBO.setData(toGpuNodeData, GL_STATIC_DRAW);
            m_context->nodesVBOElementCount = toGpuNodeData.size();
        }
        else {
            m_context->nodesVBO.setSubData(toGpuNodeData, 0);
        }
        if (m_context->nodesIBOElementCount < toGpuNodeIndexes.size()) {
            m_context->nodesIBO.setData(toGpuNodeIndexes, GL_STATIC_DRAW);
            m_context->nodesIBOElementCount = toGpuNodeIndexes.size();
        }
        else {
            m_context->nodesIBO.setSubData(toGpuNodeIndexes, 0);
        }

        glEnable(GL_PRIMITIVE_RESTART);
        m_context->nodesVAO.drawElements(GL_LINE_STRIP, static_cast<GLsizei>(toGpuNodeIndexes.size()), GL_UNSIGNED_INT);
        glDisable(GL_PRIMITIVE_RESTART);
    }

#ifdef OPTION_WITH_PNG_EXPORT
    Bitmap image{ uint32_t(m_winSize.x), uint32_t(m_winSize.y) };
    glReadPixels(0, 0, m_winSize.x, m_winSize.y, GL_RGB, GL_UNSIGNED_BYTE,
        image.rgb8_data());
    image.toPngFile(std::to_string(m_model->frameCount()) + ".png");
#endif
    return true;
}
