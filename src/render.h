#pragma once

#include <memory>

#include <glm/vec2.hpp>

struct GLFWwindow;
namespace globjects
{
class Buffer;
class Program;
class Shader;
class VertexArray;
}
class Model;

class Renderer
{
public:
    static void setModel(Model *model);
    static void init(GLFWwindow *window);

    static bool drawScene();

    static void handleResize(GLFWwindow *window, int width, int height);
    static void handleKeypress(GLFWwindow*, int, int, int, int);

private:
    static Renderer & instance();
    Renderer();
    ~Renderer();

    void handleKeypress_mem(GLFWwindow*, int, int, int, int);
    bool drawScene_mem();

private:
    GLFWwindow *m_window;
    Model * m_model;
    bool m_paused;
    size_t m_nextFrameToRender;
    glm::ivec2 m_winSize;
    float m_angleX;
    float m_angleY;
    float m_positionZ;
    bool m_renderTree;
    bool m_enableBlending;

    struct Context
    {
        Context();
        ~Context();

        globjects::Buffer bodiesVBO;
        size_t bodiesVBOElementCount;
        globjects::VertexArray bodiesVAO;
        globjects::Shader vertexShader;
        globjects::Shader fragmentShader;
        globjects::Program program;

        globjects::Buffer nodesVBO;
        size_t nodesVBOElementCount;
        globjects::Buffer nodesIBO;
        size_t nodesIBOElementCount;
        globjects::VertexArray nodesVAO;
    };
    std::unique_ptr<Context> m_context;
};
