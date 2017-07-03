#pragma once

class Model;

class Renderer
{
public:
    static void setModel(Model * model);
    static void init();

    static void drawScene();

    static void handleResize(int w, int h);
    static void handleKeypress(unsigned char key, int x, int y);
    static void handleSpecial(int key, int x, int y);

private:
    static Renderer & instance();
    Renderer();
    ~Renderer();

    void handleKeypress_mem(int key, int x, int y);
    void drawScene_mem();

private:
    Model * m_model;
    bool paused;
    size_t nextFrameToRender;
    float angleX;
    float angleY;
    float positionZ;
};
