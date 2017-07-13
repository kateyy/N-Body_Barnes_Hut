//Barnes-hut implementation - O(n log n) nbody algorithm implementation

//Copyright 2012 Mateus Zitelli <zitellimateus@gmail.com>

//This program is free software; you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation; either version 2 of the License, or
//(at your option) any later version.

//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.

//You should have received a copy of the GNU General Public License
//along with this program; if not, write to the Free Software
//Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//MA 02110-1301, USA.

#include <cstdio>
#include <cstring>
#include <limits>
#include <iostream>
#include <string>

#include "config.h"
#include "core.h"

#ifdef OPTION_WITH_RENDERING
#include <GLFW/glfw3.h>
#include <globjects/globjects.h>
#include "render.h"
#endif


using namespace config;

struct Args
{
    Args(int argc, char **argv);
    size_t bodiesQuantity = BODIES_QUANTITY;
    size_t frameLimit = std::numeric_limits<size_t>::max();
    bool visualMode = true;
};

Args::Args(int argc, char **argv)
{
    int ch = 0;

    for (int n = 1; n < argc; n++) {	/* Scan through args. */
        switch ((int)argv[n][0]) {	/* Check for option character. */
        case '-':
        case '/':
        {
            const size_t len = strlen(argv[n]);
            for (size_t m = 1; m < len; ++m) {	/* Scan through options. */
                ch = (int)argv[n][m];
                switch (ch) {
                case 'n':		/* Legal options. */
                case 'N':
                case 'f':
                case 'F':
                case 'b':
                case 'B':
                    break;
                default:
                    printf("Illegal option code = %c\n", ch);
                    exit(1);
                    break;
                }
            }
            break;
        }
        default:
            if (ch == 'n' || ch == 'N') {
                bodiesQuantity = std::stoull(argv[n]);
            }
            else if (ch == 'f' || ch == 'F') {
                frameLimit = std::stoull(argv[n]);
            }
            else if (ch == 'b' || ch == 'B') {
                visualMode = std::atoi(argv[n]) != 0;
            }
            break;
        }
    }
}

int main(int argc, char **argv)
{
    const Args args(argc, argv);

    Model model;
    model.visualMode = args.visualMode;
    model.setFrameLimit(args.frameLimit);
    model.init(args.bodiesQuantity);

#ifdef OPTION_WITH_RENDERING
    Renderer::setModel(&model);
    if (args.visualMode) {
        if (!glfwInit()) {
            std::cerr << "[GLFW] Initialization failed. No OpenGL devices available?" << std::endl;
            return 1;
        }
        struct CallTerminate
        {
            ~CallTerminate() { glfwTerminate(); }
        } callTerminate;
        glfwSetErrorCallback([] (int errnum, const char * errmsg) {
            std::cerr << "[GLFW] error " << errnum << " - \"" << errmsg << "\"" << std::endl;
        });
        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_DOUBLEBUFFER, true);
        struct GLFWwindow_t
        {
            GLFWwindow_t(int width, int height, const char * const title)
                : window{ glfwCreateWindow(width, height, title, nullptr, nullptr) } {}
            ~GLFWwindow_t() {
                if (window) {
                    glfwDestroyWindow(window);
                }
            }
            operator GLFWwindow*() { return window; }
        private:
            GLFWwindow *window;
        };
        GLFWwindow_t window{ WIDTH, HEIGHT, "N-Body: Barnes Hut" };
        if (!window) {
            std::cerr << "Context creation failed. Terminate execution." << std::endl;
            return 2;
        }

        glfwSetKeyCallback(window, Renderer::handleKeypress);
        glfwSetFramebufferSizeCallback(window, Renderer::handleResize);
        glfwMakeContextCurrent(window);
        globjects::init();
        Renderer::init(window);

        model.run();

        while (!glfwWindowShouldClose(window))
        {
            glfwPollEvents();
            if (Renderer::drawScene()) {
                glfwSwapBuffers(window);
            }
        }

        model.stopAndWait();
    } else
#endif
    {
        model.benchMode();
    }
    return 0;
}
