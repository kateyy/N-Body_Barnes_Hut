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
    size_t iterations = 0;
    bool visualMode = true;
    std::string bodiesInitScheme;
    std::string inputFileName;
    std::string outputFileName;
    bool generatorMode = false;

    void printHelp() {
        std::cerr << "Usage:" << std::endl
            << "[-h|-H] - Print this help." << std::endl
            << "[-i|-I] XY - Stop after XY iterations. Default: Don't stop." << std::endl
            << "[-b|-B] ABC - Initialization scheme for the bodies." << std::endl
            << "[-n|-N] XY - Create XY bodies. Default: " << BODIES_QUANTITY << std::endl
            << "[-f|-F] inputFileName.xyzmabc - Import bodies instead of generating them." << std::endl
            << "        Line format: position.x y z mass speed.x y z" << std::endl
            << "[-o|-o] outputFileName.xyzmabc - Export bodies after each iteration." << std::endl
            << "[-g|-G] - Generator mode: generate bodies, export to outputFileName and exit." << std::endl
            << "[-p|-P] - Print sizeof(Body)-sizeof(std::mutex), sizeof(std::mutex), and exit." << std::endl
#ifdef OPTION_WITH_RENDERING
            << "[-v|-V] 0|1 - Enable visual mode. Default: Enabled." << std::endl;
#else
            << "[-v|-V] 0|1 - Enable visual mode. NOT AVAILBLE IN THE CURRENT BUILD." << std::endl;
#endif
    }
};

Args::Args(int argc, char **argv)
{
    for (int n = 1; n < argc; ++n) {   /* Scan through args. */
        if (argv[n][0] != '-' && argv[n][0] != '/') {
            std::cerr << "Options must begin with - or /" << std::endl;
            exit(1);
        }
        const char option = argv[n][1] == '\0' ? ' ' : argv[n][1];
        const char * value = argc > n + 1
                ? argv[n+1]
                : (const char*)nullptr;

        switch (option) {
        case 'h':
        case 'H':
            printHelp();
            exit(0);
        case 'g':
        case 'G':
            generatorMode = true;
            continue;
        case 'b':
        case 'B':
            bodiesInitScheme = value ? value : "(unspecified)";
            if (value) {
                ++n;
            }
            continue;
        case 'p':
        case 'P': {
                const size_t size_bytes = sizeof(Body)-sizeof(std::mutex);
                double size = size_bytes;
                int unit = 0;
                while (size > 1024.0) {
                    size /= 1024.0;
                    ++unit;
                }
                const std::vector<std::string> units { "bytes", "KB", "MB", "GB", "TB" };
                std::cout << "sizeof(Body)-sizeof(std::mutex): " << sizeof(Body)-sizeof(std::mutex) << " ";
                if (unit == 0) {
                    std::cout << units[0] << std::endl;
                } else {
                    std::cout << units[0] << " (" << size_t(size) << " " << units[unit] << ")" << std::endl;
                }
                std::cout << "sizeof(std::mutex):              " << sizeof(std::mutex) << std::endl;
                std::cout << "sizeof(Body):                    " << sizeof(Body) << std::endl;
                exit(0);
            }
        }

        if (!value) {
            std::cerr << "No value given for option " << option << std::endl;
            printHelp();
            exit(1);
        }
        switch (option) {
        case 'n':
        case 'N':
            bodiesQuantity = std::stoull(value);
            break;
        case 'f':
        case 'F':
            inputFileName = value;
            break;
        case 'o':
        case 'O':
            outputFileName = value;
            break;
        case 'i':
        case 'I':
            iterations = std::stoull(value);
            break;
        case 'v':
        case 'V':
            visualMode = std::atoi(value) != 0;
            break;
        default:
            std::cerr << "Invalid option: " << option << std::endl;
            printHelp();
            exit(1);
        }
        ++n;
    }
}

int main(int argc, char **argv)
{
    const Args args(argc, argv);

    Model model;
    model.visualMode = args.visualMode;
    model.setFrameLimit(args.iterations);
    model.inputFileName = args.inputFileName;
    model.outputFileName = args.outputFileName;
    if (!model.init(args.bodiesInitScheme, args.bodiesQuantity)) {
        return 2;
    }
    if (args.generatorMode) {
        return model.exportBodies() ? 0 : 3;
    }

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
        std::cout
            // << model.totalRuntimeSeconds() << ";"
            << model.totalRuntimeSeconds() - model.runtimeFirstNFrames() << std::endl;
    }
    return 0;
}
