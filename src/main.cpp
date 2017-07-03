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
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

#include <GL/glut.h>

#include "core.h"
#include "render.h"
#include "config.h"

using namespace config;

struct Args
{
    Args(int argc, char **argv);
    size_t bodiesQuantity = BODIES_QUANTITY;
    size_t frameLimit = -1;
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
  std::srand(0);
  Model model;
  model.visualMode = args.visualMode;
  model.setFrameLimit(args.frameLimit);
  model.init(args.bodiesQuantity);
  Renderer::setModel(&model);
  if (args.visualMode) {
    // Bitmap image;
    // image.width = WIDTH;
    // image.height = HEIGHT;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA | GLUT_DEPTH);
    glutInitWindowSize(WIDTH, HEIGHT);

    glutCreateWindow("teste");
    Renderer::init();

    glutDisplayFunc(Renderer::drawScene);
    glutKeyboardFunc(Renderer::handleKeypress);
    glutSpecialFunc(Renderer::handleSpecial);
    glutReshapeFunc(Renderer::handleResize);

    model.run();

    glutMainLoop();

    model.stopAndWait();
  }
  else {
    model.benchMode();
  }
  return 0;
}
