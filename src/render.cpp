#include <iomanip>
#include <iostream>
#include <GL/glut.h>
#include "render.h"
#include "core.h"
#include "config.h"

using namespace config;

namespace
{

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
  : m_model{ nullptr }
  , paused{ false }
  , nextFrameToRender{ 0u }
  , angleX{ 0 }
  , angleY{ 0 }
  , positionZ{ -10 }
{
}

Renderer::~Renderer() = default;

void Renderer::init()
{
  glEnable (GL_DEPTH_TEST);
  glEnable (GL_NORMALIZE);
  glEnable (GL_BLEND);
  glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Renderer::handleResize(int w, int h)
{
  glViewport (0, 0, w, h);
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective (45.0, (float) w / (float) h, 1.0, 200.0);
}


void Renderer::handleKeypress(unsigned char key, int x, int y)
{
  handleSpecial(key, x, y);
}

void Renderer::handleSpecial(int key, int x, int y)
{
  instance().handleKeypress_mem(key, x, y);
}

void Renderer::handleKeypress_mem(int key, int x, int y)
{
  switch (key) {
  case 27:			//Escape key
    exit (0);
    break;
  case GLUT_KEY_RIGHT:
    angleX += 2;
    break;
  case GLUT_KEY_LEFT:
    angleX -= 2;
    break;
  case GLUT_KEY_UP:
    angleY += 2;
    break;
  case GLUT_KEY_DOWN:
    angleY -= 2;
    break;
  case GLUT_KEY_F12:
    paused = !paused;
    m_model->pause(paused);
    break;
  case 43:
    positionZ -= 1;
    break;
  case 95:
    positionZ += 1;
    break;
  }

  //angleX += 0.5;
  if (angleX > 360)
    angleX -= 360;
  if (angleY > 360)
    angleY -= 360;
  if (angleX < 0)
    angleX += 360;
  if (angleY < 0)
    angleY += 360;

  printf ("%i\n", key);
  glutPostRedisplay ();
}

void Renderer::drawScene()
{
  instance().drawScene_mem();
}

void Renderer::drawScene_mem()
{
  if (!m_model) {
    return;
  }

  // Skip rendering if the model has no progress yet.
  const auto modelFrame = m_model->frameCount();
  if (modelFrame < nextFrameToRender) {
    printf("ren: skipping\n");
    return;
  }
  nextFrameToRender = modelFrame + 1;

  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode (GL_MODELVIEW);
  glLoadIdentity ();

  glTranslatef (0.0f, 0.0f, positionZ);

  glRotatef (angleX, 0.0f, 1.0f, 0.0f);
  glRotatef (angleY, 1.0f, 1.0f, 0.0f);
  glBegin (GL_POINTS);

  auto lockedData = m_model->lockedData();

  const auto startTime = std::chrono::high_resolution_clock::now();


  for (const Body &body : lockedData.bodies) {
    float color[3];
    GiveRainbowColor (body.acel, color);
    glColor4f (color[0], color[1], color[2], 0.5f);
    glVertex3f(body.position.x / (50000 * LY),
      body.position.y / (50000 * LY),
      body.position.z / (50000 * LY));
  }
  glEnd ();
  //Tree renderization
#if 1
  for (size_t i = 0; i < lockedData.nodes.size(); ++i) {
    const Node &node = lockedData.nodes[i];
    glColor4f (Color_list[i % 56][0] / 256.0, Color_list[i % 56][1] / 256.0,
	       Color_list[i % 56][2] / 256.0, 1);
    glBegin (GL_LINE_STRIP);
    glVertex3f (node.start.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.end.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY), node.end.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.end.y / (50000 * LY), node.end.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.end.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glEnd ();
    glBegin (GL_LINE_STRIP);
    glVertex3f (node.start.x / (50000 * LY),
		node.end.y / (50000 * LY), node.end.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY), node.end.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glVertex3f (node.start.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glEnd ();
    glBegin (GL_LINE_STRIP);
    glVertex3f (node.end.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY),
		node.start.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glEnd ();
    glBegin (GL_LINE_STRIP);
    glVertex3f (node.end.x / (50000 * LY), node.end.y / (50000 * LY),
		node.start.z / (50000 * LY));
    glVertex3f (node.end.x / (50000 * LY), node.end.y / (50000 * LY),
		node.end.z / (50000 * LY));
    glEnd ();
  }
#endif
  glutSwapBuffers();
  glFlush();
  glutPostRedisplay();
#if 0
  glReadPixels (0, 0, HEIGHT, WIDTH, GL_RGB, GL_UNSIGNED_BYTE, pRGB);
  free (image.pixels);
  image.pixels = malloc (sizeof (pixel_t) * WIDTH * HEIGHT);
  for (i = 0; i < WIDTH * HEIGHT; i++) {
    image.pixels[i].red = pRGB[i * 3];
    image.pixels[i].green = pRGB[i * 3 + 1];
    image.pixels[i].blue = pRGB[i * 3 + 2];
  }
  sprintf (buf, "%i.png", frame++);
  save_png_to_file (&image, buf);
#endif

  const auto endTime = std::chrono::high_resolution_clock::now();
  std::cout << "Render time:  " << std::setw(11) << timeDiffNanonSecs(startTime, endTime) / 1000
      << "micro seconds" << std::endl;
}
