#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/freeglut.h>

#include "eigen_engine.hpp"

static epiks::t_eigen_engine g_engine;

static void update_frame() { g_engine.update_frame(); }
static void render_frame() { g_engine.render_frame(); }

static void regular_key_pressed (uint8_t key, int32_t, int32_t) { g_engine.regular_key_pressed (key); }
static void regular_key_released(uint8_t key, int32_t, int32_t) { g_engine.regular_key_released(key); }
static void special_key_pressed (int32_t key, int32_t, int32_t) { g_engine.special_key_pressed (key); }
static void special_key_released(int32_t key, int32_t, int32_t) { g_engine.special_key_released(key); }

static void init_glut(int argc, char** argv) {
	glutInit(&argc, argv);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

	glutInitWindowSize(800, 800);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("eigenphys-iks");
	glewInit();


	glutDisplayFunc(render_frame);
	glutIdleFunc(update_frame);

	glutKeyboardFunc(regular_key_pressed);
	glutKeyboardUpFunc(regular_key_released);
	glutSpecialFunc(special_key_pressed);
	glutSpecialUpFunc(special_key_released);
}

int main(int argc, char** argv) {
	init_glut(argc, argv);
	g_engine.loop();
    return 0;
}

