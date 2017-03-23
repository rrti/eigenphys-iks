#include <GL/glew.h>
#include <GL/gl.h>

#include "opengl_light.hpp"

void opengl::t_light::enable() const { glEnable(GL_LIGHT0 + m_id); }
void opengl::t_light::disable() const { glDisable(GL_LIGHT0 + m_id); }

void opengl::t_light::set_gl_state() const {
	glPushMatrix();

	// position is transformed by the active modelview matrix
	// this includes the camera transform when we are called;
	// lights are in eye-space by default
	// glLoadIdentity();

	glLightfv(GL_LIGHT0 + m_id, GL_POSITION, &m_position[0]);
	glLightfv(GL_LIGHT0 + m_id, GL_AMBIENT ,  m_ambi_clr);
	glLightfv(GL_LIGHT0 + m_id, GL_DIFFUSE ,  m_diff_clr);
	glLightfv(GL_LIGHT0 + m_id, GL_SPECULAR,  m_spec_clr);

	glPopMatrix();
}

