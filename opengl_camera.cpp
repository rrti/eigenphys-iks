#include <GL/freeglut.h>

#include "opengl_camera.hpp"

void opengl::t_camera::update() {
	// recalculate eye-space z-vector and lookat-target
	const t_vec3f v = get_tgt() - get_pos();

	const float lsq = v.dot(v);
	const float inv = 1.0f / std::sqrt(lsq);

	set_tgt(get_pos() + v * inv);
	set_vec(consts::AXIS_IDX_Z, v * inv);
	set_vec(consts::AXIS_IDX_X, cross_zy());
}


void opengl::t_camera::set_gl_proj_mat() const {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0f, 1.0f, 1.0f, 100.0f);
}

void opengl::t_camera::set_gl_view_mat() const {
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(m_pos.x(), m_pos.y(), m_pos.z(),  m_tgt.x(), m_tgt.y(), m_tgt.z(),  m_vec[consts::AXIS_IDX_Y].x(), m_vec[consts::AXIS_IDX_Y].y(), m_vec[consts::AXIS_IDX_Y].z());
}

