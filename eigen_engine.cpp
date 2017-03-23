#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/freeglut.h>

#include "eigen_engine.hpp"

void epiks::t_eigen_engine::loop() {
	init();
	glutMainLoop();
	kill();
}


void epiks::t_eigen_engine::handle_input(float dt) {
	opengl::t_camera& c = m_render_state.get_camera();

	epiks::t_spring_grid& sg = m_physics_state.get_rope();
	epiks::t_spring_anchor& sa = sg.get_anchor(0);

	sa.vel += (c.get_z_vec() * dt * 0.5f * (m_input_state.regular_keys['i'] != 0));
	sa.vel -= (c.get_z_vec() * dt * 0.5f * (m_input_state.regular_keys['k'] != 0));
	sa.vel -= (c.get_x_vec() * dt * 0.5f * (m_input_state.regular_keys['j'] != 0));
	sa.vel += (c.get_x_vec() * dt * 0.5f * (m_input_state.regular_keys['l'] != 0));
	sa.vel += (c.get_y_vec() * dt * 0.5f * (m_input_state.regular_keys['u'] != 0));
	sa.vel -= (c.get_y_vec() * dt * 0.5f * (m_input_state.regular_keys['o'] != 0));


	c.set_pos(c.get_pos() + (c.get_z_vec() * dt * 0.01f * (m_input_state.regular_keys['w'] != 0))); // strafe fwd (-z=fwd)
	c.set_pos(c.get_pos() - (c.get_z_vec() * dt * 0.01f * (m_input_state.regular_keys['s'] != 0))); // strafe back
	c.set_pos(c.get_pos() - (c.get_x_vec() * dt * 0.01f * (m_input_state.regular_keys['a'] != 0))); // strafe left
	c.set_pos(c.get_pos() + (c.get_x_vec() * dt * 0.01f * (m_input_state.regular_keys['d'] != 0))); // strafe right
	c.set_pos(c.get_pos() + (c.get_y_vec() * dt * 0.01f * (m_input_state.regular_keys['q'] != 0))); // strafe up (+y=up)
	c.set_pos(c.get_pos() - (c.get_y_vec() * dt * 0.01f * (m_input_state.regular_keys['e'] != 0))); // strafe down

	c.set_tgt(c.get_pos() +  c.get_z_vec());
	c.set_tgt(c.get_tgt() - (c.get_y_vec() * dt * 0.0025f * (m_input_state.special_keys[GLUT_KEY_UP   ] != 0)));
	c.set_tgt(c.get_tgt() + (c.get_y_vec() * dt * 0.0025f * (m_input_state.special_keys[GLUT_KEY_DOWN ] != 0)));
	c.set_tgt(c.get_tgt() - (c.get_x_vec() * dt * 0.0025f * (m_input_state.special_keys[GLUT_KEY_LEFT ] != 0)));
	c.set_tgt(c.get_tgt() + (c.get_x_vec() * dt * 0.0025f * (m_input_state.special_keys[GLUT_KEY_RIGHT] != 0)));
}


void epiks::t_eigen_engine::update_frame() {
	while (m_wall_clock.get_render_time_ns() >= consts::SIM_STEP_TIME_NS) {
		// execute one physics-timestep (tick/update)
		m_update_timer.tick_time();
		m_physics_state.step(consts::SIM_STEP_TIME_NS * 0.001f * 0.001f * 0.001f);

		m_wall_clock.add_render_time_ns(-consts::SIM_STEP_TIME_NS);
		m_wall_clock.add_update_time_ns(m_update_timer.tock_time());
		m_wall_clock.add_system_time_ns(m_update_timer.tock_time());
	}

	m_wall_clock.add_update_call();
}

void epiks::t_eigen_engine::render_frame() {
	handle_input(m_render_timer.tock_time() * 0.001f * 0.001f);
	m_render_timer.tick_time();

	m_render_state.setup_camera();
	m_render_state.setup_lights(m_physics_state);
	m_render_state.render_scene(m_physics_state);
	m_render_state.swap_buffers();

	if (m_wall_clock.update(m_render_timer.tock_time())) {
		m_wall_clock.add_system_time_ns(-consts::WALL_SEC_TIME_NS);
		m_wall_clock.output_timings(stdout);
	}
}

