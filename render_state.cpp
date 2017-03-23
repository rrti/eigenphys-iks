#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/freeglut.h>

#include "render_state.hpp"
#include "physics_state.hpp"
#include "eigen_ik_solver.hpp"
#include "global_consts.hpp"

void opengl::t_render_state::init(const epiks::t_physics_state& ps) {
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glShadeModel(GL_SMOOTH);
		glClearColor(0.75f, 0.75f, 0.75f, 1.0f);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
	}

	{
		m_opengl_lights[0] = opengl::t_light(0);
		m_opengl_lights[1] = opengl::t_light(1);

		m_opengl_lights[0].set_diff_clr(0.8f, 0.8f,  0.8f, 1.0f);
		m_opengl_lights[0].set_spec_clr(0.4f, 0.4f,  0.4f, 1.0f);
		m_opengl_lights[0].set_ambi_clr(0.2f, 0.2f,  0.2f, 1.0f);
		m_opengl_lights[1].set_diff_clr(1.0f, 0.0f,  0.0f, 1.0f);
		m_opengl_lights[1].set_ambi_clr(0.1f, 0.1f,  0.1f, 1.0f);

		m_opengl_lights[0].enable();
		m_opengl_lights[1].enable();
	}

	{
		glGenBuffers(1, &m_quad_vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, m_quad_vbo_id);
		glBufferStorage(GL_ARRAY_BUFFER, num_quad_vbo_bytes, nullptr, quad_vbo_flag_bits);

		m_quad_vbo_ptr = reinterpret_cast<float*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, num_quad_vbo_bytes, quad_vbo_flag_bits));

		const t_pos3f v0 = {-consts::WORLD_PARAMS.ground_plane_scale, 0.0f, -consts::WORLD_PARAMS.ground_plane_scale}; // TL
		const t_pos3f v1 = { consts::WORLD_PARAMS.ground_plane_scale, 0.0f, -consts::WORLD_PARAMS.ground_plane_scale}; // TR
		const t_pos3f v2 = { consts::WORLD_PARAMS.ground_plane_scale, 0.0f,  consts::WORLD_PARAMS.ground_plane_scale}; // BR
		const t_pos3f v3 = {-consts::WORLD_PARAMS.ground_plane_scale, 0.0f,  consts::WORLD_PARAMS.ground_plane_scale}; // BL
		const t_vec3f n0 = consts::WORLD_AXES[consts::AXIS_IDX_Y];

		for (uint32_t j = 0; j < 3; j++) { m_quad_vbo_ptr[ 0 + j] = v3[j]; m_quad_vbo_ptr[ 3 + j] = n0[j]; }
		for (uint32_t j = 0; j < 3; j++) { m_quad_vbo_ptr[ 6 + j] = v2[j]; m_quad_vbo_ptr[ 9 + j] = n0[j]; }
		for (uint32_t j = 0; j < 3; j++) { m_quad_vbo_ptr[12 + j] = v1[j]; m_quad_vbo_ptr[15 + j] = n0[j]; }
		for (uint32_t j = 0; j < 3; j++) { m_quad_vbo_ptr[18 + j] = v0[j]; m_quad_vbo_ptr[21 + j] = n0[j]; }

		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	{
		constexpr float scale = 0.15f;
		constexpr float angle = (2.0f * M_PI) / num_cone_divs;

		assert(num_cone_vbo_bytes == (num_cone_elems * cone_vbo_elem_size));

		glGenBuffers(1, &m_cone_vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, m_cone_vbo_id);
		glBufferStorage(GL_ARRAY_BUFFER, num_cone_vbo_bytes, nullptr, cone_vbo_flag_bits);

		m_cone_vbo_ptr = reinterpret_cast<float*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, num_cone_vbo_bytes, cone_vbo_flag_bits));

		const t_pos3f v0 = {0.0f, 0.0f, 0.0f};
		const t_pos3f v1 =  consts::WORLD_AXES[consts::AXIS_IDX_Z];
		const t_vec3f n0 = -consts::WORLD_AXES[consts::AXIS_IDX_Z];

		for (uint32_t i = 0; i < num_cone_divs; i++) {
			// generate vertices in counter-clockwise order
			const t_pos3f v3 = {scale * std::cos((i    ) * angle), scale * std::sin((i    ) * angle), 0.0f};
			const t_pos3f v4 = {scale * std::cos((i + 1) * angle), scale * std::sin((i + 1) * angle), 0.0f};
			const t_vec3f n1 = (((v4 - v3).normalized()).cross(((v1 - v3).normalized()))).normalized();

			// cone side-face slice (three elements, 3 * (sizeof(t_pos3f) + sizeof(t_vec3f)) bytes, 3 * 6 floats)
			for (uint32_t j = 0; j < 3; j++) {
				m_cone_vbo_ptr[i * num_cone_div_floats + (0 + j)] = v3[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (3 + j)] = n1[j];

				m_cone_vbo_ptr[i * num_cone_div_floats + (6 + j)] = v4[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (9 + j)] = n1[j];

				m_cone_vbo_ptr[i * num_cone_div_floats + (12 + j)] = v1[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (15 + j)] = n1[j];
			}

			// cone base-cap slice (three elements, 3 * (sizeof(t_pos3f) + sizeof(t_vec3f)) bytes, 3 * 6 floats)
			for (uint32_t j = 0; j < 3; j++) {
				m_cone_vbo_ptr[i * num_cone_div_floats + (18 + j)] = v0[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (21 + j)] = n0[j];

				m_cone_vbo_ptr[i * num_cone_div_floats + (24 + j)] = v4[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (27 + j)] = n0[j];

				m_cone_vbo_ptr[i * num_cone_div_floats + (30 + j)] = v3[j];
				m_cone_vbo_ptr[i * num_cone_div_floats + (33 + j)] = n0[j];
			}
		}

		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	{
		const epiks::t_spring_grid& rope = ps.get_rope();

		glGenBuffers(1, &m_rope_vbo_id);
		glBindBuffer(GL_ARRAY_BUFFER, m_rope_vbo_id);
		glBufferStorage(GL_ARRAY_BUFFER, rope.get_num_springs() * 2 * rope_vbo_elem_size, nullptr, rope_vbo_flag_bits);

		m_rope_vbo_ptr = reinterpret_cast<float*>(glMapBufferRange(GL_ARRAY_BUFFER, 0, rope.get_num_springs() * 2 * rope_vbo_elem_size, rope_vbo_flag_bits));

		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	m_quadric_obj = gluNewQuadric();
}

void opengl::t_render_state::kill() {
	m_opengl_lights[0].disable();
	m_opengl_lights[1].disable();

	{
		glDeleteBuffers(1, &m_cone_vbo_id);
		glDeleteBuffers(1, &m_quad_vbo_id);

		glBindBuffer(GL_ARRAY_BUFFER, m_rope_vbo_id);
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glDeleteBuffers(1, &m_rope_vbo_id);
	}

	gluDeleteQuadric(m_quadric_obj);
}


void opengl::t_render_state::setup_camera() {
	m_opengl_camera.clamp_pos({-100.0f, consts::WORLD_PARAMS.ground_plane_level, -100.0f}, {100.0f, consts::WORLD_PARAMS.ground_plane_level + 100.0f, 100.0f});
	m_opengl_camera.update();
	m_opengl_camera.set_gl_proj_mat();
	m_opengl_camera.set_gl_view_mat();
}

void opengl::t_render_state::setup_lights(const epiks::t_physics_state& ps) {
	const epiks::t_spring_grid& rope = ps.get_rope();
	const epiks::t_point_object& tail = rope.get_object(rope.get_num_springs());

	const t_pos3f& tp = tail.get_pos();

	m_opengl_lights[0].set_position(0.0f, consts::WORLD_PARAMS.ground_plane_level + 1.0f, 0.0f, 1.0f);
	m_opengl_lights[0].set_gl_state();

	m_opengl_lights[1].set_position(tp.x(), tp.y(), tp.z(), 1.0f);
	m_opengl_lights[1].set_gl_state();
}

void opengl::t_render_state::swap_buffers() const {
	glutSwapBuffers();
	glutPostRedisplay();
}


void opengl::t_render_state::render_scene(const epiks::t_physics_state& ps) const {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	{
		glBindBuffer(GL_ARRAY_BUFFER, m_quad_vbo_id);
		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);

		glVertexPointer(3, GL_FLOAT, quad_vbo_elem_size, quad_vbo_vertex_offset);
		glNormalPointer(   GL_FLOAT, quad_vbo_elem_size, quad_vbo_normal_offset);

		glDrawArrays(GL_QUADS, 0, num_quad_elems);

		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	for (size_t i = 0; i < epiks::t_physics_state::NUM_ARMS; i++) {
		const epiks::t_rb_chain& arm = ps.get_arm(i);

		t_pos3f goal_pos = arm.get_goal_pos();
		t_pos3f base_pos = arm.get_base_pos();

		// same handedness as OpenGL
		t_mat44f base_mat;

		{
			// goal-sphere
			glPushMatrix();
			glTranslatef(goal_pos.x(), goal_pos.y(), goal_pos.z());
			gluQuadricDrawStyle(m_quadric_obj, GLU_FILL);
			gluSphere(m_quadric_obj, 0.1f, 20, 20);
			glPopMatrix();
		}


		glPushMatrix();
		glBindBuffer(GL_ARRAY_BUFFER, m_cone_vbo_id);

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_NORMAL_ARRAY);

		glVertexPointer(3, GL_FLOAT, cone_vbo_elem_size, cone_vbo_vertex_offset);
		glNormalPointer(   GL_FLOAT, cone_vbo_elem_size, cone_vbo_normal_offset);


		#if 0
		// hack to use glMultiDrawArrays or glDrawArraysInstanced in FFP
		static   float matrices[16 * 15 * 2 * 3] = {0.0f};
		static   float  weights[16 * 15 * 2 * 3] = {0.0f};
		static uint8_t  indices[     15 * 2 * 3] = {0   };

		glEnableClientState(GL_MATRIX_INDEX_ARRAY);
		glEnableClientState(WEIGHT_ARRAY);
		glMatrixIndexPointer(1, GL_UNSIGNED_BYTE, 0, &indices[0]); // per-vertex
		glWeightPointer(1, GL_FLOAT, 0, &weights[0]);
		glMatrixMode(GL_MATRIX_PALETTE);

		for (size_t k = 0; k < arm.get_num_pieces(); k++) {
			const epiks::t_rb_piece& j = arm.get_piece(k);

			base_mat = math::compose_transform_matrix(base_pos, j.get_transform());
			base_pos = base_pos + j.get_tail_pos();

			glCurrentPaletteMatrix(k);
			glLoadMatrixf(base_mat.data());
		}

		glMatrixMode(GL_MODELVIEW);
		#endif


		// fixed support-piece
		glPushMatrix();
		glTranslatef(base_pos.x(), consts::WORLD_PARAMS.ground_plane_level, base_pos.z());
		glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
		glScalef(0.5f, 0.5f, 0.1f);
		glDrawArrays(GL_TRIANGLES, 0, num_cone_divs * num_cone_div_elems);
		glPopMatrix();

		for (const epiks::t_rb_piece& j: arm.get_pieces()) {
			base_mat = math::compose_transform_matrix(base_pos, j.get_transform());
			base_pos = base_pos + j.get_tail_pos();

			glPushMatrix();
			glMultMatrixf(base_mat.data());
			glScalef(1.0f, 1.0f, j.get_length());
			glDrawArrays(GL_TRIANGLES, 0, num_cone_divs * num_cone_div_elems);
			glPopMatrix();
		}

		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glPopMatrix();
	}

	{
		glBindBuffer(GL_ARRAY_BUFFER, m_rope_vbo_id);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, rope_vbo_elem_size, rope_vbo_vertex_offset);

		const epiks::t_spring_grid& rope = ps.get_rope();

		for (size_t idx = 0; idx < rope.get_num_springs(); ++idx) {
			const epiks::t_point_object& lhs_obj = rope.get_object(idx    );
			const epiks::t_point_object& rhs_obj = rope.get_object(idx + 1);

			const t_vec3f& lhs_pos = lhs_obj.get_pos();
			const t_vec3f& rhs_pos = rhs_obj.get_pos();

			m_rope_vbo_ptr[idx * 6 + 0] = lhs_pos.x();
			m_rope_vbo_ptr[idx * 6 + 1] = lhs_pos.y();
			m_rope_vbo_ptr[idx * 6 + 2] = lhs_pos.z();

			m_rope_vbo_ptr[idx * 6 + 3] = rhs_pos.x();
			m_rope_vbo_ptr[idx * 6 + 4] = rhs_pos.y();
			m_rope_vbo_ptr[idx * 6 + 5] = rhs_pos.z();
		}

		glLineWidth(4.0f);
		glDrawArrays(GL_LINES, 0, rope.get_num_springs() * 2);
		glDisableClientState(GL_VERTEX_ARRAY);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glLineWidth(1.0f);
	}
}

