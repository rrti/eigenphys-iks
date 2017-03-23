#include "physics_state.hpp"

void epiks::t_physics_state::init() {
	constexpr float r = consts::WORLD_PARAMS.ground_plane_scale * 0.5f;
	constexpr float a = (M_PI * 2.0f) / NUM_ARMS;

	for (size_t i = 0; i < NUM_ARMS; i++) {
		m_arms[i].add_piece(0.2f);
		m_arms[i].add_piece(0.4f);
		m_arms[i].add_piece(0.8f);
		m_arms[i].add_piece(0.6f);
		m_arms[i].add_piece(0.4f);
		m_arms[i].add_piece(0.3f);

		m_arms[i].set_base_pos({std::cos(i * a) * r, consts::WORLD_PARAMS.ground_plane_level + 0.1f, std::sin(i * a) * r});
		m_arms[i].set_goal_pos({0.0f, consts::WORLD_PARAMS.ground_plane_level + 10.0f, 0.0f}); // redundant
	}

	m_rope.add_anchor(consts::ROPE_ANCHOR);
	m_rope.add_springs();
}

void epiks::t_physics_state::step(float dt) {
	const epiks::t_spring_grid_params& gp = m_rope.get_grid_params();
	const epiks::t_point_object& po = m_rope.get_object((gp.num_links_x * gp.num_links_y) - 1); // tail

	for (size_t i = 0; i < NUM_ARMS; i++) {
		m_arms[i].solve(po.get_pos());
		m_rope.add_pulling_acc((m_arms[i].get_tail_pos() - po.get_pos()) * 5.0f);
	}

	m_rope.update(dt);
}

