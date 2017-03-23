#ifndef EIGENPHYSIKS_SPRING_GRID_HDR
#define EIGENPHYSIKS_SPRING_GRID_HDR

#include <cmath>
#include <vector>

#include "eigen_types.hpp"
#include "global_consts.hpp"
#include "world_consts.hpp"

namespace epiks {
	struct t_point_object {
	public:
		t_point_object(float m): m_raw_mass(m), m_inv_mass(1.0f / m) {
			set_pos({0.0f, 0.0f, 0.0f});
			set_vel({0.0f, 0.0f, 0.0f});
			reset_force();
		}

		const t_pos3f& get_pos() const { return m_pos; }
		const t_vec3f& get_vel() const { return m_vel; }
		const t_vec3f& get_force() const { return m_force; }

		float get_mass() const { return m_raw_mass; }

		void set_pos(const t_pos3f& pos) { m_pos = pos; }
		void set_vel(const t_vec3f& vel) { m_vel = vel; }

		void set_force(const t_vec3f& f) { m_force  = f; }
		void add_force(const t_vec3f& f) { m_force += f; }

		void reset_force() { set_force({0.0f, 0.0f, 0.0f}); }

		void apply_forces(float dt) {
			#if 0
			const t_vec3f a1 = (m_force * m_inv_mass) * dt;
			const t_vec3f x1 = ps.x0 + (ps.v1 * dt) + (a1 * dt * dt * 0.5f);
			const t_vec3f x2 = ps.x1 * 2.0f - ps.x0 + (a1 * dt * dt       );

			// Stormer-Verlet; derive velocity from positions x2 and x1
			ss.x0 = mix(ps.x0, ps.x1, ct != 0.0f);
			ss.x1 = mix(   x1,    x2, ct != 0.0f);
			ss.v1 = (ss.x1 - ps.x0) / (dt * 2.0f);

			#else

			m_vel += ((m_force * m_inv_mass) * dt);
			m_pos += (m_vel * dt);

			assert(!std::isnan(m_pos.x()) && !std::isnan(m_pos.y()) && !std::isnan(m_pos.z()));
			assert(!std::isnan(m_vel.x()) && !std::isnan(m_vel.y()) && !std::isnan(m_vel.z()));
			#endif
		}

	private:
		t_pos3f m_pos;
		t_vec3f m_vel;
		t_vec3f m_force;

		float m_raw_mass;
		float m_inv_mass;
	};



	struct t_spring_object {
	public:
		t_spring_object() {}
		t_spring_object(size_t lhs_obj_idx, size_t rhs_obj_idx) {
			m_lhs_obj_idx = lhs_obj_idx;
			m_rhs_obj_idx = rhs_obj_idx;
		}

		void solve_forces(std::vector<t_point_object>& objects, const t_spring_base_params& consts) {
			t_point_object& lhs_obj = objects[m_lhs_obj_idx];
			t_point_object& rhs_obj = objects[m_rhs_obj_idx];

			const t_vec3f spring_vector = lhs_obj.get_pos() - rhs_obj.get_pos();

			// calculate how much the spring has extended or contracted from its neutral length
			const float cur_length = spring_vector.norm();
			const float dif_length = cur_length - consts.rest_length;

			t_vec3f force = {0.0f, 0.0f, 0.0f};

			if (cur_length > 0.0f) {
				force = (spring_vector / cur_length);

				force *= dif_length;
				force *= (-consts.stiff_const);
			}

			force += (-(lhs_obj.get_vel() - rhs_obj.get_vel()) * consts.frict_const);

			lhs_obj.add_force( force);
			rhs_obj.add_force(-force);
		}

	private:
		size_t m_lhs_obj_idx; // index of mass at 'left' tip of spring
		size_t m_rhs_obj_idx; // index of mass at 'right' tip of spring
	};



	struct t_spring_grid {
	public:
		t_spring_grid(
			const t_spring_grid_params& spring_grid_params,
			const t_spring_base_params& spring_base_params,
			const t_world_params& world_params
		) {
			const t_spring_grid_params& gp = spring_grid_params;
			const t_spring_base_params& sp = spring_base_params;

			m_objects.resize(gp.num_links_x * gp.num_links_y, gp.link_mass);
			m_springs.reserve(gp.num_links_x * gp.num_links_y);

			m_spring_grid_params = gp;
			m_spring_base_params = sp;
			m_world_params = world_params;
		}

		void add_springs() {
			const t_spring_grid_params& gp = m_spring_grid_params;
			const t_spring_base_params& sp = m_spring_base_params;

			assert(!m_anchors.empty());
			assert(gp.num_links_x != 0 && gp.num_links_y != 0);

			// set initial object positions; neutral-length distances between masses
			for (size_t y = 0; y < gp.num_links_y; y++) {
				for (size_t x = 0; x < gp.num_links_x; x++) {
					m_objects[get_obj_idx(x, y)].set_pos({x * sp.rest_length, m_anchors[0].pos.y() - (y * sp.rest_length), 0.0f});
				}
			}

			// bind point-objects together with springs
			for (size_t y = 0; y < (gp.num_links_y - 1); y++) {
				for (size_t x = 0; x < (gp.num_links_x - 1); x++) {
					m_springs.emplace_back(get_obj_idx(x, y), get_obj_idx(x + 1, y    ));
					m_springs.emplace_back(get_obj_idx(x, y), get_obj_idx(x    , y + 1));
				}
			}

			// bottom-most row
			for (size_t x = 0; x < (gp.num_links_x - 1); x++) {
				m_springs.emplace_back(get_obj_idx(x, gp.num_links_y - 1), get_obj_idx(x + 1, gp.num_links_y - 1));
			}
			// right-most column
			for (size_t y = 0; y < (gp.num_links_y - 1); y++) {
				m_springs.emplace_back(get_obj_idx(gp.num_links_x - 1, y), get_obj_idx(gp.num_links_x - 1, y + 1));
			}
		}

		size_t get_num_objects() const { return (m_objects.size()); }
		size_t get_num_springs() const { return (m_springs.size()); }

		const t_point_object& get_object(size_t i) const { return m_objects[i]; }
		const t_spring_object& get_spring(size_t i) const { return m_springs[i]; }
		const t_spring_anchor& get_anchor(size_t i) const { return m_anchors[i]; }
		      t_spring_anchor& get_anchor(size_t i)       { return m_anchors[i]; }

		const t_spring_grid_params& get_grid_params() const { return m_spring_grid_params; }
			  t_spring_grid_params& get_grid_params()       { return m_spring_grid_params; }
		const t_spring_base_params& get_base_params() const { return m_spring_base_params; }
			  t_spring_base_params& get_base_params()       { return m_spring_base_params; }

		void add_anchor(const t_spring_anchor& anchor) { m_anchors.push_back(anchor); }
		void add_pulling_acc(const t_vec3f& acc) { m_spring_grid_params.pulling_acc += acc; }

		void update(float dt) {
			reset_forces();
			solve_forces();
			apply_forces(dt);
			update_anchors(dt);
		}

	private:
		void reset_forces() {
			for (t_point_object& o: m_objects) {
				o.reset_force();
			}
		}

		void solve_forces() {
			// add internal spring forces
			for (t_spring_object& s: m_springs) {
				s.solve_forces(m_objects, m_spring_base_params);
			}

			// add pulling force on tail objects
			for (size_t x = 0; x < m_spring_grid_params.num_links_x; x++) {
				const size_t y = m_spring_grid_params.num_links_y - 1;
				const size_t i = get_obj_idx(x, y);

				m_objects[i].add_force(m_spring_grid_params.pulling_acc * m_objects[i].get_mass());
			}

			// add common forces
			for (t_point_object& o: m_objects) {
				// o.add_force(m_spring_grid_params.pulling_acc * o.get_mass()); // F = m*a
				o.add_force(m_spring_grid_params.gravity_acc * o.get_mass()); // F = m*g
				o.add_force(-o.get_vel() * m_world_params.atmos_frict_coeff); // air friction

				const t_pos3f p = o.get_pos();
				const t_vec3f v = o.get_vel();

				if (p.y() >= m_world_params.ground_plane_level)
					continue;

				// apply ground-friction force
				o.add_force(-v.cwiseProduct(consts::WORLD_AXES[consts::AXIS_IDX_XZ]) * m_world_params.ground_frict_coeff);
				// absorb ground-collision energy
				o.add_force(-v.cwiseProduct(consts::WORLD_AXES[consts::AXIS_IDX_Y]) * m_world_params.ground_absor_coeff * (v.y() < 0.0f));
				// apply ground-repulsion force (damped spring)
				o.add_force({0.0f, m_world_params.ground_repul_coeff * (m_world_params.ground_plane_level - p.y()), 0.0f});
			}

			m_spring_grid_params.pulling_acc *= 0.0f;
		}

		void apply_forces(float dt) {
			for (t_point_object& o: m_objects) {
				o.apply_forces(dt);
			}
		}

		void update_anchors(float dt) {
			for (t_spring_anchor& anchor: m_anchors) {
				t_pos3f& pos = anchor.pos;
				t_vec3f& vel = anchor.vel;

				pos += (vel * dt);
				vel *= 0.85f;

				vel.y() *=         (pos.y() >= m_world_params.ground_plane_level);
				pos.y()  = std::max(pos.y(),   m_world_params.ground_plane_level);

				m_objects[anchor.obj_idx].set_pos(pos);
				m_objects[anchor.obj_idx].set_vel(vel);
			}
		}

		size_t get_obj_idx(size_t x, size_t y) const { return (y * m_spring_grid_params.num_links_x + x); }

	private:
		std::vector<t_point_object> m_objects;
		std::vector<t_spring_object> m_springs;
		std::vector<t_spring_anchor> m_anchors;

		t_spring_grid_params m_spring_grid_params;
		t_spring_base_params m_spring_base_params;
		t_world_params m_world_params;
	};
};

#endif

