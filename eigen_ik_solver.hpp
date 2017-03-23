#ifndef EIGENPHYSIKS_SOLVER_HDR
#define EIGENPHYSIKS_SOLVER_HDR

#include <vector>

#include "eigen_types.hpp"
#include "eigen_math.hpp"
#include "global_consts.hpp"
#include "world_consts.hpp"

namespace epiks {
	class t_rb_piece {
	public:
		t_rb_piece(float length = 0.0f) {
			m_curr_trans = t_rot4f::Identity();
			m_iter_trans = t_rot4f::Identity();
			m_best_trans = t_rot4f::Identity();

			m_length = length;
		}

		// tail is base of next segment; same as returning base + zaxis * length
		// t_pos3f get_base_pos() const { return (transform_pos(t_pos3f(0.0f, 0.0f, m_length * 0.0f))); }
		t_pos3f get_tail_pos() const { return (transform_pos(t_pos3f(0.0f, 0.0f, m_length * 1.0f))); }
		t_pos3f transform_pos(const t_pos3f& pos) const { return (m_curr_trans * pos); }

		t_rot4f get_transform() const { return m_curr_trans; }
		t_rot4f chain_transform(const t_rot4f& trans) const { return (t_rot4f(trans * m_curr_trans)); }

		t_vec3f get_axis(size_t idx) const { return (m_curr_trans * consts::WORLD_AXES[idx]); }
		t_vec3f get_x_axis() const { return (get_axis(consts::AXIS_IDX_X)); }
		t_vec3f get_y_axis() const { return (get_axis(consts::AXIS_IDX_Y)); }
		t_vec3f get_z_axis() const { return (get_axis(consts::AXIS_IDX_Z)); }

		float get_length() const { return m_length; }
		float get_angle() const { return (m_curr_trans.angle()); }


		void save_iter_transform() { m_iter_trans = m_curr_trans; }
		void load_iter_transform() { m_curr_trans = m_iter_trans; }

		void save_best_transform() { m_best_trans = m_curr_trans; }
		void load_best_transform() { m_curr_trans = m_best_trans; }


		void apply_transform(float angle, const t_vec3f& axis) { apply_transform(t_rot4f(angle, axis)); }
		void apply_transform(const t_rot4f& trans) { m_curr_trans = chain_transform(trans); }
		void apply_transform(const t_vec3f& angles) {
			apply_transform(angles.x(), get_x_axis()); // pitch
			apply_transform(angles.y(), get_y_axis()); //   yaw
			apply_transform(angles.z(), get_z_axis()); //  roll
		}

	private:
		// note: rotations are NOT relative to the previous piece
		t_rot4f m_curr_trans;
		t_rot4f m_iter_trans;
		t_rot4f m_best_trans;

		float m_length;
	};


	class t_rb_chain {
	public:
		t_rb_chain() {
			set_base_pos({0.0f, 0.0f, 0.0f});
			set_goal_pos({0.0f, 0.0f, 0.0f});

			m_pieces.clear();
			m_pieces.reserve(8);
		}

		size_t get_num_pieces() const { return (m_pieces.size()); }

		const std::vector<t_rb_piece>& get_pieces() const { return m_pieces; }
		      std::vector<t_rb_piece>& get_pieces()       { return m_pieces; }

		const t_rb_piece& get_piece(size_t i) const { return m_pieces[i]; }
		      t_rb_piece& get_piece(size_t i)       { return m_pieces[i]; }

		// all in world-space
		t_pos3f get_base_pos() const { return m_base_pos; }
		t_pos3f get_goal_pos() const { return m_goal_pos; }
		t_pos3f get_tail_pos() const { return m_tail_pos; }

		void set_base_pos(const t_pos3f& ws_base_pos) { m_base_pos = ws_base_pos; }
		void set_goal_pos(const t_pos3f& ws_goal_pos) { m_goal_pos = ws_goal_pos; }

		void add_piece(float length) { m_pieces.emplace_back(length); }
		void pop_piece() { m_pieces.pop_back(); }

		void solve(t_pos3f ws_goal_pos);

	private:
		t_mat33f calc_piece_jacobian(t_rb_piece& piece, const t_pos3f& curr_end_pos);
		t_matXYf calc_jacobian(const t_pos3f& chain_end_pos);
		t_matXYf calc_inv_jacobian(const t_pos3f& chain_end_pos) { return (math::calc_pseudo_inverse(calc_jacobian(chain_end_pos))); }

		t_pos3f get_rel_goal_pos(const t_pos3f& goal_pos) { return (get_rel_goal_vec(goal_pos) * get_rel_goal_dist(goal_pos)); }
		t_pos3f get_rel_goal_vec(const t_pos3f& goal_pos) { return ((goal_pos - m_base_pos).normalized()); }

		// computes the end-effector position in object-space
		t_pos3f calc_tail_pos(size_t min_piece_idx = 0, size_t max_piece_idx = size_t(-1)) const;


		void load_best_transforms() { for (t_rb_piece& j: m_pieces) { j.load_best_transform(); } }
		void save_best_transforms() { for (t_rb_piece& j: m_pieces) { j.save_best_transform(); } }
		void load_iter_transforms() { for (t_rb_piece& j: m_pieces) { j.load_iter_transform(); } }
		void save_iter_transforms() { for (t_rb_piece& j: m_pieces) { j.save_iter_transform(); } }
		void apply_transforms(const t_matX1f& mat) {
			for (size_t i = 0; i < m_pieces.size(); i++) {
				m_pieces[i].apply_transform(t_vec3f(mat[i * 3 + 0], mat[i * 3 + 1], mat[i * 3 + 2]));
			}
		}


		bool decr_iter_error(const t_pos3f& goal_pos, t_pos3f& curr_pos, t_matX1f& delta_mat, float& iter_error, float& best_error);

		float get_rel_goal_dist(const t_pos3f& goal_pos) const { return (std::min((goal_pos - m_base_pos).norm(), get_max_length())); }
		float get_max_length() const {
			float len = 0.0f;
			for (const t_rb_piece& j: m_pieces) {
				len += j.get_length();
			}
			return len;
		}
		float get_sum_squared_angles() const {
			float sum = 0.0f;
			for (const t_rb_piece& j: m_pieces) {
				sum += (j.get_angle() * j.get_angle());
			}
			return sum;
		}

	private:
		// world-space {chain anchor,solved end-effector,previous goal} positions
		t_pos3f m_base_pos;
		t_pos3f m_goal_pos;
		t_pos3f m_tail_pos;

		// rigid-body segments making up the kinematic chain
		std::vector<t_rb_piece> m_pieces;
	};
};

#endif

