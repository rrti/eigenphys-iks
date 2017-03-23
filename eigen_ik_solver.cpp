#include <cstdio>
#include <limits>

#include <GL/glew.h>
#include <GL/gl.h>

#include "eigen_ik_solver.hpp"

static constexpr size_t MAX_SOLVE_ITERS = 200;
static constexpr size_t MAX_ERROR_DECRS = 100;

static constexpr float MIN_ERROR_BOUND = 0.0050f;
static constexpr float ROT_DELTA_ANGLE = 0.0005f;

void epiks::t_rb_chain::solve(t_pos3f ws_goal_pos) {
	// return early if the goal-position did not change
	if ((ws_goal_pos - m_goal_pos).norm() < MIN_ERROR_BOUND)
		return;

	// translate world-space goal to object-space
	t_pos3f goal_pos = get_rel_goal_pos(ws_goal_pos);
	t_pos3f curr_pos = calc_tail_pos();

	t_matX1f delta_mat;

	// set initial error-bounds
	// float prev_err = std::numeric_limits<float>::max();
	float best_error = std::numeric_limits<float>::max();
	float iter_error = std::numeric_limits<float>::max();

	for (size_t num_solve_iters = 0; ((iter_error > MIN_ERROR_BOUND) && (num_solve_iters < MAX_SOLVE_ITERS)); num_solve_iters++) {
		save_iter_transforms();
		apply_transforms(delta_mat = calc_inv_jacobian(curr_pos) * (goal_pos - curr_pos));

		// revert transforms and bail out when error stops decreasing
		if (!decr_iter_error(goal_pos, curr_pos, delta_mat, iter_error, best_error)) {
			load_best_transforms();
			break;
		}

		// error decreased this iteration, save the transforms
		save_best_transforms();

		// prev_err = best_error;
		best_error = iter_error;
    }

	// remember final WS end-effector position; differs from goal if unreachable
	m_tail_pos = m_base_pos + curr_pos;
	m_goal_pos = ws_goal_pos;
}

bool epiks::t_rb_chain::decr_iter_error(const t_pos3f& goal_pos, t_pos3f& curr_pos, t_matX1f& delta_mat, float& iter_error, float& best_error) {
	// prev_err = iter_error;
	iter_error = (goal_pos - (curr_pos = calc_tail_pos())).norm();

	for (size_t num_error_decrs = 0; ((iter_error >= best_error) && (num_error_decrs < MAX_ERROR_DECRS)); num_error_decrs++) {
		// iterated past minimum, cut rotation-angles in half and re-apply them
		load_iter_transforms();
		apply_transforms(delta_mat *= 0.5f);

		// prev_err = iter_error;
		iter_error = (goal_pos - (curr_pos = calc_tail_pos())).norm();
	}

	return (iter_error < best_error);
}


t_matXYf epiks::t_rb_chain::calc_jacobian(const t_pos3f& chain_end_pos) {
	assert(chain_end_pos == calc_tail_pos());

	// create the Jacobian matrix; transposed for easier indexing
	t_matXYf chain_jac_mat(3 * m_pieces.size(), 3);
	t_mat33f piece_jac_mat;

	for (size_t i = 0; i < m_pieces.size(); i++) {
		#if 0
		const t_pos3f& piece_end_pos = m_pieces[i].get_tail_pos();
		const t_rot4f& piece_end_rot = m_pieces[i].get_transform();
		#endif

		piece_jac_mat = calc_piece_jacobian(m_pieces[i], chain_end_pos);

		chain_jac_mat(i * 3 + 0, 0) = piece_jac_mat(0, 0);
		chain_jac_mat(i * 3 + 0, 1) = piece_jac_mat(0, 1);
		chain_jac_mat(i * 3 + 0, 2) = piece_jac_mat(0, 2);

		chain_jac_mat(i * 3 + 1, 0) = piece_jac_mat(1, 0);
		chain_jac_mat(i * 3 + 1, 1) = piece_jac_mat(1, 1);
		chain_jac_mat(i * 3 + 1, 2) = piece_jac_mat(1, 2);

		chain_jac_mat(i * 3 + 2, 0) = piece_jac_mat(2, 0);
		chain_jac_mat(i * 3 + 2, 1) = piece_jac_mat(2, 1);
		chain_jac_mat(i * 3 + 2, 2) = piece_jac_mat(2, 2);
	}

	return (chain_jac_mat.transpose());
}

t_mat33f epiks::t_rb_chain::calc_piece_jacobian(t_rb_piece& piece, const t_pos3f& curr_end_pos) {
	t_mat33f piece_jac_mat;

	for (size_t axis_idx = consts::AXIS_IDX_X; axis_idx <= consts::AXIS_IDX_Z; axis_idx++) {
		// forward and inverse differential rotations
		const t_rot4f fwd_diff_rot = t_rot4f(ROT_DELTA_ANGLE, piece.get_axis(axis_idx));
		const t_rot4f inv_diff_rot = fwd_diff_rot.inverse();

		// find out the delta-transform's influence on the chain end-effector
		// (could also start from piece_end_rot to run in half-quadratic time)
		// TODO: per-axis angular constraints to emulate other types of joints, minimize SSE
		piece.apply_transform(fwd_diff_rot);

		const t_pos3f next_end_pos = calc_tail_pos();
		const t_vec3f diff_end_pos = (next_end_pos - curr_end_pos) / fwd_diff_rot.angle();

		// restore current unperturbed transform for this piece
		piece.apply_transform(inv_diff_rot);

		// set the per-axis partial derivatives <dx/dtheta, dy/dtheta, dz/dtheta>
		piece_jac_mat(axis_idx, 0) = diff_end_pos.x();
		piece_jac_mat(axis_idx, 1) = diff_end_pos.y();
		piece_jac_mat(axis_idx, 2) = diff_end_pos.z();
	}

	return piece_jac_mat;
}



t_pos3f epiks::t_rb_chain::calc_tail_pos(size_t min_piece_idx, size_t max_piece_idx) const {
	t_pos3f pos = {0.0f, 0.0f, 0.0f};

	if (max_piece_idx == size_t(-1))
		max_piece_idx = m_pieces.size() - 1;

	for (size_t i = min_piece_idx; i <= max_piece_idx; i++) {
		pos += m_pieces[i].get_tail_pos();
	}

	return pos;
}

