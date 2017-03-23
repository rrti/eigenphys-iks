#ifndef EIGENPHYSIKS_MATH_HDR
#define EIGENPHYSIKS_MATH_HDR

#include "eigen_types.hpp"

namespace math {
	template<typename t_matrix>
	static t_matrix calc_pseudo_inverse(const t_matrix& a, double epsilon = std::numeric_limits<double>::epsilon()) {
		typedef Eigen::JacobiSVD<t_matrix> t_svd_matrix;
		typedef Eigen::Matrix<float, -1, 1> t_sin_val_matrix;
		typedef Eigen::ArrayWrapper<const t_sin_val_matrix> t_sin_val_array;

		typedef Eigen::internal::scalar_abs_op<float> t_scalar_abs_op;
		typedef Eigen::internal::scalar_inverse_op<float> t_scalar_inv_op;
		typedef Eigen::internal::scalar_constant_op<float> t_scalar_const_op;
		typedef std::binder2nd<std::greater<float> > t_scalar_gt_op; // Eigen has no internal::scalar_gt_op

		typedef Eigen::CwiseUnaryOp<t_scalar_abs_op, const t_sin_val_array> t_abs_val_array;
		typedef Eigen::CwiseUnaryOp<t_scalar_inv_op, const t_sin_val_array> t_inv_val_array;
		typedef Eigen::CwiseUnaryOp<t_scalar_gt_op , const t_abs_val_array> t_cmp_val_array; // 1 or 0

		typedef Eigen::CwiseNullaryOp<t_scalar_const_op, const t_inv_val_array> t_nul_val_array;
		typedef Eigen::Select<t_cmp_val_array, t_inv_val_array, t_nul_val_array> t_select;

		const t_svd_matrix svd_matrix(a, Eigen::ComputeThinU | Eigen::ComputeThinV);

		const t_matrix& v_matrix = svd_matrix.matrixV();
		const t_matrix& u_matrix = svd_matrix.matrixU();

		// diagonal matrix; singular values are returned in decreasing order of magnitude
		const t_sin_val_matrix& svals_mat = svd_matrix.singularValues();
		const t_sin_val_array& svals_arr = svals_mat.array();

		const t_inv_val_array& svals_inv_arr = svals_arr.inverse();
		const t_abs_val_array& svals_abs_arr = svals_arr.abs();
		const t_cmp_val_array& svals_cmp_arr = (svals_abs_arr > (epsilon * std::max(a.cols(), a.rows()) * svals_abs_arr(0)));
		const t_nul_val_array  const_nul_arr = t_nul_val_array(svals_arr.rows(), svals_arr.cols(), 0.0);

		// compute the inverses of all sv's whose absolute value exceeds epsilon
		// const t_select& sel_arr = svals_cmp_arr.select(svals_inv_arr, 0.0);
		const t_select& sel_arr = svals_cmp_arr.select(svals_inv_arr, const_nul_arr);
		const t_matrix& sel_mat = sel_arr.matrix();

		// Moore-Penrose pseudo-inverse
		return (v_matrix * sel_mat.asDiagonal() * u_matrix.adjoint());
	}


	// this surpresses "defined but not used" warnings
	template<typename t_dummy = void>
	static t_mat44f compose_transform_matrix(const t_pos3f& pos, const t_rot4f& rot) {
		t_mat44f m;

		const t_pos3f& t = pos;
		const t_mat33f& r = rot.toRotationMatrix();

		// X-column
		m(0, 0) = r(0, 0);
		m(1, 0) = r(1, 0);
		m(2, 0) = r(2, 0);
		m(3, 0) = 0.0f;
		// Y-column
		m(0, 1) = r(0, 1);
		m(1, 1) = r(1, 1);
		m(2, 1) = r(2, 1);
		m(3, 1) = 0.0f;
		// Z-column
		m(0, 2) = r(0, 2);
		m(1, 2) = r(1, 2);
		m(2, 2) = r(2, 2);
		m(3, 2) = 0.0f;
		// T-column
		m(0, 3) = t.x();
		m(1, 3) = t.y();
		m(2, 3) = t.z();
		m(3, 3) = 1.0f;
		return m;
	}

	template<typename t_dummy = void>
	static t_mat44f compose_transform_matrix(const t_xform& xform) {
		return (compose_transform_matrix(xform.pos, xform.rot));
	}
};

#endif

