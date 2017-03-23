#ifndef EIGENPHYSIKS_TYPES_HDR
#define EIGENPHYSIKS_TYPES_HDR

// by default, we want aligned and vectorized (SSE) code
// note: fails on win32 because SSE instructions have to
// be 16-byte aligned but the stack uses 4-byte alignment
#include <Eigen/Dense>

namespace math {
	// NB: these do *not* zero-initialize on construction
	typedef Eigen::Vector3f t_pos3f;
	typedef Eigen::Vector3f t_vec3f;
	typedef Eigen::AngleAxisf t_rot4f;

	typedef Eigen::Matrix<float,              1,              3> t_mat13f;
	typedef Eigen::Matrix<float,              3,              3> t_mat33f;
	typedef Eigen::Matrix<float,              4,              4> t_mat44f;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> t_matXYf; // Eigen::MatrixXf
	typedef Eigen::Matrix<float, Eigen::Dynamic,              1> t_matX1f;

	// composite transform for IK-chain pieces
	struct t_xform {
		t_pos3f pos;
		t_rot4f rot;
	};
};

using math::t_pos3f;
using math::t_vec3f;
using math::t_rot4f;

using math::t_mat13f;
using math::t_mat33f;
using math::t_mat44f;
using math::t_matXYf;
using math::t_matX1f;

using math::t_xform;

#endif

