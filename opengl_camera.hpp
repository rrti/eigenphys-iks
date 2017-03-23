#ifndef EIGENPHYSIKS_OPENGL_CAMERA_HDR
#define EIGENPHYSIKS_OPENGL_CAMERA_HDR

#include "eigen_types.hpp"
#include "input_state.hpp"
#include "global_consts.hpp"
#include "world_consts.hpp"

namespace opengl {
	struct t_camera {
	public:
		t_camera() {
			set_axes();
			set_pos({0.0f, 5.0f, 10.0f});
			set_tgt(get_pos() + get_z_vec());
		}

		void set_axes() {
			m_vec[consts::AXIS_IDX_X] =  consts::WORLD_AXES[consts::AXIS_IDX_X];
			m_vec[consts::AXIS_IDX_Y] =  consts::WORLD_AXES[consts::AXIS_IDX_Y];
			m_vec[consts::AXIS_IDX_Z] = -consts::WORLD_AXES[consts::AXIS_IDX_Z];
		}

		void set_pos(const t_pos3f& pos) { m_pos = pos; }
		void set_tgt(const t_pos3f& tgt) { m_tgt = tgt; }
		void set_vec(size_t i, const t_vec3f& vec) { m_vec[i] = vec; }

		void update();
		void clamp_pos(const t_pos3f& mins, const t_pos3f& maxs) {
			m_pos.x() = std::max(mins.x(), std::min(maxs.x(), m_pos.x()));
			m_pos.y() = std::max(mins.y(), std::min(maxs.y(), m_pos.y()));
			m_pos.z() = std::max(mins.z(), std::min(maxs.z(), m_pos.z()));
		}

		void set_gl_proj_mat() const;
		void set_gl_view_mat() const;

		const t_pos3f& get_pos() const { return m_pos; }
		const t_pos3f& get_tgt() const { return m_tgt; }
		const t_vec3f& get_vec(size_t i) const { return m_vec[i]; }

		const t_vec3f& get_x_vec() const { return (get_vec(consts::AXIS_IDX_X)); }
		const t_vec3f& get_y_vec() const { return (get_vec(consts::AXIS_IDX_Y)); }
		const t_vec3f& get_z_vec() const { return (get_vec(consts::AXIS_IDX_Z)); }

		t_vec3f cross_zy() const { return (m_vec[consts::AXIS_IDX_Z].cross(m_vec[consts::AXIS_IDX_Y])); }

	public:
		t_pos3f m_pos;
		t_vec3f m_vec[3];
		t_pos3f m_tgt;
	};
};

#endif

