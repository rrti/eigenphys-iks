#ifndef EIGENPHYSIKS_WORLD_CONSTS_HDR
#define EIGENPHYSIKS_WORLD_CONSTS_HDR

namespace epiks {
	struct t_spring_grid_params {
		size_t num_links_x;
		size_t num_links_y;

		float link_mass;
		float dummy_var;

		t_vec3f gravity_acc;
		t_vec3f pulling_acc;
	};

	struct t_spring_anchor {
		size_t obj_idx;

		t_pos3f pos;
		t_vec3f vel;
	};

	struct t_spring_base_params {
		float rest_length;
		float stiff_const; // stiffness
		float frict_const; // internal damping
	};

	struct t_world_params {
		float  atmos_frict_coeff;
		float ground_repul_coeff;
		float ground_frict_coeff;
		float ground_absor_coeff;
		float ground_plane_level;
		float ground_plane_scale;
	};
};

namespace consts {
	enum {
		AXIS_IDX_X   = 0,
		AXIS_IDX_Y   = 1,
		AXIS_IDX_Z   = 2,

		AXIS_IDX_XZ  = 3,
		AXIS_IDX_XY  = 4,
		AXIS_IDX_YZ  = 5,
		AXIS_IDX_XYZ = 6,
	};


	// NOTE:
	//   ground-repulsion and spring-stiffness can not be too large
	//   or the simulation will numerically blow up, depends on the
	//   time-step size and integration method
	static const     epiks::t_spring_grid_params ROPE_PARAMS = {1, 30,  0.05f, 0.0f,  {0.0f, -9.81f, 0.0f}, {0.0f, 0.0f, 0.0f}};
	static const     epiks::t_spring_anchor      ROPE_ANCHOR = {0, {0.0f, 5.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

	static constexpr epiks::t_spring_base_params SPRING_PARAMS = {0.05f, 100.0f, 0.2f};
	static constexpr epiks::t_world_params WORLD_PARAMS = {0.02f, 100.0f, 0.2f, 2.0f, 0.0f, 5.0f};

	static const t_vec3f WORLD_AXES[AXIS_IDX_XYZ + 1] = {
		t_vec3f(1.0f, 0.0f, 0.0f), // x
		t_vec3f(0.0f, 1.0f, 0.0f), // y
		t_vec3f(0.0f, 0.0f, 1.0f), // z

		t_vec3f(1.0f, 0.0f, 1.0f), // xz
		t_vec3f(1.0f, 1.0f, 0.0f), // xy
		t_vec3f(0.0f, 1.0f, 1.0f), // yz
		t_vec3f(1.0f, 1.0f, 1.0f), // xyz
	};
};

#endif

