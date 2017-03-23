#ifndef EIGENPHYSIKS_STATE_HDR
#define EIGENPHYSIKS_STATE_HDR

#include "eigen_ik_solver.hpp"
#include "spring_rope.hpp"
#include "world_consts.hpp"

namespace epiks {
	struct t_physics_state {
	public:
		t_physics_state(): m_rope{consts::ROPE_PARAMS, consts::SPRING_PARAMS, consts::WORLD_PARAMS} {}

		void init();
		void kill() {}
		void step(float dt);

		const epiks::t_spring_grid& get_rope() const { return m_rope; }
		      epiks::t_spring_grid& get_rope()       { return m_rope; }

		const epiks::t_rb_chain& get_arm(size_t i) const { return m_arms[i]; }
		      epiks::t_rb_chain& get_arm(size_t i)       { return m_arms[i]; }

	public:
		static constexpr size_t NUM_ARMS = 6;

	private:
		epiks::t_spring_grid m_rope;
		epiks::t_rb_chain m_arms[NUM_ARMS];
	};
};

#endif

