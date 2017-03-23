#ifndef EIGENPHYSIKS_ENGINE_HDR
#define EIGENPHYSIKS_ENGINE_HDR

#include "input_state.hpp"
#include "physics_state.hpp"
#include "render_state.hpp"
#include "system_timer.hpp"
#include "wall_clock.hpp"

namespace epiks {
	struct t_eigen_engine {
	public:
		void loop();
		void init() {
			m_physics_state.init();
			m_render_state.init(m_physics_state);
		}
		void kill() {
			m_render_state.kill();
			m_physics_state.kill();
		}

		void regular_key_pressed(uint8_t key) {
			m_input_state.regular_keys[key] = 1;

			switch (key) {
				case 'p': { m_wall_clock.inv_render_dt_mult(                        ); } break;
				case 'f': { m_wall_clock.add_render_time_ns(consts::SIM_STEP_TIME_NS); } break;
				case  27: {                            exit(                       0); } break;
			}
		}
		void regular_key_released(uint8_t key) { m_input_state.regular_keys[key] = 0; }
		void special_key_pressed (int32_t key) { m_input_state.special_keys[key] = 1; }
		void special_key_released(int32_t key) { m_input_state.special_keys[key] = 0; }

		void handle_input(float dt);
		void update_frame();
		void render_frame();

	private:
		util::t_system_timer m_render_timer;
		util::t_system_timer m_update_timer;
		util::t_wall_clock m_wall_clock;
		util::t_input_state m_input_state;

		epiks::t_physics_state m_physics_state;
		opengl::t_render_state m_render_state;
	};
};

#endif

