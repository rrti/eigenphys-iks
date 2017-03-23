#ifndef EIGENPHYSIKS_WALL_CLOCK_HDR
#define EIGENPHYSIKS_WALL_CLOCK_HDR

#include <cstdio> // std::fprintf

#include "global_consts.hpp"

namespace util {
	struct t_wall_clock {
	public:
		bool update(uint64_t tock_time) {
			render_time_ns += (tock_time * render_dt_mult);
			system_time_ns += (tock_time                 );
			n_render_calls += 1;

			return (system_time_ns >= consts::WALL_SEC_TIME_NS);
		}

		void inv_render_dt_mult() { render_dt_mult = 1 - render_dt_mult; }
		void add_render_time_ns(uint64_t dt) { render_time_ns += dt; }
		void add_update_time_ns(uint64_t dt) { update_time_ns += dt; }
		void add_system_time_ns(uint64_t dt) { system_time_ns += dt; }

		void output_timings(FILE* out) {
			std::fprintf(out, "[wc::%s] {update,render}_calls={%u,%u}\n", __func__, n_update_calls, n_render_calls);
		}

		uint32_t add_update_call() { return (n_update_calls++); }

		uint64_t get_render_time_ns() const { return render_time_ns; }
		uint64_t get_system_time_ns() const { return system_time_ns; }

	private:
		uint32_t n_update_calls = 0; // total number of Update calls executed
		uint32_t n_render_calls = 0; // total number of Render calls executed

		uint32_t render_dt_mult = 1; // if 0, renderer produces no time for simulation

		uint64_t system_time_ns = 0; // total time (ns) spent in Update+Render calls
		uint64_t update_time_ns = 0; // total time (ns) spent in Update calls
		uint64_t render_time_ns = 0; // total time (ns) spent in Render calls
	};
};

#endif

