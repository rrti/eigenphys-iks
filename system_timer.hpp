#ifndef EIGENPHYSIKS_SYSTEM_TIMER_HDR
#define EIGENPHYSIKS_SYSTEM_TIMER_HDR

#include <chrono>

namespace util {
	struct t_system_timer {
	public:
		uint64_t tick_time() { return (tick(),           0); }
		uint64_t tock_time() { return (tock(), diff_time()); }
		uint64_t diff_time() const { return ((nsecs()).count()); }

	private:
		std::chrono::high_resolution_clock::time_point& tick() { return (t0 = std::chrono::high_resolution_clock::now()); }
		std::chrono::high_resolution_clock::time_point& tock() { return (t1 = std::chrono::high_resolution_clock::now()); }
		std::chrono::nanoseconds nsecs() const { return (t1 - t0); }

	private:
		std::chrono::high_resolution_clock::time_point t0;
		std::chrono::high_resolution_clock::time_point t1;
	};
};

#endif

