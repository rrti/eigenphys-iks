#ifndef EIGENPHYSIKS_GLOBAL_CONSTS_HDR
#define EIGENPHYSIKS_GLOBAL_CONSTS_HDR

namespace consts {
	static constexpr uint32_t SIM_STEP_RATE    = 60 * 2; // Hz
	static constexpr    float SIM_STEP_SIZE    = 1.0f / SIM_STEP_RATE; // dt (ms)
	static constexpr    float SIM_STEP_SIZE_SQ = SIM_STEP_SIZE * SIM_STEP_SIZE;
	static constexpr uint64_t SIM_STEP_TIME_NS = (1000.0f / SIM_STEP_RATE) * 1000 * 1000;
	static constexpr uint64_t WALL_SEC_TIME_NS = 1000 * 1000 * 1000;
};

#endif

