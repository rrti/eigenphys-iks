#ifndef EIGENPHYSIKS_INPUT_STATE_HDR
#define EIGENPHYSIKS_INPUT_STATE_HDR

namespace util {
	struct t_input_state {
		// need two arrays because GLUT_KEY_LEFT maps to 'd', etc
		uint8_t regular_keys[256] = {0};
		uint8_t special_keys[256] = {0};
	};
};

#endif

