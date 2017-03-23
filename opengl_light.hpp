#ifndef EIGENPHYSIKS_OPENGL_LIGHT_HDR
#define EIGENPHYSIKS_OPENGL_LIGHT_HDR

namespace opengl {
	struct t_light {
	public:
		t_light(uint8_t id = 0): m_id(id) {
			set_diff_clr(0.0f, 0.0f, 0.0f, 0.0f);
			set_spec_clr(0.0f, 0.0f, 0.0f, 0.0f);
			set_ambi_clr(0.0f, 0.0f, 0.0f, 0.0f);
			set_position(0.0f, 0.0f, 0.0f, 1.0f);
		}

		void enable() const;
		void disable() const;

		void set_gl_state() const;

		void set_diff_clr(float r, float g, float b, float a) { m_diff_clr[0] = r; m_diff_clr[1] = g; m_diff_clr[2] = b; m_diff_clr[3] = a; }
		void set_spec_clr(float r, float g, float b, float a) { m_spec_clr[0] = r; m_spec_clr[1] = g; m_spec_clr[2] = b; m_spec_clr[3] = a; }
		void set_ambi_clr(float r, float g, float b, float a) { m_ambi_clr[0] = r; m_ambi_clr[1] = g; m_ambi_clr[2] = b; m_ambi_clr[3] = a; }
		void set_position(float x, float y, float z, float w) { m_position[0] = x; m_position[1] = y; m_position[2] = z; m_position[3] = w; }

	private:
		uint8_t m_id;

		float m_diff_clr[4];
		float m_ambi_clr[4];
		float m_spec_clr[4];
		float m_position[4]; // w=1 means positional
	};
};

#endif

