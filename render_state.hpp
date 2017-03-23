#ifndef EIGENPHYSIKS_RENDER_STATE_HDR
#define EIGENPHYSIKS_RENDER_STATE_HDR

#include "eigen_types.hpp"
#include "opengl_camera.hpp"
#include "opengl_light.hpp"

struct GLUquadric;

namespace epiks {
	struct t_physics_state;
};

namespace opengl {
	struct t_render_state {
	public:
		void init(const epiks::t_physics_state& ps);
		void kill();

		void setup_camera();
		void setup_lights(const epiks::t_physics_state& ps);
		void swap_buffers() const;
		void render_scene(const epiks::t_physics_state& ps) const;

		opengl::t_camera& get_camera() { return m_opengl_camera; }
		opengl::t_light& get_light(size_t i) { return m_opengl_lights[i]; }

	public:
		uint32_t m_cone_vbo_id;
		uint32_t m_quad_vbo_id;
		uint32_t m_rope_vbo_id;

		static constexpr uint32_t cone_vbo_elem_size = sizeof(t_pos3f) + sizeof(t_vec3f); // v+n
		static constexpr uint32_t cone_vbo_flag_bits = GL_MAP_WRITE_BIT;
		static constexpr uint32_t quad_vbo_elem_size = sizeof(t_pos3f) + sizeof(t_vec3f); // v+n
		static constexpr uint32_t quad_vbo_flag_bits = GL_MAP_WRITE_BIT;
		static constexpr uint32_t rope_vbo_elem_size = sizeof(t_pos3f); // v
		static constexpr uint32_t rope_vbo_flag_bits = GL_MAP_WRITE_BIT | GL_MAP_PERSISTENT_BIT;

		static constexpr uint32_t num_cone_divs          = 15;
		static constexpr uint32_t num_cone_div_elems     = 2 * 3; // elements per div
		static constexpr uint32_t num_cone_elems         = num_cone_divs * num_cone_div_elems;
		static constexpr uint32_t num_quad_elems         = 4;
		static constexpr uint32_t num_cone_elem_floats   = cone_vbo_elem_size / sizeof(float); // floats per element
		static constexpr uint32_t num_cone_div_floats    = num_cone_div_elems * num_cone_elem_floats; // floats per div
		static constexpr uint32_t num_cone_vbo_bytes     = num_cone_elems * cone_vbo_elem_size;
		static constexpr uint32_t num_quad_vbo_bytes     = num_quad_elems * quad_vbo_elem_size;

		static constexpr void* cone_vbo_vertex_offset = reinterpret_cast<void*>(              0);
		static constexpr void* cone_vbo_normal_offset = reinterpret_cast<void*>(sizeof(t_pos3f));
		static constexpr void* quad_vbo_vertex_offset = reinterpret_cast<void*>(              0);
		static constexpr void* quad_vbo_normal_offset = reinterpret_cast<void*>(sizeof(t_pos3f));
		static constexpr void* rope_vbo_vertex_offset = reinterpret_cast<void*>(              0);

		float* m_cone_vbo_ptr;
		float* m_quad_vbo_ptr;
		float* m_rope_vbo_ptr;

		GLUquadric* m_quadric_obj;

	public:
		opengl::t_camera m_opengl_camera;
		opengl::t_light m_opengl_lights[2];
	};
};

#endif

