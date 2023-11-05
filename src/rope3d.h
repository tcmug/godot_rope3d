#ifndef Rope3D__H
#define Rope3D__H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/path3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>

namespace godot {

	class Rope3D: public Node3D {
		GDCLASS(Rope3D, Node3D)

		private:

			int collision_layer;
			int collision_mask;
			NodePath rope_end;
			// NodePath rope_path;

		protected:

			static void _bind_methods();
			void initialize_arrays();

			void create_segment(Vector3 a, Vector3 b);

		public:
			Rope3D();
			~Rope3D();

			void set_collision_mask(int value);
			int get_collision_mask() const;

			void set_collision_layer(int value);
			int get_collision_layer() const;

			void set_rope_end(NodePath value);
			NodePath get_rope_end() const;

			// void set_rope_path(NodePath value);
			// NodePath get_rope_path() const;

			void _ready() override;
			void _process(double delta) override;

	};
}

#endif
