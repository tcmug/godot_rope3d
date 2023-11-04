#ifndef Rope3D__H
#define Rope3D__H

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>

namespace godot {

	class Rope3D: public MeshInstance3D {
		GDCLASS(Rope3D, MeshInstance3D)

		private:

			int collision_layer;
			int collision_mask;

		protected:
			static void _bind_methods();
			void initialize_arrays();

		public:
			Rope3D();
			~Rope3D();

			void set_collision_mask(int value);
			int get_collision_mask() const;

			void set_collision_layer(int value);
			int get_collision_layer() const;

			void _ready() override;
			void _process(double delta) override;

	};
}

#endif
