#ifndef ROPE3D_H
#define ROPE3D_H

#include "godot_cpp/core/defs.hpp"
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/path3d.hpp>
#include <godot_cpp/classes/pin_joint3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

namespace godot {

class Rope3D : public Path3D {
	GDCLASS(Rope3D, Path3D)

private:
	int collision_layer;
	int collision_mask;
	NodePath rope_end;
	real_t rope_width;

	// Mesh:
	Ref<Material> material;
	MeshInstance3D *mesh_instance;
	Array geometry;
	PackedVector3Array vertex_buffer;
	PackedVector3Array normal_buffer;
	PackedFloat32Array tangent_buffer;
	PackedVector2Array uv_buffer;
	bool is_created;

protected:
	static void _bind_methods();
	void initialize_arrays();

	Vector<Node3D *> tracked_nodes;
	RigidBody3D *create_segment();
	void create_rope();
	void initialize_geometry();
	PhysicsBody3D *get_rope_end_ptr();

public:
	Rope3D();
	~Rope3D();

	void set_collision_mask(int value);
	int get_collision_mask() const;

	void set_collision_layer(int value);
	int get_collision_layer() const;

	void set_rope_end(NodePath value);
	NodePath get_rope_end() const;

	void set_rope_width(real_t value);
	real_t get_rope_width() const;

	Ref<Material> get_material() const;
	void set_material(const Ref<Material> material);
	// void set_rope_path(NodePath value);
	// NodePath get_rope_path() const;

	void _ready() override;
	void _process(double delta) override;
};
} // namespace godot

#endif
