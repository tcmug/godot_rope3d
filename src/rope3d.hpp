#ifndef ROPE3D_H
#define ROPE3D_H

#include "godot_cpp/core/defs.hpp"
#include <godot_cpp/classes/joint3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/path3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>

namespace godot {

class Rope3D : public Path3D {
	GDCLASS(Rope3D, Path3D)

private:
	int collision_layer;
	int collision_mask;

	NodePath rope_node_a;
	NodePath rope_node_b;

	int rope_num_segments;

	real_t rope_width;
	real_t rope_joint_bias;
	real_t rope_joint_damping;
	real_t rope_joint_impulse_clamp;
	real_t rope_segment_mass;
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

	RigidBody3D *create_segment(const Vector3 &origin, const Vector3 &next, real_t segment_length);
	Joint3D *create_pivot(const Vector3 &origin, const NodePath &a, const NodePath &b);

	void create_rope();
	void initialize_geometry();

public:
	Rope3D();
	~Rope3D();

	void set_collision_mask(int value);
	int get_collision_mask() const;

	void set_collision_layer(int value);
	int get_collision_layer() const;

	void set_rope_node_a(NodePath value);
	void set_rope_node_b(NodePath value);

	NodePath get_rope_node_a() const;
	NodePath get_rope_node_b() const;

	void set_rope_num_segments(int value);
	void set_rope_width(real_t value);
	void set_rope_joint_bias(real_t value);
	void set_rope_joint_damping(real_t value);
	void set_rope_joint_impulse_clamp(real_t value);

	int get_rope_num_segments() const;
	real_t get_rope_width() const;
	real_t get_rope_joint_bias() const;
	real_t get_rope_joint_damping() const;
	real_t get_rope_joint_impulse_clamp() const;

	Ref<Material> get_material() const;
	void set_material(const Ref<Material> material);

	void _ready() override;
	void _process(double delta) override;
};
} // namespace godot

#endif
