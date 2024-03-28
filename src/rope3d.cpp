#include "rope3d.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

using namespace godot;

void Rope3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_collision_layer", "value"), &Rope3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Rope3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "value"), &Rope3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Rope3D::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_rope_node_a", "value"), &Rope3D::set_rope_node_a);
	ClassDB::bind_method(D_METHOD("get_rope_node_a"), &Rope3D::get_rope_node_a);

	ClassDB::bind_method(D_METHOD("set_rope_node_b", "value"), &Rope3D::set_rope_node_b);
	ClassDB::bind_method(D_METHOD("get_rope_node_b"), &Rope3D::get_rope_node_b);

	ClassDB::bind_method(D_METHOD("set_rope_width", "value"), &Rope3D::set_rope_width);
	ClassDB::bind_method(D_METHOD("set_rope_joint_bias", "value"), &Rope3D::set_rope_joint_bias);
	ClassDB::bind_method(D_METHOD("set_rope_joint_damping", "value"), &Rope3D::set_rope_joint_damping);
	ClassDB::bind_method(D_METHOD("set_rope_joint_impulse_clamp", "value"), &Rope3D::set_rope_joint_impulse_clamp);
	ClassDB::bind_method(D_METHOD("get_rope_width"), &Rope3D::get_rope_width);
	ClassDB::bind_method(D_METHOD("get_rope_joint_bias"), &Rope3D::get_rope_joint_bias);
	ClassDB::bind_method(D_METHOD("get_rope_joint_damping"), &Rope3D::get_rope_joint_damping);
	ClassDB::bind_method(D_METHOD("get_rope_joint_impulse_clamp"), &Rope3D::get_rope_joint_impulse_clamp);

	ClassDB::bind_method(D_METHOD("set_material", "new_material"), &Rope3D::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &Rope3D::get_material);

	ADD_GROUP("Collision", "collision_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_GROUP("Rope", "rope_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_node_a", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysicsBody3D"), "set_rope_node_a", "get_rope_node_a");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_node_b", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysicsBody3D"), "set_rope_node_b", "get_rope_node_b");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_width"), "set_rope_width", "get_rope_width");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_joint_bias"), "set_rope_joint_bias", "get_rope_joint_bias");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_joint_damping"), "set_rope_joint_damping", "get_rope_joint_damping");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_joint_impulse_clamp"), "set_rope_joint_impulse_clamp", "get_rope_joint_impulse_clamp");

	ADD_GROUP("Geometry", "geometry_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::OBJECT, "geometry_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");
}

Rope3D::Rope3D() {
	collision_layer = 1;
	collision_mask = 1;
	rope_width = 0.05;
	rope_joint_bias = 0.3;
	rope_joint_damping = 1.0;
	rope_joint_impulse_clamp = 0.0;
	rope_segment_mass = 0.1;
	is_created = false;
	mesh_instance = nullptr;
}

Rope3D::~Rope3D() {
}

void Rope3D::_ready() {
	// Uncomment to reset link and materials for debugging and crash avoidance.
	// rope_end = NodePath();
	// material = Ref<Material>(nullptr);
	// Before spawning the mesh, need to figure out which end OWNs the mesh?
	if (Engine::get_singleton()->is_editor_hint()) {
		set_process(false);
	}
}

void Rope3D::set_rope_node_a(NodePath value) {
	rope_node_a = value;
}

void Rope3D::set_rope_node_b(NodePath value) {
	rope_node_b = value;
}

NodePath Rope3D::get_rope_node_a() const {
	return rope_node_a;
}

NodePath Rope3D::get_rope_node_b() const {
	return rope_node_b;
}

void Rope3D::set_rope_width(real_t value) {
	rope_width = value;
}

void Rope3D::set_rope_joint_bias(real_t value) {
	rope_joint_bias = value;
}

void Rope3D::set_rope_joint_damping(real_t value) {
	rope_joint_damping = value;
}

void Rope3D::set_rope_joint_impulse_clamp(real_t value) {
	rope_joint_impulse_clamp = value;
}

real_t Rope3D::get_rope_width() const {
	return rope_width;
}

real_t Rope3D::get_rope_joint_bias() const {
	return rope_joint_bias;
}

real_t Rope3D::get_rope_joint_damping() const {
	return rope_joint_damping;
}

real_t Rope3D::get_rope_joint_impulse_clamp() const {
	return rope_joint_impulse_clamp;
}

void Rope3D::set_collision_mask(int value) {
	collision_mask = value;
}

int Rope3D::get_collision_mask() const {
	return collision_mask;
}

void Rope3D::set_collision_layer(int value) {
	collision_layer = value;
}

int Rope3D::get_collision_layer() const {
	return collision_layer;
}

void Rope3D::set_material(Ref<Material> new_material) {
	material = new_material;
}

Ref<Material> Rope3D::get_material() const {
	return material;
}

RigidBody3D *Rope3D::create_segment() {
	// Prepare shape.
	CylinderShape3D *collider = memnew(CylinderShape3D);
	real_t bake_interval = get_curve().ptr()->get_bake_interval();
	collider->set_height(bake_interval);
	collider->set_radius(rope_width);

	// Prepare collider.
	CollisionShape3D *shape = memnew(CollisionShape3D);
	shape->set_shape(collider);
	shape->set_rotation_degrees(Vector3(90, 0, 0));

	// Prepare sphysics body.
	RigidBody3D *segment = memnew(RigidBody3D);
	segment->set_as_top_level(false);
	segment->add_child(shape);
	segment->set_mass(rope_segment_mass);

	// Set properties.
	// TODO: Make these tweakable!
	//segment->set_linear_damp(0.0);
	segment->set_angular_damp(100.0);
	segment->set_collision_layer(collision_layer);
	segment->set_collision_mask(collision_mask);

	// Add under self, as top level object.
	//add_child(segment);

	// Align and orient segment.
	//segment->look_at_from_position(origin, origin + direction, Vector3(0, 1, 0));

	/*
	TODO: Implement collision monitor later
	  if (collision_monitor) {
		  segment->contact_monitor = true
		  segment->contacts_reported = 1
		  segment->set_script(segment_script)
		  segment->rope = self
		  segment->connect("body_entered", segment, '_on_body_entered')
		  segment->connect("rope_body_entered", self, '_on_rope_body_entered')
	}*/

	return segment;
}

void Rope3D::create_rope() {
	//Vector3 point_a = get_global_transform().origin;
	//Vector3 point_b = target->get_global_transform().origin;
	// Create points that make up the rope.
	Curve3D *curve = get_curve().ptr();
	PackedVector3Array pivot_points = curve->get_baked_points();

	PhysicsBody3D *node_a = Object::cast_to<PhysicsBody3D>(get_node_or_null(rope_node_a));
	PhysicsBody3D *node_b = Object::cast_to<PhysicsBody3D>(get_node_or_null(rope_node_b));
	PhysicsBody3D *previous = node_a;

	Vector3 direction;
	real_t half_bake_interval = curve->get_bake_interval() * 0.5;
	Vector3 global_pos = get_global_position();
	PinJoint3D *pivot;
	PhysicsBody3D *segment;
	for (int i = 0; i < pivot_points.size(); i++) {
		Vector3 origin = to_global(pivot_points[i]);
		if (i < pivot_points.size() - 1) {
			direction = origin.direction_to(to_global(pivot_points[i + 1]));
		}

		if (i < pivot_points.size() - 1) {
			segment = create_segment();
			add_child(segment);
			segment->look_at_from_position(origin + (direction * half_bake_interval), origin + direction, Vector3(0, 1, 0));
			tracked_nodes.push_back(segment);
		} else {
			segment = node_b;
			if (!node_b) {
				previous = nullptr;
			}
		}

		if (previous) {
			pivot = memnew(PinJoint3D);
			add_child(pivot);
			pivot->set_param(PinJoint3D::PARAM_BIAS, rope_joint_bias);
			pivot->set_param(PinJoint3D::PARAM_DAMPING, rope_joint_damping);
			pivot->set_param(PinJoint3D::PARAM_IMPULSE_CLAMP, rope_joint_impulse_clamp);
			pivot->set_global_position(origin);
			pivot->set_node_a(previous->get_path());
			pivot->set_node_b(segment->get_path());
		}

		previous = segment;
	}

	// When cutting a rope, we know the segment, from the segment
	// we can fetch the rope owner and pinjoints that make up the point.
	// copy material and other properties & construct a new rope simply from the points.

	initialize_geometry();
	UtilityFunctions::print("Rope created.");
}

void Rope3D::initialize_geometry() {
	UtilityFunctions::print("Initialize geometry.");
	int num_points = tracked_nodes.size();
	vertex_buffer.resize(num_points * 2);
	normal_buffer.resize(num_points * 2);
	tangent_buffer.resize(num_points * 2 * 4);
	uv_buffer.resize(num_points * 2);

	geometry.resize(ArrayMesh::ARRAY_MAX);
	geometry[ArrayMesh::ARRAY_VERTEX] = vertex_buffer;
	geometry[ArrayMesh::ARRAY_NORMAL] = normal_buffer;
	geometry[ArrayMesh::ARRAY_TANGENT] = tangent_buffer;
	geometry[ArrayMesh::ARRAY_TEX_UV] = uv_buffer;

	mesh_instance = memnew(MeshInstance3D);
	mesh_instance->set_material_override(material);
	ArrayMesh *mesh = memnew(ArrayMesh);
	mesh_instance->set_mesh(Ref(mesh));
	add_child(mesh_instance);
}

void Rope3D::_process(double delta) {
	if (!is_created) {
		UtilityFunctions::print("Rope created");
		create_rope();
		is_created = true;
	}

	int num_vertices = vertex_buffer.size();
	int num_points = tracked_nodes.size();
	Camera3D *camera = get_viewport()->get_camera_3d();

	if (camera && mesh_instance) {
		// Transform points to the vertex buffer.
		const Vector3 camera_position = to_local(camera->get_global_position());

		int vi = 0, ci = 0, ni = 0, uvi = 0, ti = 0;

		// These are for visibility AABB:
		Vector3 min_pos = Vector3(10000, 10000, 10000);
		Vector3 max_pos = Vector3(-10000, -10000, -10000);

		Vector3 direction_vector;

		for (int i = 0; i < num_points; i++) {
			Vector3 position = to_local(tracked_nodes[i]->get_global_position());
			Vector3 normal = position.direction_to(camera_position);
			if (i < num_points - 1) {
				direction_vector = position.direction_to(
						to_local(tracked_nodes[i + 1]->get_global_position()));
			}
			Vector3 orientation = direction_vector.cross(normal);
			Vector3 tangent = direction_vector;
			double sz = rope_width;

			Vector3 edge_vector = orientation * sz;

			// Keep track of min and max points.
			min_pos.x = min(position.x, min_pos.x);
			min_pos.y = min(position.y, min_pos.y);
			min_pos.z = min(position.z, min_pos.z);
			max_pos.x = max(position.x, max_pos.x);
			max_pos.y = max(position.y, max_pos.y);
			max_pos.z = max(position.z, max_pos.z);

			vertex_buffer[vi++] = position + edge_vector;
			vertex_buffer[vi++] = position - edge_vector;

			normal_buffer[ni++] = normal;
			normal_buffer[ni++] = normal;

			tangent_buffer[ti++] = tangent.x;
			tangent_buffer[ti++] = tangent.y;
			tangent_buffer[ti++] = tangent.z;
			tangent_buffer[ti++] = 1;
			tangent_buffer[ti++] = tangent.x;
			tangent_buffer[ti++] = tangent.y;
			tangent_buffer[ti++] = tangent.z;
			tangent_buffer[ti++] = 1;

			double ux = i / double(num_points);
			double x = ux;
			uv_buffer[uvi++] = Vector2(x, 0);
			uv_buffer[uvi++] = Vector2(x, 1);
		}

		// Set mesh aabb for visibility;
		Vector3 aabb_center = (min_pos + max_pos) * 0.5;
		Vector3 aabb_size = (max_pos - aabb_center);
		AABB aabb(aabb_center, aabb_size);
		mesh_instance->set_custom_aabb(aabb);

		Ref<ArrayMesh> mesh = mesh_instance->get_mesh();
		if (mesh.is_valid()) {
			ArrayMesh *array_mesh = mesh.ptr();
			if (array_mesh) {
				array_mesh->clear_surfaces();
				geometry[ArrayMesh::ARRAY_VERTEX] = vertex_buffer;
				geometry[ArrayMesh::ARRAY_NORMAL] = normal_buffer;
				geometry[ArrayMesh::ARRAY_TANGENT] = tangent_buffer;
				geometry[ArrayMesh::ARRAY_TEX_UV] = uv_buffer;
				array_mesh->add_surface_from_arrays(Mesh::PrimitiveType::PRIMITIVE_TRIANGLE_STRIP, geometry);
			}
		}

		/*
				Ref<ShaderMaterial> mat = get_material_override();
				if (mat.is_valid()) {
					ShaderMaterial *material = mat.ptr();
					material->set_shader_parameter("MAX_VERTICES", float(num_vertices));
					material->set_shader_parameter("SPAWN_INTERVAL_SECONDS", float(update_interval));
				}
			}*/
	}
}
