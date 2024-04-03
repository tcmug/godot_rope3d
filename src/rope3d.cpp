#include "rope3d.hpp"
#include "godot_cpp/classes/joint3d.hpp"
#include "godot_cpp/classes/pin_joint3d.hpp"
#include "godot_cpp/variant/packed_vector3_array.hpp"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/capsule_shape3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/cone_twist_joint3d.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/generic6_dof_joint3d.hpp>
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
	ClassDB::bind_method(D_METHOD("set_rope_segment_length", "value"), &Rope3D::set_rope_segment_length);
	ClassDB::bind_method(D_METHOD("set_rope_max_segments", "value"), &Rope3D::set_rope_max_segments);
	ClassDB::bind_method(D_METHOD("set_rope_segment_mass", "value"), &Rope3D::set_rope_segment_mass);
	ClassDB::bind_method(D_METHOD("set_rope_segment_angular_damp", "value"), &Rope3D::set_rope_segment_angular_damp);

	ClassDB::bind_method(D_METHOD("get_rope_width"), &Rope3D::get_rope_width);
	ClassDB::bind_method(D_METHOD("get_rope_segment_length"), &Rope3D::get_rope_segment_length);
	ClassDB::bind_method(D_METHOD("get_rope_max_segments"), &Rope3D::get_rope_max_segments);
	ClassDB::bind_method(D_METHOD("get_rope_segment_mass"), &Rope3D::get_rope_segment_mass);
	ClassDB::bind_method(D_METHOD("get_rope_segment_angular_damp"), &Rope3D::get_rope_segment_angular_damp);

	ClassDB::bind_method(D_METHOD("set_material", "new_material"), &Rope3D::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &Rope3D::get_material);

	ADD_GROUP("Collision", "collision_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_GROUP("Rope", "rope_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_width"), "set_rope_width", "get_rope_width");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_segment_length"), "set_rope_segment_length", "get_rope_segment_length");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "rope_max_segments", PROPERTY_HINT_RANGE, "1,20,1"), "set_rope_max_segments", "get_rope_max_segments");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_node_a", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysicsBody3D"), "set_rope_node_a", "get_rope_node_a");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_node_b", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "PhysicsBody3D"), "set_rope_node_b", "get_rope_node_b");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_segment_mass"), "set_rope_segment_mass", "get_rope_segment_mass");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_segment_angular_damp"), "set_rope_segment_angular_damp", "get_rope_segment_angular_damp");

	ADD_GROUP("Geometry", "geometry_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::OBJECT, "geometry_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");
}

Rope3D::Rope3D() {
	collision_layer = 1;
	collision_mask = 1;

	rope_width = 0.1;
	rope_segment_angular_damp = 50.0;
	rope_segment_mass = 0.1;
	rope_segment_length = 0.2;
	rope_max_segments = 5;

	is_created = false;
	mesh_instance = nullptr;
}

Rope3D::~Rope3D() {
}

void Rope3D::_ready() {
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

real_t Rope3D::get_rope_segment_length() const {
	return rope_segment_length;
}

real_t Rope3D::get_rope_segment_mass() const {
	return rope_segment_mass;
}

int Rope3D::get_rope_max_segments() const {
	return rope_max_segments;
}

real_t Rope3D::get_rope_width() const {
	return rope_width;
}

void Rope3D::set_rope_segment_angular_damp(real_t value) {
	rope_segment_angular_damp = value;
}

void Rope3D::set_rope_segment_mass(real_t value) {
	rope_segment_mass = value;
}

void Rope3D::set_rope_segment_length(real_t value) {
	rope_segment_length = value;
}

void Rope3D::set_rope_max_segments(int value) {
	rope_max_segments = value;
}

void Rope3D::set_rope_width(real_t value) {
	rope_width = value;
}

real_t Rope3D::get_rope_segment_angular_damp() const {
	return rope_segment_angular_damp;
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

RigidBody3D *Rope3D::create_segment(const Vector3 &origin, const Vector3 &next, real_t segment_length) {
	// Prepare shape.
	//CylinderShape3D *collider = memnew(CylinderShape3D);
	CapsuleShape3D *collider = memnew(CapsuleShape3D);

	real_t bake_interval = get_curve().ptr()->get_bake_interval();
	collider->set_height(segment_length * 0.95);
	collider->set_radius(rope_width);

	// Prepare collider.
	CollisionShape3D *shape = memnew(CollisionShape3D);
	shape->set_shape(collider);
	shape->set_rotation_degrees(Vector3(90, 0, 0));

	// Prepare sphysics body.
	RigidBody3D *segment = memnew(RigidBody3D);
	add_child(segment);
	segment->set_use_continuous_collision_detection(true);

	Node3D *tracker = memnew(Node3D);
	tracker->set_position(Vector3(0, 0, segment_length * 0.5));
	segment->add_child(tracker);

	tracker = memnew(Node3D);
	tracker->set_position(Vector3(0, 0, segment_length * -0.5));
	segment->add_child(tracker);

	segment->add_child(shape);

	segment->set_mass(rope_segment_mass);
	segment->set_angular_damp(rope_segment_angular_damp);

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
	Vector3 physics_origin = (origin + next) * 0.5;
	segment->look_at_from_position(physics_origin, next, Vector3(0, 1, 0));
	return segment;
}

Joint3D *Rope3D::create_pivot(const Vector3 &origin, const NodePath &a, const NodePath &b) {
	//Generic6DOFJoint3D *pivot = memnew(Generic6DOFJoint3D);
	PinJoint3D *pivot = memnew(PinJoint3D);
	add_child(pivot);
	// Order of the following is important.
	pivot->set_global_position(origin);
	pivot->set_node_a(a);
	pivot->set_node_b(b);

	/*
	  pivot->set_flag_x(Generic6DOFJoint3D::FLAG_ENABLE_ANGULAR_LIMIT, false);
	  pivot->set_flag_y(Generic6DOFJoint3D::FLAG_ENABLE_ANGULAR_LIMIT, false);
	  pivot->set_flag_z(Generic6DOFJoint3D::FLAG_ENABLE_ANGULAR_LIMIT, false);
	  pivot->set_flag_x(Generic6DOFJoint3D::FLAG_ENABLE_LINEAR_SPRING, true);
	  pivot->set_flag_y(Generic6DOFJoint3D::FLAG_ENABLE_LINEAR_SPRING, true);
	  pivot->set_flag_z(Generic6DOFJoint3D::FLAG_ENABLE_LINEAR_SPRING, true);
	  pivot->set_param_z(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_DAMPING, rope_joint_damping);
	  pivot->set_param_y(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_DAMPING, rope_joint_damping);
	  pivot->set_param_z(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_DAMPING, rope_joint_damping);
	  pivot->set_param_z(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_STIFFNESS, rope_joint_bias);
	  pivot->set_param_y(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_STIFFNESS, rope_joint_bias);
	  pivot->set_param_z(Generic6DOFJoint3D::PARAM_LINEAR_SPRING_STIFFNESS, rope_joint_bias);
	*/
	return pivot;
}

void Rope3D::create_rope() {
	Curve3D *curve = get_curve().ptr();
	PhysicsBody3D *node_a = Object::cast_to<PhysicsBody3D>(get_node_or_null(rope_node_a));
	PhysicsBody3D *node_b = Object::cast_to<PhysicsBody3D>(get_node_or_null(rope_node_b));
	PhysicsBody3D *previous = node_a;

	PhysicsBody3D *segment;

	real_t rope_length = curve->get_baked_length();
	real_t num_segments = min(rope_max_segments, round(rope_length / rope_segment_length)) + 1;
	real_t segment_length = rope_length / num_segments;

	Vector3 from = to_global(curve->sample_baked(0));

	for (int i = 0; i < num_segments; i++) {
		Vector3 to = to_global(curve->sample_baked((i + 1) * segment_length));

		segment = create_segment(from, to, segment_length);
		tracked_nodes.push_back(Object::cast_to<Node3D>(segment->get_child(0)));

		if (previous) {
			create_pivot(from, previous->get_path(), segment->get_path());
		}

		from = to;
		previous = segment;
	}

	// Create last segment and a joint if there is something we're attached to.
	tracked_nodes.push_back(Object::cast_to<Node3D>(previous->get_child(1)));
	segment = node_b;
	if (node_b) {
		create_pivot(from, previous->get_path(), segment->get_path());
	}

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
