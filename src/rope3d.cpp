#include "rope3d.hpp"

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
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

	ClassDB::bind_method(D_METHOD("set_rope_end", "value"), &Rope3D::set_rope_end);
	ClassDB::bind_method(D_METHOD("get_rope_end"), &Rope3D::get_rope_end);

	ClassDB::bind_method(D_METHOD("set_width", "value"), &Rope3D::set_width);
	ClassDB::bind_method(D_METHOD("get_width"), &Rope3D::get_width);

	ClassDB::bind_method(D_METHOD("set_segment_length", "value"), &Rope3D::set_segment_length);
	ClassDB::bind_method(D_METHOD("get_segment_length"), &Rope3D::get_segment_length);

	ClassDB::bind_method(D_METHOD("set_material", "new_material"), &Rope3D::set_material);
	ClassDB::bind_method(D_METHOD("get_material"), &Rope3D::get_material);

	ADD_GROUP("Collision", "collision_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_GROUP("Rope", "rope_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_end", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Rope3D"), "set_rope_end", "get_rope_end");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_width"), "set_width", "get_width");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::FLOAT, "rope_segment_length"), "set_segment_length", "get_segment_length");

	ADD_GROUP("Geometry", "geometry_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::OBJECT, "geometry_material", PROPERTY_HINT_RESOURCE_TYPE, "Material"), "set_material", "get_material");
}

Rope3D::Rope3D() {
	collision_layer = 1;
	collision_mask = 1;
	segment_length = 0.25;
	width = 0.05;
	is_owner = true;
	is_created = false;
}

Rope3D::~Rope3D() {
}

void Rope3D::_ready() {
	//rope_end = NodePath();
	//material = Ref<Material>(nullptr);
	// Before spawning the mesh, need to figure out which end OWNs the mesh?
	UtilityFunctions::print("start-init");
	if (is_owner && !rope_end.is_empty()) {
		Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
		if (other) {
			other->is_owner = false;
			UtilityFunctions::print("owner");
		} else {
			UtilityFunctions::print("no-end");
		}
	}
	UtilityFunctions::print("end-init");
	set_process(is_owner);
}

void Rope3D::set_rope_end(NodePath value) {
	// Create a bi-directional link between the start and end of the rope.
	if (value.is_empty()) {
		// Clear also the other side of the rope.
		// Also update the other ends rope_end to point to nothing.
		if (!rope_end.is_empty()) {
			Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
			if (other) {
				other->rope_end = NodePath();
			}
			rope_end = NodePath();
		}
		rope_end = value;
	} else if (value != rope_end) {
		// Changing the rope end; must be a Rope3D and other than self.
		// Also update the other ends rope_end to point to this.
		Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(value));
		if (other != this) {
			if (other) {
				other->rope_end = get_path();
				other->width = width;
				other->segment_length = segment_length;
				other->material = material;
			}
			rope_end = value;
		}
	}
	UtilityFunctions::print("set rope end");
}

NodePath Rope3D::get_rope_end() const {
	return rope_end;
}

void Rope3D::set_width(real_t value) {
	width = value;
	Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
	if (other && other != this) {
		other->width = width;
	}
}

real_t Rope3D::get_width() const {
	return width;
}

void Rope3D::set_segment_length(real_t value) {
	segment_length = value;
	Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
	if (other && other != this) {
		other->segment_length = segment_length;
	}
}

real_t Rope3D::get_segment_length() const {
	return segment_length;
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
	Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
	if (other && other != this) {
		other->material = material;
	}
}

Ref<Material> Rope3D::get_material() const {
	return material;
}

RigidBody3D *Rope3D::create_segment(Vector3 origin, Vector3 direction) {
	// Prepare shape.
	CylinderShape3D *collider = memnew(CylinderShape3D);
	collider->set_height(segment_length);
	collider->set_radius(width);

	// Prepare collider.
	CollisionShape3D *shape = memnew(CollisionShape3D);
	shape->set_shape(collider);
	shape->set_rotation_degrees(Vector3(90, 0, 0));

	// Prepare sphysics body.
	RigidBody3D *segment = memnew(RigidBody3D);
	segment->set_as_top_level(true);
	segment->add_child(shape);
	segment->set_mass(2.0);

	// Set properties.
	// TODO: Make these tweakable!
	segment->set_linear_damp(0.0);
	segment->set_angular_damp(50);
	segment->set_collision_layer(collision_layer);
	segment->set_collision_mask(collision_mask);

	// Add under self, as top level object.
	add_child(segment);

	// Align and orient segment.
	segment->look_at_from_position(origin, origin + direction, Vector3(0, 1, 0));

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

PinJoint3D *Rope3D::create_joint(Vector3 local_position, Vector3 direction, Node3D *a, Node3D *b) {
	PinJoint3D *joint = memnew(PinJoint3D);
	joint->set_position(local_position);
	joint->set_node_a(a->get_path());
	joint->set_node_b(b->get_path());
	add_child(joint);
	// TODO: Add book keeping
	//joints.push_back(joint)
	//if (a is Rope3DSegment) {
	//  a.joint = joint
	//}
	return joint;
}

void Rope3D::create_rope() {
	Node3D *target = Object::cast_to<Node3D>(get_node_or_null(rope_end));

	if (!target) {
		return;
	}

	PhysicsBody3D *physics_object = Object::cast_to<PhysicsBody3D>(get_parent());
	PhysicsBody3D *target_physics_object = Object::cast_to<PhysicsBody3D>(target->get_parent());

	if (!physics_object || !target_physics_object) {
		return;
	}

	Vector3 point_a = get_global_transform().origin;
	Vector3 point_b = target->get_global_transform().origin;

	int number_of_segments = Math::ceil(point_a.distance_to(point_b) / segment_length);
	Vector3 dir = point_a.direction_to(point_b);
	float segment_adjusted_length = point_a.distance_to(point_b) / number_of_segments;
	Vector3 segment_step = dir * segment_adjusted_length;

	Vector3 segment_pos = point_a - (segment_step * 0.5);
	Vector3 joint_position = Vector3(0, 0, 0);
	PhysicsBody3D *previous = physics_object;

	tracked_nodes.push_back(this);

	for (int i = 0; i < number_of_segments; i++) {
		segment_pos += segment_step;
		// You need to implement create_segment function
		PhysicsBody3D *segment = create_segment(segment_pos, dir);
		PinJoint3D *joint = create_joint(joint_position, dir, previous, segment);

		// TODO: This needs book keeping
		//if (!joint_a)
		//		joint_a = joint;

		joint_position += segment_step;
		previous = segment;
		tracked_nodes.push_back(segment);
	}

	tracked_nodes.push_back(target);

	// TODO: This needs book keeping
	PinJoint3D *joint_b = create_joint(joint_position, dir, previous, target_physics_object);

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
	/*mesh_instance.material_override = material
		# FIXME: For some reason the translation affects our
		# mesh even when it is toplevel, so shift it to world origin.
	*/
	mesh_instance->set_position(-get_global_transform().origin);
	mesh_instance->set_as_top_level(true);
	mesh_instance->set_material_override(material);
	ArrayMesh *mesh = memnew(ArrayMesh);
	mesh_instance->set_mesh(Ref(mesh));
	add_child(mesh_instance);

	/*
	  // Construct vertices, normals, and UVs
		  PoolVector3Array vertices;
		  vertices.resize(tracking.size() * 2);

		  PoolVector3Array normals;
		  normals.resize(tracking.size() * 2);

		  PoolVector2Array vertex_uvs;
		  vertex_uvs.resize(tracking.size() * 2);

		  // Set UVs
		  Array uvs;
		  uvs.push_back(Vector2(0, 0));
		  uvs.push_back(Vector2(1, 0));
		  uvs.push_back(Vector2(0, 1));
		  uvs.push_back(Vector2(1, 1));
		  for (int i = 0; i < tracking.size() * 2; i++) {
			  vertex_uvs.set(i, uvs[i % 4]);
		  }

		  // Create mesh and mesh instance
		  Transform transform;
		  ArrayMesh *mesh = ArrayMesh::_new();
		  MeshInstance *mesh_instance = MeshInstance::_new();
		  mesh_instance->set_mesh(mesh);
		  // FIXME: Translation affects mesh even when it is toplevel
		  mesh_instance->set_translation(-get_global_transform().get_origin());
		  mesh_instance->set_as_toplevel(true);
		  add_child(mesh_instance);
		*/
}

void Rope3D::_process(double delta) {
	if (!is_created) {
		UtilityFunctions::print("process");
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
			Vector3 position = tracked_nodes[i]->get_global_position();
			Vector3 normal = position.direction_to(camera_position);
			if (i < num_points - 1) {
				direction_vector = position.direction_to(
						tracked_nodes[i + 1]->get_global_position());
			}
			Vector3 orientation = direction_vector.cross(normal);
			Vector3 tangent = direction_vector;
			double sz = width;

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
	} else {
		//	set_process(false);
		//UtilityFunctions::print("No camera, no mesh_instance");
	}
}
