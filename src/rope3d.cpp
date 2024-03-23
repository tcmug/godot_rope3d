#include "rope3d.hpp"

using namespace godot;

void Rope3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_collision_layer", "value"), &Rope3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Rope3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "value"), &Rope3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Rope3D::get_collision_mask);

	ClassDB::bind_method(D_METHOD("set_rope_end", "value"), &Rope3D::set_rope_end);
	ClassDB::bind_method(D_METHOD("get_rope_end"), &Rope3D::get_rope_end);

	ADD_GROUP("Collision", "collision_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");

	ADD_GROUP("Rope", "rope_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::NODE_PATH, "rope_end", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Rope3D"), "set_rope_end", "get_rope_end");
}

Rope3D::Rope3D() {
	collision_layer = 1;
	collision_mask = 1;
	segment_length = 0.2;
	width = 0.05;
	is_owner = true;
}

Rope3D::~Rope3D() {
}

void Rope3D::_ready() {
	// Before spawning the mesh, need to figure out which end OWNs the mesh?
	if (is_owner && !rope_end.is_empty()) {
		Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(rope_end));
		if (other) {
			other->is_owner = false;
			// create_segment(Vector3(), Vector3());
			// Create segments and links between global pos and others global pos.
		}
	}

	create_rope();
	set_process(is_owner);
}

void Rope3D::_process(double delta) {
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
		}
		rope_end = value;
	} else if (value != rope_end) {
		// Changing the rope end; must be a Rope3D and other than self.
		// Also update the other ends rope_end to point to this.
		Rope3D *other = dynamic_cast<Rope3D *>(get_node_or_null(value));
		if (other != this) {
			if (other) {
				other->rope_end = get_path();
			}
			rope_end = value;
		}
	}
}

NodePath Rope3D::get_rope_end() const {
	return rope_end;
}

// void Rope3D::set_rope_path(NodePath value) {
// 	rope_path = value;
// }

// NodePath Rope3D::get_rope_path() const {
// 	return rope_path;
// }

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

#include <godot_cpp/classes/collision_shape3d.hpp>
#include <godot_cpp/classes/cylinder_shape3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>

RigidBody3D *Rope3D::create_segment(Vector3 a, Vector3 b) {
	CylinderShape3D *collider = memnew(CylinderShape3D);
	collider->set_height(segment_length);
	collider->set_radius(width);

	CollisionShape3D *shape = memnew(CollisionShape3D);
	shape->set_shape(collider);
	shape->set_rotation_degrees(Vector3(90, 0, 0));

	RigidBody3D *segment = memnew(RigidBody3D);
	segment->set_as_top_level(true);
	segment->add_child(shape);
	segment->set_mass(2.0);

	// TODO: Make these tweakable!
	segment->set_linear_damp(0.0);
	segment->set_angular_damp(50);

	Vector3 up = Vector3(0, 1, 0);
	Vector3 global_position = get_global_position();
	Vector3 global_direction = Vector3(1, 0, 0);

	this->add_child(segment);
	segment->look_at_from_position(a, a + b, up);
	segment->set_collision_layer(collision_layer);
	segment->set_collision_mask(collision_mask);

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

	//tracking.push_back(get_parent());

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
		//tracking.push_back(segment);
	}

	//tracking.push_back(target);

	// TODO: This needs book keeping
	PinJoint3D *joint_b = create_joint(joint_position, dir, previous, target_physics_object);

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
