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

void Rope3D::create_segment(Vector3 a, Vector3 b) {
	float segment_length = 10.0;
	float width = 2.0;

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
	segment->look_at_from_position(global_position, global_position + global_direction, up);
	segment->set_collision_layer(collision_layer);
	segment->set_collision_mask(collision_mask);

	// if collision_monitor:
	// 	segment.contact_monitor = true
	// 	segment.contacts_reported = 1
	// 	segment.set_script(segment_script)
	// 	segment.rope = self
	// 	segment.connect("body_entered", segment, '_on_body_entered')
	// 	segment.connect("rope_body_entered", self, '_on_rope_body_entered')

	// return segment
}
