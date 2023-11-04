#include "rope3d.h"

using namespace godot;

void Rope3D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_collision_layer", "value"), &Rope3D::set_collision_layer);
	ClassDB::bind_method(D_METHOD("get_collision_layer"), &Rope3D::get_collision_layer);

	ClassDB::bind_method(D_METHOD("set_collision_mask", "value"), &Rope3D::set_collision_mask);
	ClassDB::bind_method(D_METHOD("get_collision_mask"), &Rope3D::get_collision_mask);
	
	ADD_GROUP("Collision", "collision_");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_layer", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_layer", "get_collision_layer");
	ClassDB::add_property("Rope3D", PropertyInfo(Variant::INT, "collision_mask", PROPERTY_HINT_LAYERS_3D_PHYSICS), "set_collision_mask", "get_collision_mask");
}

Rope3D::Rope3D() {
	collision_layer = 1;
	collision_mask = 1;
}

Rope3D::~Rope3D() {
}

void Rope3D::_ready() {
}

void Rope3D::_process(double delta) {
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
