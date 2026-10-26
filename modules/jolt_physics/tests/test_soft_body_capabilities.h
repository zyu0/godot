/**************************************************************************/
/*  test_soft_body_capabilities.h                                         */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#include "soft_body_cap_test_support.h"

#include "core/object/class_db.h"
#include "servers/physics_3d/physics_server_3d_dummy.h"
#include "servers/physics_3d/physics_server_3d_extension.h"

#include <cmath>

namespace TestJoltSoftBodyCap {

TEST_SUITE("[JoltSoftBodyCap]") {
	/* ---------------------------------------------------------------- protocol */

	TEST_CASE("wrong type errors") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", 5)));
		CHECK(errors.has("PackedVector3Array"));
		CHECK(errors.has("int"));
		errors.clear();
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, 5));
		CHECK(errors.has("Dictionary"));
		CHECK(errors.has("SBREM-TYPE"));

		server->free_rid(body);
	}

	TEST_CASE("jolt lists rod properties") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		const TypedArray<Dictionary> list = server->soft_body_get_extra_property_list(body);
		for (const char *key : { K_CONFIG, K_STATE }) {
			CAPTURE(key);
			const Dictionary entry = find_property(list, key);
			REQUIRE_FALSE(entry.is_empty());
			const bool stored = ((uint32_t)entry["usage"] & PROPERTY_USAGE_STORAGE) != 0;
			CHECK(stored == (String(key) == K_CONFIG));
		}
		for (const char *key : { "rod/joints", "rod/compliance", "rod/bend", "rod/fixed" }) {
			CHECK(find_property(list, key).is_empty());
		}

		server->free_rid(body);
	}

	TEST_CASE("wrapper forwards all three") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());

		// All three calls must pass through the wrapper, not the backend directly.
		CHECK_FALSE(Probe::is_inner_jolt_server(server.ptr()));

		RID body = server->soft_body_create();
		const PackedVector3Array joints = make_joints(9);
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)));

		ErrorCapture errors;
		// Stored keys must serialize before the body enters a space.
		const PackedVector3Array read = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant());
		REQUIRE(read.size() == joints.size());
		for (int i = 0; i < joints.size(); i++) {
			CHECK(read[i] == joints[i]);
		}
		CHECK(server->soft_body_get_extra_property_list(body).size() >= 5);
		CHECK(errors.count() == 0);

		server->free_rid(body);
	}

	TEST_CASE("rod state is valid before stepping") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		// Segments, not particles: 9 joints give 8 rods.
		CHECK(Probe::vertex_count(body) == 9);
		CHECK(Probe::rod_count(body) == 8);
		const PackedFloat32Array state = server->soft_body_get_extra_property(body, K_STATE);
		REQUIRE(state.size() == 8 * 4);
		for (int i = 0; i < state.size(); i++) {
			CHECK(Math::is_finite(state[i]));
		}

		server->free_rid(body);
	}

	TEST_CASE("godot physics ignores rod keys") {
		ServerScope server("GodotPhysics3D");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		CHECK(errors.count() == errors.warning_count());
		CHECK(errors.warning_count() <= 2);
		errors.clear();
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		CHECK(server->soft_body_get_extra_property_list(body).is_empty());
		CHECK(errors.count() == errors.warning_count());
		CHECK(errors.warning_count() <= 1);
		errors.clear();
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		CHECK(server->soft_body_get_extra_property_list(body).is_empty());
		CHECK(errors.count() == 0);

		server->free_rid(body);
	}

	TEST_CASE("extension shims forward") {
		// Without GDExtension overrides, each EXBIND*R call reports one error and
		// returns its type's default. This checks all three bindings in every build.
		PhysicsServer3D *saved = PhysicsServer3D::get_singleton();
		PhysicsServer3DExtension *server = memnew(PhysicsServer3DExtension);
		const RID body;
		{
			ErrorCapture errors;
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_config(9)));
			CHECK(server->soft_body_get_extra_property(body, K_CONFIG).get_type() == Variant::NIL);
			CHECK(server->soft_body_get_extra_property_list(body).is_empty());
			CHECK(errors.count() == 3);
		}
		memdelete(server);
		PhysicsServer3D::set_singleton_for_tests(saved);

		// ClassDB records virtual signatures only under DEBUG_ENABLED.
#ifdef DEBUG_ENABLED
		List<MethodInfo> methods;
		ClassDB::get_virtual_methods("PhysicsServer3DExtension", &methods);

		HashMap<String, int> arg_counts;
		for (const MethodInfo &method : methods) {
			arg_counts[method.name] = method.arguments.size();
		}

		REQUIRE(arg_counts.has("_soft_body_set_extra_property"));
		CHECK(arg_counts["_soft_body_set_extra_property"] == 3);
		REQUIRE(arg_counts.has("_soft_body_get_extra_property"));
		CHECK(arg_counts["_soft_body_get_extra_property"] == 2);
		REQUIRE(arg_counts.has("_soft_body_get_extra_property_list"));
		CHECK(arg_counts["_soft_body_get_extra_property_list"] == 1);
#endif
	}

	/* -------------------------------------------------------------- validation */

	TEST_CASE("one joint is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(1))));
		CHECK(errors.has("SBREM-SIZE"));
		CHECK(errors.has("joint count 1 requires 2..1024"));

		server->free_rid(body);
	}

	TEST_CASE("compliance count mismatch") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", make_floats(7))));
		CHECK_MESSAGE(errors.has("has 7 entries"), errors.joined());
		CHECK_MESSAGE(errors.has("has 8 segments"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("bend count mismatch") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", make_floats(8))));

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "bend", make_floats(8))));
		CHECK_MESSAGE(errors.has("has 8 entries"), errors.joined());
		CHECK_MESSAGE(errors.has("has 7 bend-twist"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("negative fixed is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "fixed", -1)));
		CHECK_MESSAGE(errors.has("[0, 9]"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("over-range fixed is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "fixed", 10)));
		CHECK_MESSAGE(errors.has("[0, 9]"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("zero fixed is a free rod") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9, 0));
		server->soft_body_set_total_mass(body, 1.0);
		server->soft_body_set_space(body, space.rid());
		// Read after a step, so `_update_mass()` has already rewritten every
		// inverse mass at least once.
		space.step();
		REQUIRE(Probe::in_space(body));

		for (int i = 0; i < 9; i++) {
			CHECK(Math::is_equal_approx(Probe::vertex_inv_mass(body, i), 9.0f));
		}

		server->free_rid(body);
	}

	TEST_CASE("fully pinned rod") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9, 9));
		server->soft_body_set_space(body, space.rid());
		space.step();
		REQUIRE(Probe::in_space(body));

		for (int i = 0; i < 9; i++) {
			CHECK(Probe::vertex_inv_mass(body, i) == 0.0f);
		}

		server->free_rid(body);
	}

	TEST_CASE("two joint rod needs no bend") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(2))));
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", make_floats(1))));
		// One segment carries zero bend-twist constraints, not -1 of them.
		CHECK(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "bend", make_floats(0))));

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "bend", make_floats(1))));

		server->free_rid(body);
	}

	TEST_CASE("zero length segment is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;

		SUBCASE("coincident") {
			PackedVector3Array joints = make_joints(3);
			joints.set(1, joints[0]);
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)));
			CHECK_MESSAGE(errors.has("Segment 0"), errors.joined());
		}
		SUBCASE("below the epsilon") {
			PackedVector3Array joints = make_joints(3);
			joints.set(1, joints[0] + Vector3(5e-7, 0, 0));
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)));
			CHECK_MESSAGE(errors.has("Segment 0"), errors.joined());
		}
		SUBCASE("above the epsilon") {
			PackedVector3Array joints = make_joints(3);
			joints.set(1, joints[0] + Vector3(2e-6, 0, 0));
			CHECK(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)));
			CHECK(errors.count() == 0);
		}
		SUBCASE("adjacent representable values around minimum") {
			for (real_t endpoint : { std::nextafter(real_t(1e-6), real_t(0)), std::nextafter(real_t(1e-6), real_t(1)) }) {
				PackedVector3Array joints = make_joints(2, endpoint);
				const bool valid = double(joints[1].length_squared()) >= 1e-12;
				CHECK(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)) == valid);
			}
		}
		SUBCASE("finite coordinates cannot overflow exported float length") {
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(2, real_t(1e20)))));
			CHECK(errors.has("SBREM-VALUE"));
			CHECK(errors.has("squared length"));
		}

		server->free_rid(body);
	}

	TEST_CASE("non finite joints are refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		real_t value = 0.0;
		SUBCASE("nan") {
			value = NAN;
		}
		SUBCASE("inf") {
			value = INFINITY;
		}

		PackedVector3Array joints = make_joints(4);
		joints.set(2, Vector3(value, 0, 0));
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", joints)));
		CHECK_MESSAGE(errors.has("entry 2"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("negative compliance is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));

		PackedFloat32Array compliance = make_floats(8);
		compliance.set(3, -1.0f);

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", compliance)));
		CHECK_MESSAGE(errors.has("entry 3"), errors.joined());
		CHECK_MESSAGE(errors.has("negative"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("non finite compliance is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", make_floats(8))));

		ErrorCapture errors;
		SUBCASE("compliance") {
			PackedFloat32Array values = make_floats(8);
			values.set(2, NAN);
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", values)));
		}
		SUBCASE("bend") {
			PackedFloat32Array values = make_floats(7);
			values.set(2, NAN);
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "bend", values)));
		}
		CHECK_MESSAGE(errors.has("entry 2"), errors.joined());
		CHECK_MESSAGE(errors.has("finite"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("fixed without required joints is rejected") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "fixed", 5)));
		CHECK(errors.has("SBREM-KEY"));
		CHECK(errors.has("joints"));
		CHECK(server->soft_body_get_extra_property(body, K_CONFIG).get_type() == Variant::NIL);
		CHECK(set_valid_rod(server.ptr(), body, 9, 5));
		CHECK((int)Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("fixed", Variant()) == 5);
		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant())).size() == 9);

		server->free_rid(body);
	}

	TEST_CASE("rod topology replacement is atomic") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());
		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		space.step(3);
		RemainingSnapshot before(server.ptr(), body);
		const auto generation = Probe::build_counts(body).generation;

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(3))));
		CHECK_MESSAGE(errors.has("has 8 entries"), errors.joined());
		CHECK_MESSAGE(errors.has("has 2 segments"), errors.joined());
		before.unchanged();
		// A complete consistent topology replacement is atomic and resets live data.
		REQUIRE(set_valid_rod(server.ptr(), body, 3));
		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG))["joints"]).size() == 3);
		CHECK(Probe::vertex_count(body) == 3);
		CHECK(Probe::rod_count(body) == 2);
		CHECK(Probe::build_counts(body).generation == generation + 1);
		CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_STATE)).size() == 8);
		CHECK(Probe::vertex_position(body, 2) == Vector3(2, 0, 0));
		CHECK(Probe::velocity(body, 2) == Vector3());

		server->free_rid(body);
	}

	TEST_CASE("rejected set leaves state untouched") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));

		ErrorCapture errors;
		REQUIRE_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(3))));

		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant())).size() == 9);
		CHECK(PackedFloat32Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("compliance", Variant())).size() == 8);
		CHECK(PackedFloat32Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("bend", Variant())).size() == 7);
		CHECK((int)Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("fixed", Variant()) == 1);
		CHECK(server->soft_body_get_extra_property(body, K_STATE).get_type() == Variant::NIL);

		server->free_rid(body);
	}

	TEST_CASE("re-setting a key replaces it") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9))));
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(4))));

		const PackedVector3Array read = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant());
		CHECK(read.size() == 4);

		server->free_rid(body);
	}

	TEST_CASE("over-long rod is refused") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(1025))));
		CHECK_MESSAGE(errors.has("1024"), errors.joined());

		server->free_rid(body);
	}

	TEST_CASE("a rod at the cap is accepted") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		RID body = server->soft_body_create();

		ErrorCapture errors;
		CHECK(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(1024))));
		CHECK(errors.count() == 0);

		server->free_rid(body);
	}

	TEST_CASE("two joint rod orientation is bounded") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 2));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		// A single segment has no adjacent rod, so it carries no bend-twist
		// constraint at all.
		CHECK(Probe::rod_count(body) == 1);
		CHECK(Probe::rod_bend_twist_count(body) == 0);

		server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
		space.step(60);

		const PackedFloat32Array state = server->soft_body_get_extra_property(body, K_STATE);
		REQUIRE(state.size() == 4);
		for (int i = 0; i < state.size(); i++) {
			CHECK(Math::is_finite(state[i]));
		}

		server->free_rid(body);
	}

	TEST_CASE("fixed without joints does not activate the rod") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());
		RID mesh = make_quad_mesh();

		RID body = server->soft_body_create();
		{
			ErrorCapture errors;
			CHECK_FALSE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "fixed", 1)));
			CHECK(errors.has("SBREM-KEY"));
		}
		server->soft_body_set_mesh(body, mesh);
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		CHECK(Probe::rod_count(body) == 0);
		CHECK(Probe::face_count(body) == 2);

		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		CHECK(Probe::rod_count(body) == 8);
		CHECK((int)Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("fixed", Variant()) == 1);

		server->free_rid(body);
		RenderingServer::get_singleton()->free_rid(mesh);
	}

	/* ------------------------------------------------------------- arbitration */

	TEST_CASE("cloth without a mesh still errors") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();

		ErrorCapture errors;
		server->soft_body_set_space(body, space.rid());
		// Stock behavior: cloth without a mesh is inert and emits no diagnostic.
		CHECK(errors.count() == 0);
		CHECK_FALSE(Probe::in_space(body));

		server->free_rid(body);
	}

	TEST_CASE("cloth regression") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());
		RID mesh = make_quad_mesh();

		RID body = server->soft_body_create();
		server->soft_body_set_mesh(body, mesh);
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		CHECK(Probe::vertex_count(body) == 4);
		CHECK(Probe::face_count(body) == 2);
		CHECK(Probe::edge_count(body) > 0);
		CHECK(Probe::rod_count(body) == 0);

		const Vector3 before = Probe::vertex_position(body, 0);
		server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
		space.step(30);
		CHECK(Probe::vertex_position(body, 0) != before);

		server->free_rid(body);
		RenderingServer::get_singleton()->free_rid(mesh);
	}

	TEST_CASE("mesh with a rod warns") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());
		RID mesh = make_quad_mesh();

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_mesh(body, mesh);

		ErrorCapture errors;
		server->soft_body_set_space(body, space.rid());
		CHECK_MESSAGE(errors.has("mesh is ignored"), errors.joined());
		REQUIRE(Probe::in_space(body));
		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant())).size() == 9);
		CHECK(Probe::rod_count(body) == 8);
		CHECK(Probe::face_count(body) == 0);
		CHECK(Probe::vertex_count(body) == 9);

		server->free_rid(body);
		RenderingServer::get_singleton()->free_rid(mesh);
	}

	/* ------------------------------------------------------------------- traps */

	TEST_CASE("optimize makes the rod simulate") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		const Vector3 before = Probe::vertex_position(body, 8);
		server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
		space.step(30);
		CHECK(Probe::vertex_position(body, 8).distance_to(before) > 0.01);

		// Missing Optimize() leaves no solver update groups, allowing unbounded stretch.
		const real_t span = Probe::vertex_position(body, 0).distance_to(Probe::vertex_position(body, 8));
		CHECK(span > 7.0);
		CHECK(span < 9.0);

		server->free_rid(body);
	}

	TEST_CASE("rod without update position drifts") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		// A rod is anchored to the static world, so the body frame must not chase
		// its own vertices.
		CHECK_FALSE(Probe::update_position(body));

		const Vector3 origin = Probe::center_of_mass(body);
		server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
		space.step(60);

		CHECK(Probe::center_of_mass(body).distance_to(origin) < 0.001);
		// The rest length is untouched by the frame decision.
		CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 1.0f));

		server->free_rid(body);
	}

	TEST_CASE("update position set too early drifts on rebuild") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		space.step();

		// _add_to_space must restore the flag each rebuild; settings do not survive.
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "compliance", make_floats(8, 0.5f))));
		REQUIRE(Probe::in_space(body));
		CHECK_FALSE(Probe::update_position(body));

		const Vector3 origin = Probe::center_of_mass(body);
		server->soft_body_apply_central_impulse(body, Vector3(0, -20, 0));
		space.step(60);
		CHECK(Probe::center_of_mass(body).distance_to(origin) < 0.001);

		server->free_rid(body);
	}

	TEST_CASE("mass update preserves pins") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9, 2));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		// Restore pins after _update_mass() overwrites inverse masses.
		server->soft_body_set_total_mass(body, 5.0);
		space.step();

		CHECK(Probe::vertex_inv_mass(body, 0) == 0.0f);
		CHECK(Probe::vertex_inv_mass(body, 1) == 0.0f);
		CHECK(Probe::vertex_inv_mass(body, 2) > 0.0f);

		server->free_rid(body);
	}

	TEST_CASE("rebuild rederives rod properties") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 1.0f));
		CHECK(Probe::all_bishop_frames_set(body));

		// A reused settings object would short-circuit `CalculateRodProperties`,
		// leaving the previous build's lengths behind.
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, rod_candidate(server.ptr(), body, "joints", make_joints(9, 2.0))));
		REQUIRE(Probe::in_space(body));
		CHECK(Math::is_equal_approx(Probe::rod_length(body, 0), 2.0f));
		CHECK(Probe::all_bishop_frames_set(body));

		server->free_rid(body);
	}

	TEST_CASE("rod state follows source order at scale") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		const int joint_count = 200;
		RID body = server->soft_body_create();
		PackedVector3Array joints;
		joints.push_back(Vector3());
		for (int i = 0; i < joint_count - 1; ++i) {
			joints.push_back(joints[i] + Vector3(1, 0.25 * (i % 3), 0.125 * (i % 5)));
		}
		Dictionary config = rod_config(joint_count);
		config["joints"] = joints;
		REQUIRE(server->soft_body_set_extra_property(body, K_CONFIG, config));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		REQUIRE(Probe::rod_count(body) == joint_count - 1);

		const PackedFloat32Array state = server->soft_body_get_extra_property(body, K_STATE);
		REQUIRE(state.size() == 4 * (joint_count - 1));
		// Public rotations must follow authored segments, not optimizer order.
		for (int i = 0; i < joint_count - 1; i++) {
			const Quaternion rotation(state[i * 4], state[i * 4 + 1], state[i * 4 + 2], state[i * 4 + 3]);
			CHECK(rotation.is_normalized());
			const Vector3 tangent = rotation.xform(Vector3(0, 0, 1));
			CHECK(tangent.distance_to((joints[i + 1] - joints[i]).normalized()) < 0.01);
		}

		server->free_rid(body);
	}

	/* --------------------------------------------------------------- derivation */

	TEST_CASE("one pin frees orientation two pins fix it") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID free_end = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), free_end, 9, 1));
		server->soft_body_set_space(free_end, space.rid());
		REQUIRE(Probe::in_space(free_end));

		RID clamped = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), clamped, 9, 2));
		server->soft_body_set_space(clamped, space.rid());
		REQUIRE(Probe::in_space(clamped));

		const Quaternion clamped_before = read_rod_rotation(server.ptr(), clamped, 0);

		server->soft_body_apply_central_impulse(free_end, Vector3(0, -40, 0));
		server->soft_body_apply_central_impulse(clamped, Vector3(0, -40, 0));
		space.step(60);

		// One fixed joint permits motion; two clamp rod 0. Compare positions:
		// measured swing is 0.014 units, while clamped values stay bit-identical.
		CHECK(Probe::vertex_position(free_end, 1) != Vector3(1, 0, 0));
		CHECK(Probe::vertex_position(clamped, 1) == Vector3(1, 0, 0));

		// Rotation stays bit-identical in both bodies at float precision; use it
		// only for the clamped check. Point impulses require the rod's absent mesh map.
		CHECK(read_rod_rotation(server.ptr(), clamped, 0) == clamped_before);

		server->free_rid(clamped);
		server->free_rid(free_end);
	}

	/* --------------------------------------------------------------- lifecycle */

	TEST_CASE("space change keeps capability state") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		server->soft_body_set_space(body, RID());
		CHECK_FALSE(Probe::in_space(body));

		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));
		CHECK(Probe::rod_count(body) == 8);
		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant())).size() == 9);

		server->free_rid(body);
	}

	TEST_CASE("cloth vertex api rejects a rod") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9, 2));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		ErrorCapture errors;
		// A rod has no mesh, so the mesh-indexed cloth API has nothing to address.
		server->soft_body_move_point(body, 0, Vector3(1, 1, 1));
		CHECK(server->soft_body_get_point_global_position(body, 0) == Vector3());
		server->soft_body_pin_point(body, 0, true);
		CHECK_FALSE(server->soft_body_is_point_pinned(body, 0));
		server->soft_body_apply_point_impulse(body, 0, Vector3(0, 1, 0));
		CHECK(errors.count() > 0);

		space.step();
		CHECK(Probe::vertex_inv_mass(body, 0) == 0.0f);
		CHECK(Probe::vertex_inv_mass(body, 1) == 0.0f);
		CHECK(Probe::vertex_inv_mass(body, 2) > 0.0f);

		server->free_rid(body);
	}

	TEST_CASE("cloth setters do not disturb a rod") {
		ServerScope server("Jolt Physics");
		REQUIRE(server.is_valid());
		SpaceScope space(server.ptr());

		RID body = server->soft_body_create();
		REQUIRE(set_valid_rod(server.ptr(), body, 9));
		server->soft_body_set_space(body, space.rid());
		REQUIRE(Probe::in_space(body));

		const PackedVector3Array joints_before = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant());
		const PackedFloat32Array compliance_before = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("compliance", Variant());
		const PackedFloat32Array bend_before = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("bend", Variant());
		const int fixed_before = Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("fixed", Variant());
		const PackedFloat32Array state_before = server->soft_body_get_extra_property(body, K_STATE);

		const auto generation = Probe::build_counts(body).generation;
		// Valid typed writes preserve author config and rebuild derived live data.
		server->soft_body_set_simulation_precision(body, 7);
		server->soft_body_set_total_mass(body, 3.0);
		server->soft_body_set_linear_stiffness(body, 0.25);
		server->soft_body_set_shrinking_factor(body, 0.5);
		server->soft_body_set_pressure_coefficient(body, 2.0);
		server->soft_body_set_damping_coefficient(body, 0.75);

		CHECK(server->soft_body_get_simulation_precision(body) == 7);
		CHECK(Math::is_equal_approx(server->soft_body_get_total_mass(body), (real_t)3.0));
		CHECK(Math::is_equal_approx(server->soft_body_get_linear_stiffness(body), (real_t)0.25));
		CHECK(Math::is_equal_approx(server->soft_body_get_shrinking_factor(body), (real_t)0.5));
		CHECK(Math::is_equal_approx(server->soft_body_get_pressure_coefficient(body), (real_t)2.0));
		CHECK(Math::is_equal_approx(server->soft_body_get_damping_coefficient(body), (real_t)0.75));

		CHECK(PackedVector3Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("joints", Variant())) == joints_before);
		CHECK(PackedFloat32Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("compliance", Variant())) == compliance_before);
		CHECK(PackedFloat32Array(Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("bend", Variant())) == bend_before);
		CHECK((int)Dictionary(server->soft_body_get_extra_property(body, K_CONFIG)).get("fixed", Variant()) == fixed_before);
		CHECK(PackedFloat32Array(server->soft_body_get_extra_property(body, K_STATE)) == state_before);
		CHECK(Probe::build_counts(body).generation > generation);
		CHECK_FALSE(Probe::update_position(body));

		server->free_rid(body);
	}

	TEST_CASE("cached servers are released and can be recreated") {
		ServerScope::clear_cache();
		struct Cleanup {
			PhysicsServer3D *saved = PhysicsServer3D::get_singleton();
			PhysicsServer3DDummy *sentinel = memnew(PhysicsServer3DDummy);
			~Cleanup() {
				ServerScope::clear_cache();
				memdelete(sentinel);
				PhysicsServer3D::set_singleton_for_tests(saved);
			}
		} cleanup;
		const int baseline = ObjectDB::get_object_count();
		const char *names[] = { "Jolt Physics", "GodotPhysics3D" };
		for (int round = 0; round < 2; ++round) {
			CAPTURE(round);
			ObjectID ids[2];
			for (int i = 0; i < 2; ++i) {
				const char *name = names[round == 0 ? i : 1 - i];
				{
					ServerScope server(name);
					REQUIRE(server.is_valid());
					ids[i] = server->get_instance_id();
					if (String(name) == "Jolt Physics") {
						SpaceScope space(server.ptr());
						VolumeBodyScope body(server.ptr(), space.rid(), fixture_tetra());
						space.step(2);
					}
				}
				CHECK(PhysicsServer3D::get_singleton() == cleanup.sentinel);
				{
					ServerScope reused(name);
					REQUIRE(reused.is_valid());
					CHECK(reused->get_instance_id() == ids[i]);
				}
			}
			// Each cached backend has one wrapper and one inner server.
			CHECK(ObjectDB::get_object_count() == baseline + 4);
			ServerScope::clear_cache();
			CHECK(ObjectDB::get_object_count() == baseline);
			CHECK(PhysicsServer3D::get_singleton() == cleanup.sentinel);
			CHECK(Probe::current_jolt_server() == nullptr);
			for (ObjectID id : ids) {
				CHECK(ObjectDB::get_instance(id) == nullptr);
			}
			ServerScope::clear_cache();
			CHECK(ObjectDB::get_object_count() == baseline);
			CHECK(PhysicsServer3D::get_singleton() == cleanup.sentinel);
		}
	}

} // TEST_SUITE

} // namespace TestJoltSoftBodyCap
