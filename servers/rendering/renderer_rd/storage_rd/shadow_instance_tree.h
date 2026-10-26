#pragma once

#include "core/math/projection.h"
#include "core/math/transform_3d.h"
#include "core/templates/vector.h"
#include <algorithm>
#include <memory>

// Project-owned per-MultiMesh spatial hierarchy. No instance records are reordered.
// Local bounds include all vertex deformation; aggregate is used by scene culling.
class ShadowInstanceTree {
	struct Node {
		AABB bounds;
		int left = -1;
		int right = -1;
		uint32_t from = 0;
		uint32_t count = 0;
		float min_lod_scale = 1; // hierarchical LOD hook
		float max_lod_scale = 1; // hierarchical LOD hook
	};
	AABB local_bounds;
	Vector<Transform3D> transforms;
	Vector<AABB> boxes;
	Vector<uint32_t> order;
	Vector<Node> nodes;
	Vector<float> lod_scales; // hierarchical LOD hook
	bool snapshot = false;
	bool dirty = true;
	bool valid = false;
	uint64_t revision = 1;

	struct BuildJob;
	std::shared_ptr<BuildJob> async_job;
	uint64_t aggregate_revision = 0;
	AABB conservative_aggregate;
	bool aggregate_valid = false;
	void rebuild();


	// BEGIN in-place update state
	Vector<int> update_parents;
	Vector<int> update_leaves;
	void build_update_maps();
	bool update_in_place(uint32_t p_index, const Transform3D &p_transform);
	// END in-place update state

	int build_node(uint32_t p_from, uint32_t p_count) {
		Node node;
		const bool with_lod = lod_enabled(); // hierarchical LOD hook
		node.from = p_from;
		node.count = p_count;
		node.bounds = boxes[order[p_from]];
		if (with_lod) { node.min_lod_scale = node.max_lod_scale = lod_scales[order[p_from]]; } // hierarchical LOD hook
		for (uint32_t i = 1; i < p_count; i++) {
			node.bounds.merge_with(boxes[order[p_from + i]]);
			if (with_lod) { node.min_lod_scale = MIN(node.min_lod_scale, lod_scales[order[p_from + i]]); node.max_lod_scale = MAX(node.max_lod_scale, lod_scales[order[p_from + i]]); } // hierarchical LOD hook
		}
		int index = nodes.size();
		nodes.push_back(node);
		if (p_count > 16) {
			int axis = node.bounds.size.max_axis_index();
			uint32_t *ids = order.ptrw();
			std::sort(ids + p_from, ids + p_from + p_count, [&](uint32_t a, uint32_t b) {
				real_t ca = boxes[a].get_center()[axis];
				real_t cb = boxes[b].get_center()[axis];
				return ca == cb ? a < b : ca < cb;
			});
			node.left = build_node(p_from, p_count / 2);
			node.right = build_node(p_from + p_count / 2, p_count - p_count / 2);
			nodes.write[index] = node;
		}
		return index;
	}

	void rebuild_sync() {
		if (!dirty) {
			return;
		}
		dirty = false;
		valid = false;
		nodes.clear();
		update_parents.clear(); // in-place hook
		update_leaves.clear(); // in-place hook
		if (!requested() || !snapshot || transforms.is_empty()) {
			return;
		}
		boxes.resize(transforms.size());
		const bool with_lod = lod_enabled(); // hierarchical LOD hook
		if (with_lod) { lod_scales.resize(transforms.size()); } // hierarchical LOD hook
		order.resize(transforms.size());
		for (int i = 0; i < transforms.size(); i++) {
			if (!transforms[i].is_finite()) {
				return;
			}
			boxes.write[i] = transforms[i].xform(local_bounds);
			if (!boxes[i].is_finite()) {
				return;
			}
			order.write[i] = i;
			if (with_lod) { lod_scales.write[i] = lod_basis_scale(transforms[i].basis); } // hierarchical LOD hook
		}
		build_node(0, transforms.size());
		valid = true;
		build_update_maps(); // in-place hook
	}

public:
	// BEGIN hierarchical LOD view
	static bool lod_enabled();
	static float lod_basis_scale(const Basis &p_basis);
	const AABB &lod_node_bounds(int p_node) const { return nodes[p_node].bounds; }
	uint32_t lod_node_first(int p_node) const { return nodes[p_node].from; }
	uint32_t lod_node_count(int p_node) const { return nodes[p_node].count; }
	int lod_node_left(int p_node) const { return nodes[p_node].left; }
	int lod_node_right(int p_node) const { return nodes[p_node].right; }
	float lod_node_min_scale(int p_node) const { return nodes[p_node].min_lod_scale; }
	float lod_node_max_scale(int p_node) const { return nodes[p_node].max_lod_scale; }
	uint32_t lod_original_id(uint32_t p_sorted) const { return order[p_sorted]; }
	// END hierarchical LOD view
	bool get_conservative_aggregate(AABB &r_bounds);
	bool requested() const { return local_bounds != AABB(); }
	const AABB &get_local_bounds() const { return local_bounds; }
	uint64_t get_revision() const { return revision; }
	void reset() {
		revision++;
		local_bounds = AABB();
		transforms.clear();
		lod_scales.clear(); // hierarchical LOD hook
		boxes.clear();
		order.clear();
		nodes.clear();
		update_parents.clear(); // in-place hook
		update_leaves.clear(); // in-place hook
		snapshot = false;
		dirty = true;
		valid = false;
	}
	void set_bounds(const AABB &p_bounds) {
		revision++;
		local_bounds = p_bounds;
		dirty = true;
	}
	void set_buffer(const float *p_data, uint32_t p_count, uint32_t p_stride) {
		if (!requested()) {
			return;
		}
		revision++;
		transforms.resize(p_count);
		for (uint32_t i = 0; i < p_count; i++) {
			Transform3D &t = transforms.write[i];
			const float *record = p_data + i * p_stride;
			for (int row = 0; row < 3; row++) {
				for (int col = 0; col < 3; col++) {
					t.basis.rows[row][col] = record[row * 4 + col];
				}
				t.origin[row] = record[row * 4 + 3];
			}
		}
		snapshot = true;
		dirty = true;
	}
	bool has_snapshot(uint32_t p_count) const { return snapshot && transforms.size() == int(p_count); }
	void set_transform(uint32_t p_index, const Transform3D &p_transform) {
		if (snapshot && p_index < uint32_t(transforms.size())) {
			// BEGIN in-place update dispatch
			if (update_in_place(p_index, p_transform)) {
				return;
			}
			// END in-place update dispatch
			revision++;
			transforms.write[p_index] = p_transform;
			dirty = true;
		}
	}
	bool active() { rebuild(); return valid && !dirty; }
	uint32_t get_transform_count() const { return transforms.size(); }
	const Transform3D &get_transform(uint32_t p_index) const { return transforms[p_index]; }
	void set_transforms(const Vector<Transform3D> &p_transforms) { revision++; transforms = p_transforms; snapshot = true; dirty = true; }
	const Transform3D &get_first_transform() const { return transforms[0]; }
	AABB get_aggregate() { rebuild(); return valid ? nodes[0].bounds : AABB(); }

	// An entirely contained batch cannot shed any instances. Keep its native
	// direct draw, avoiding GPU compaction, copies and indirect vertex remapping.
	// The margin is stricter than the fine test; uncertain boundaries still cull.
	bool fully_inside(const Projection &p_projection, bool p_pancake) {
		if (!active()) {
			return false;
		}
		for (int corner = 0; corner < 8; corner++) {
			Vector3 point = nodes[0].bounds.get_endpoint(corner);
			Vector4 clip = p_projection.xform(Vector4(point.x, point.y, point.z, 1));
			if (!clip.is_finite()) {
				return false;
			}
			real_t edge = clip.w - 4e-5 * MAX(real_t(1), Math::abs(clip.w));
			if (Math::abs(clip.x) >= edge || Math::abs(clip.y) >= edge || clip.z >= edge || (!p_pancake && -clip.z >= edge)) {
				return false;
			}
		}
		return true;
	}

	// p_projection maps MultiMesh-local positions into uncorrected shadow clip
	// space. Coarse guard dominates the fine shader guard for any box corner.
	Vector<uint32_t> candidates(const Projection &p_projection, uint32_t p_visible, bool p_pancake) {
		Vector<uint32_t> result;
		if (!active()) {
			return result;
		}
		Plane planes[6];
		real_t lengths[6];
		Vector4 w(p_projection[0][3], p_projection[1][3], p_projection[2][3], p_projection[3][3]);
		const int axes[6] = { 2, 2, 0, 1, 0, 1 };
		const int signs[6] = { 1, -1, 1, -1, -1, 1 };
		for (int p = 0; p < 6; p++) {
			planes[p] = p_projection.get_projection_plane(Projection::Planes(p));
			Vector3 normal(w.x, w.y, w.z);
			normal += Vector3(p_projection[0][axes[p]], p_projection[1][axes[p]], p_projection[2][axes[p]]) * signs[p];
			lengths[p] = normal.length();
		}
		Vector<int> stack;
		stack.push_back(0);
		while (!stack.is_empty()) {
			int index = stack[stack.size() - 1];
			stack.resize(stack.size() - 1);
			const Node &node = nodes[index];
			Vector3 center = node.bounds.get_center();
			Vector3 extent = node.bounds.size * 0.5;
			real_t max_w = Math::abs(w.dot(Vector4(center.x, center.y, center.z, 1))) + Vector3(w.x, w.y, w.z).abs().dot(extent);
			bool outside = false;
			for (int p = 0; p < 6; p++) {
				if ((p == Projection::PLANE_NEAR && p_pancake) || lengths[p] <= 0) {
					continue;
				}
				real_t guard = 2e-5 * MAX(real_t(1), max_w) / lengths[p];
				if (planes[p].distance_to(center) - planes[p].normal.abs().dot(extent) > guard) {
					outside = true;
					break;
				}
			}
			if (outside) {
				continue;
			}
			if (node.left < 0) {
				for (uint32_t i = 0; i < node.count; i++) {
					uint32_t id = order[node.from + i];
					if (id < p_visible) {
						result.push_back(id);
					}
				}
			} else {
				stack.push_back(node.right);
				stack.push_back(node.left);
			}
		}
		return result;
	}
};
