#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

// Project-owned implementation of the measured hierarchical LOD contract.
// Bounds are promoted to double before distance evaluation; LOD distances remain
// float, matching the pinned source's FVector -> float evaluation boundaries.
// Node access is immutable. child_index() explicitly maps each child ordinal;
// Godot preorder left/right node indices must never be treated as an interval.
class ShadowHierarchicalLOD {
public:
	using Point = std::array<double, 3>;
	struct Node {
		Point min;
		Point max;
		uint32_t first = 0;
		uint32_t count = 0;
		uint32_t child_count = 0;
		bool test_occlusion = false;
		bool split_for_occlusion = false;
	};
	struct Parameters {
		std::array<Point, 2> origins;
		std::array<float, 8> min_distance;
		std::array<float, 8> max_distance;
		std::array<uint32_t, 8> min_instances_to_split;
		int lod_count = 0;
		float displacement = 0;
		bool single = true;
		bool multiple = false;
		bool overestimate = false;
	};
	struct Runs {
		std::array<std::vector<uint32_t>, 8> single;
		std::array<std::vector<uint32_t>, 8> multiple;
		std::array<uint32_t, 8> single_count = {};
		std::array<uint32_t, 8> multiple_count = {};
	};

private:
	static double distance(const Point &a, const Point &b) {
		double x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2];
		return std::sqrt(x * x + y * y + z * z);
	}
public:
	static std::array<float, 2> distance_range(const Node &node, const Parameters &parameters) {
		Point center;
		for (int axis = 0; axis < 3; axis++) {
			center[axis] = (node.min[axis] + node.max[axis]) * 0.5;
		}
		float first = distance(center, parameters.origins[0]);
		float second = distance(center, parameters.origins[1]);
		float radius = distance(node.max, node.min) * 0.5;
		return { std::min(first, second) - radius, std::max(first, second) + radius };
	}

public:
	static void narrow(const Node &node, const Parameters &parameters, int &min_lod, int &max_lod) {
		if (min_lod == max_lod) {
			return;
		}
		auto distances = distance_range(node, parameters);
		while (max_lod > min_lod && distances[0] > parameters.max_distance[min_lod]) {
			min_lod++;
		}
		while (max_lod > min_lod && distances[1] < parameters.min_distance[max_lod - 1]) {
			max_lod--;
		}
	}
	static bool can_group(const Node &node, const Parameters &parameters) {
		return distance_range(node, parameters)[1] < parameters.max_distance[parameters.lod_count - 1];
	}
	static void append(std::vector<uint32_t> &runs, uint32_t first, uint32_t last) {
		if (!runs.empty() && runs.back() + 1 == first) {
			runs.back() = last;
		} else {
			runs.push_back(first);
			runs.push_back(last);
		}
	}

	// classify(bounds): -1 outside, 0 intersecting, +1 fully contained.
	// occluded(index) is queried only for nodes explicitly carrying query results.
	template <class NodeAt, class ChildIndex, class Classify, class Occluded>
	static void select(const Parameters &parameters, int index, int min_lod, int max_lod, bool contained,
			NodeAt &node_at, ChildIndex &child_index, Classify &classify, Occluded &occluded, Runs &runs) {
		auto policy = [&](const Node &node, int, int &lo, int &hi) { narrow(node, parameters, lo, hi); };
		select_with_policy(parameters, index, min_lod, max_lod, contained, node_at, child_index, classify, occluded, policy, runs);
	}

	// Godot's edge-error selector is a host policy, preserving its exact float
	// operations instead of algebraically converting them to UE distance planes.
	template <class NodeAt, class ChildIndex, class Classify, class Occluded, class Policy>
	static void select_with_policy(const Parameters &parameters, int index, int min_lod, int max_lod, bool contained,
			NodeAt &node_at, ChildIndex &child_index, Classify &classify, Occluded &occluded, Policy &policy, Runs &runs) {
		Node node = node_at(index);
		for (int axis = 0; axis < 3; axis++) {
			node.min[axis] -= parameters.displacement;
			node.max[axis] += parameters.displacement;
		}
		if (!contained) {
			int relation = classify(node);
			if (relation < 0) {
				return;
			}
			contained = relation > 0;
		}
		if (min_lod != max_lod) {
			policy(node, index, min_lod, max_lod);
			if (min_lod >= parameters.lod_count) {
				return;
			}
		}
		if (node.test_occlusion && occluded(index)) {
			return;
		}
		bool group = node.child_count == 0 || (node.count < parameters.min_instances_to_split[min_lod] && can_group(node, parameters));
		bool split = (!contained || min_lod < max_lod || node.split_for_occlusion) && !group;
		if (split) {
			for (uint32_t child = 0; child < node.child_count; child++) {
				select_with_policy(parameters, child_index(index, child), min_lod, max_lod, contained, node_at, child_index, classify, occluded, policy, runs);
			}
			return;
		}
		max_lod = std::min(max_lod, parameters.lod_count - 1);
		uint32_t last = node.first + node.count - 1;
		if (parameters.single) {
			int lod = parameters.overestimate ? max_lod : min_lod;
			if (lod < 8) {
				append(runs.single[lod], node.first, last);
				runs.single_count[lod] += node.count;
			}
		}
		if (parameters.multiple) {
			for (int lod = min_lod; lod <= max_lod; lod++) {
				if (lod < 8) {
					append(runs.multiple[lod], node.first, last);
					runs.multiple_count[lod] += node.count;
				}
			}
		}
	}
};
