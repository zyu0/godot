#pragma once

#include "core/templates/hash_map.h"
#include "servers/rendering/renderer_rd/shaders/forward_mobile/shadow_instance_cull.glsl.gen.h"
#include "servers/rendering/renderer_rd/storage_rd/shadow_instance_tree.h"

namespace RendererRD {
class ShadowInstanceCulling {
public:
	struct Draw {
		RID transforms;
		RID commands;
		uint32_t command_offset = 0;
		uint32_t remap_base = 0;
		uint32_t original_id_base = 0;
		uint32_t merged_sources = 0;
		// 1: outside caster volume, 2: no clip candidates, 3: singleton direct, 4: singleton OBB outside.
		uint32_t cpu_decision = 0;
		uint32_t geometry_count = 0;
		bool skip() const { return cpu_decision == 1 || cpu_decision == 2 || cpu_decision == 4 || cpu_decision == 5; }
	};
	struct LODBucket {
		uint32_t surface = 0;
		uint32_t lod = 0;
		Vector<uint32_t> instances;
	};
	struct LODPartition {
		uint32_t surfaces = 0;
		Vector<LODBucket> buckets;
	};
	struct LODDraw {
		uint32_t lod = 0;
		Draw draw;
	};
	struct SourceSlice {
		RID buffer;
		ShadowInstanceTree *tree = nullptr;
		uint32_t count = 0;
		uint32_t offset = 0;
	};
	enum MergeStatus { KEEP_ORIGINAL, REJECT_ALL, READY };
	struct MergeResult {
		MergeStatus status = KEEP_ORIGINAL;
		Vector<Draw> draws;
	};
private:
	struct MergeSourceKey {
		RID buffer;
		uint64_t revision = 0;
		uint32_t count = 0;
		uint32_t offset = 0;
	};
	struct CachedMerge {
		Vector<MergeSourceKey> sources;
		ShadowInstanceTree tree;
		Vector<uint32_t> original_ids;
		uint64_t last_used_frame = 0;
	};
	HashMap<uint32_t, CachedMerge> merge_cache;
	uint32_t merge_cache_instances = 0;
	uint64_t merge_cache_prune_frame = UINT64_MAX;
	struct Job {
		RID output;
		RID gathered;
		RID commands;
		uint32_t output_bytes = 0;
		uint32_t command_bytes = 0;
	};
	struct Pool {
		Vector<Job> jobs;
		uint32_t used = 0;
	};
	struct PushConstant {
		float projection[16];
		float center[4];
		float extent[4];
		uint32_t counts[4];
		uint32_t options[4];
	};
	static_assert(sizeof(PushConstant) == 128);
	ShadowInstanceCullShaderRD shader;
	RID version;
	RID pipeline;
	Vector<Pool> pools;
	uint64_t frame = UINT64_MAX;
	uint32_t current_pool = 0;
	bool initialize();
	void free_job(Job &p_job);
public:
	Vector<Draw> prepare(RID p_source, RID p_scene_shader, const Projection &p_projection,
			ShadowInstanceTree &p_tree, uint32_t p_instances, uint32_t p_source_offset,
			uint32_t p_vec4_stride, const Vector<uint32_t> &p_draw_counts, bool p_pancake, const Vector<Plane> &p_caster_planes, const Vector<SourceSlice> *p_sources = nullptr, const Vector<uint32_t> *p_original_ids = nullptr, bool p_merge_cache_hit = false, bool *r_needs_gpu = nullptr, const LODPartition *p_lods = nullptr);
	MergeResult prepare_merged(const Vector<SourceSlice> &p_sources, RID p_scene_shader, const Projection &p_projection, uint32_t p_vec4_stride, const Vector<uint32_t> &p_draw_counts, bool p_pancake, const Vector<Plane> &p_caster_planes);
	static void trace_merged_submission(const Draw &p_draw);
	static void trace_pipeline(bool p_shadow, bool p_uber);
	static void trace_lod_submission(const LODDraw &p_draw);
	static void trace_cpu_draw(const Draw &p_draw, uint32_t p_instances);
	~ShadowInstanceCulling();
};
}
