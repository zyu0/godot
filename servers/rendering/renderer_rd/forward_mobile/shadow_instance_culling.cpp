#include "shadow_instance_culling.h"

#include "core/os/os.h"
#include "core/io/json.h"
#include "core/object/callable_mp.h"
#include "servers/rendering/renderer_rd/uniform_set_cache_rd.h"

using namespace RendererRD;

// Diagnostic live gate: warm native pipelines without consuming trace budgets.
// Only callers with tracing enabled evaluate this; queued readbacks still finish.
static bool shadow_cull_trace_held() {
	return OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE_HOLD") == "1";
}

// Opt-in CPU wall attribution, accumulated by frame to avoid per-job logging.
// Stages are disjoint. Driver encoding/GPU execution happen later and are not
// measured here. Diagnostic clock calls themselves add overhead.
namespace {
struct ShadowCullCpuProfile {
 uint64_t frame = UINT64_MAX;
 uint64_t usec[7] = {};
 uint32_t calls[7] = {};
 bool enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_CPU_PHASES") == "1";
 void begin_frame() {
  uint64_t next = RD::get_singleton()->get_frames_drawn();
  if (next == frame) { return; }
  if (frame != UINT64_MAX) {
   Dictionary report;
   report["frame"] = int64_t(frame);
   static const char *names[] = { "merge_tree", "query", "control_upload", "gather_record", "bindings_push", "compute_record", "draw_result" };
   for (int i = 0; i < 7; i++) {
    report[String(names[i]) + "_us"] = int64_t(usec[i]);
    report[String(names[i]) + "_calls"] = int64_t(calls[i]);
   }
   print_line("SHADOW_CULL_CPU_PHASES " + JSON::stringify(report));
  }
  frame = next;
  for (int i = 0; i < 7; i++) { usec[i] = 0; calls[i] = 0; }
 }
};
ShadowCullCpuProfile &shadow_cull_cpu_profile() {
 static ShadowCullCpuProfile profile;
 return profile;
}
struct ShadowCullCpuTimer {
 uint64_t started = 0;
 int stage;
 explicit ShadowCullCpuTimer(int p_stage) : stage(p_stage) {
  auto &profile = shadow_cull_cpu_profile();
  if (profile.enabled) { profile.begin_frame(); started = OS::get_singleton()->get_ticks_usec(); }
 }
 void stop() {
  if (!started) { return; }
  auto &profile = shadow_cull_cpu_profile();
  profile.usec[stage] += OS::get_singleton()->get_ticks_usec() - started;
  profile.calls[stage]++;
  started = 0;
 }
 void next(int p_stage) {
  if (!started) { return; }
  stop(); stage = p_stage; started = OS::get_singleton()->get_ticks_usec();
 }
 ~ShadowCullCpuTimer() { stop(); }
};
}

// Async callbacks own their shared Dictionary, not a pointer into the renderer
// or its recyclable resource pool. Safe even if the scene is released meanwhile.
static void shadow_cull_trace_readback(const Vector<uint8_t> &p_bytes, Dictionary p_report, int p_part) {
	if (p_part == 0) {
		const uint32_t *words = reinterpret_cast<const uint32_t *>(p_bytes.ptr());
		Array counts;
		for (int i = 0; i < p_bytes.size() / 20; i++) { counts.push_back(int64_t(words[i * 5 + 1])); }
		p_report["draw_counts"] = counts;
	} else if (p_part == 1) {
		p_report["output_bytes"] = p_bytes;

	} else {
		const float *values = reinterpret_cast<const float *>(p_bytes.ptr());
		Array records;
		for (int i = 0; i < p_bytes.size() / 4; i++) { records.push_back(values[i]); }
		p_report["records"] = records;
		const uint32_t *words = reinterpret_cast<const uint32_t *>(p_bytes.ptr());
		Array record_words;
		for (int i = 0; i < p_bytes.size() / 4; i++) { record_words.push_back(int64_t(words[i])); }
		p_report["source_record_words"] = record_words;
	}
	if (p_report.has("draw_counts") && p_report.has("output_bytes") && p_report.has("records")) {
		Array counts = p_report["draw_counts"];
		Vector<uint8_t> output = p_report["output_bytes"];
		const uint32_t *words = reinterpret_cast<const uint32_t *>(output.ptr());
		uint32_t remap_base = p_report["remap_base"];
		uint32_t capacity = p_report["input"];
		uint32_t count = counts[0];
		Array actual;
		for (uint32_t i = 0; i < MIN(count, capacity); i++) { actual.push_back(int64_t(words[remap_base + i])); }
		p_report["actual_ids"] = actual;
		Array selected_record_words;
		uint32_t record_stride = uint32_t(p_report["stride"]) * 4;
		for (uint32_t i = 0; i < MIN(count, capacity); i++) {
			uint32_t id = words[remap_base + i];
			if (id >= capacity) { continue; } // The independent oracle rejects invalid IDs.
			for (uint32_t word = 0; word < record_stride; word++) { selected_record_words.push_back(int64_t(words[id * record_stride + word])); }
		}
		p_report["selected_record_words"] = selected_record_words;
		if (uint32_t(p_report.get("original_id_base", 0)) != 0) {
			Array local_ids;
			uint32_t base = p_report["original_id_base"];
			for (uint32_t i = 0; i < MIN(count, capacity); i++) { local_ids.push_back(int64_t(words[base + i])); }
			p_report["actual_local_ids"] = local_ids;
		}
		if (p_report.has("lod_buckets")) {
			Array buckets = p_report["lod_buckets"];
			for (int bucket_index = 0; bucket_index < buckets.size(); bucket_index++) {
				Dictionary bucket = buckets[bucket_index];
				Array ids, records;
				uint32_t base = bucket["remap_base"];
				Array source_ids = bucket["ids"];
				uint32_t visible = counts[bucket_index];
				for (uint32_t slot = 0; slot < MIN(visible, uint32_t(source_ids.size())); slot++) {
					uint32_t id = words[base + slot];
					ids.push_back(int64_t(id));
					if (id < capacity) {
						for (uint32_t word = 0; word < record_stride; word++) { records.push_back(int64_t(words[id * record_stride + word])); }
					}
				}
				bucket["actual_ids"] = ids;
				bucket["selected_record_words"] = records;
			}
		}
		p_report.erase("output_bytes");
		print_line("SHADOW_INSTANCE_CULL_TRACE " + JSON::stringify(p_report, "", true, true));
	}
}


// Diagnostic only: correlate tiny repeat-image changes with native pipeline
// specialization. Counts describe selected valid pipelines, not GPU duration.
void ShadowInstanceCulling::trace_merged_submission(const Draw &p_draw) {
	static const bool enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE") == "1";
	static uint32_t count = 0;
	if (!enabled || p_draw.merged_sources < 2 || shadow_cull_trace_held() || count++ >= 4096) { return; }
	Dictionary report;
	report["frame"] = int64_t(RD::get_singleton()->get_frames_drawn());
	report["sources"] = int64_t(p_draw.merged_sources);
	report["geometry_count"] = int64_t(p_draw.geometry_count);
	print_line("SHADOW_INSTANCE_MERGED_SUBMIT " + JSON::stringify(report));
}

void ShadowInstanceCulling::trace_lod_submission(const LODDraw &p_draw) {
	static const bool enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE") == "1";
	static uint32_t count = 0;
	if (!enabled || shadow_cull_trace_held() || count++ >= 4096) { return; }
	Dictionary row;
	row["frame"] = int64_t(RD::get_singleton()->get_frames_drawn());
	row["command_buffer"] = int64_t(p_draw.draw.commands.get_id());
	row["command_offset"] = p_draw.draw.command_offset;
	row["lod"] = p_draw.lod;
	row["indices"] = p_draw.draw.geometry_count;
	row["remap_base"] = p_draw.draw.remap_base;
	print_line("SHADOW_LOD_SUBMIT " + JSON::stringify(row));
}

void ShadowInstanceCulling::trace_pipeline(bool p_shadow, bool p_uber) {
	static const bool enabled = OS::get_singleton()->get_environment("ANGE_MOBILE_PIPELINE_TRACE") == "1";
	if (!enabled) { return; }
	static uint64_t last_frame = UINT64_MAX;
	static uint32_t counts[4] = {};
	static uint32_t lines = 0;
	uint64_t current = RD::get_singleton()->get_frames_drawn();
	if (current != last_frame) {
		if (last_frame != UINT64_MAX && lines++ < 1024) {
			Dictionary report;
			report["frame"] = int64_t(last_frame);
			report["color_specialized"] = counts[0]; report["color_uber"] = counts[1];
			report["shadow_specialized"] = counts[2]; report["shadow_uber"] = counts[3];
			print_line("MOBILE_PIPELINE_TRACE " + JSON::stringify(report));
		}
		for (uint32_t &count : counts) { count = 0; }
		last_frame = current;
	}
	counts[(p_shadow ? 2 : 0) + (p_uber ? 1 : 0)]++;
}

void ShadowInstanceCulling::trace_cpu_draw(const Draw &p_draw, uint32_t p_instances) {
	static const bool enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE") == "1";
	static const uint32_t limit = []() {
		String value = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE_MAX");
		return value.is_valid_int() ? uint32_t(MAX(int64_t(1), MIN(int64_t(4096), value.to_int()))) : 96u;
	}();
	static uint32_t count = 0;
	if (!enabled || !p_draw.cpu_decision || shadow_cull_trace_held() || count++ >= limit) {
		return;
	}
	Dictionary report;
	report["frame"] = int64_t(RD::get_singleton()->get_frames_drawn());
	report["input"] = int64_t(p_instances);
	report["geometry_count"] = int64_t(p_draw.geometry_count);
	report["decision"] = p_draw.cpu_decision == 5 ? "merged_follower" : (p_draw.skip() ? "skip" : "singleton_direct");
	report["reason"] = int64_t(p_draw.cpu_decision);
	print_line("SHADOW_INSTANCE_CULL_CPU_DRAW " + JSON::stringify(report));
}

bool ShadowInstanceCulling::initialize() {
	if (pipeline.is_valid()) {
		return true;
	}
	if (version.is_null()) {
		Vector<String> variants;
		variants.push_back("");
		shader.initialize(variants);
		version = shader.version_create();
	}
	RID shader_rid = shader.version_get_shader(version, 0);
	if (shader_rid.is_null()) {
		return false;
	}
	pipeline = RD::get_singleton()->compute_pipeline_create(shader_rid);
	return pipeline.is_valid();
}

void ShadowInstanceCulling::free_job(Job &p_job) {
	for (RID rid : { p_job.output, p_job.gathered, p_job.commands }) {
		if (rid.is_valid()) {
			RD::get_singleton()->free_rid(rid);
		}
	}
	p_job = Job();
}

ShadowInstanceCulling::~ShadowInstanceCulling() {
	for (Pool &pool : pools) {
		for (Job &job : pool.jobs) {
			free_job(job);
		}
	}
	if (version.is_valid()) {
		shader.version_free(version);
	}
}

Vector<ShadowInstanceCulling::Draw> ShadowInstanceCulling::prepare(RID p_source, RID p_scene_shader, const Projection &p_projection,
		ShadowInstanceTree &p_tree, uint32_t p_instances, uint32_t p_source_offset,
		uint32_t p_vec4_stride, const Vector<uint32_t> &p_draw_counts, bool p_pancake, const Vector<Plane> &p_caster_planes, const Vector<SourceSlice> *p_sources, const Vector<uint32_t> *p_original_ids, bool p_merge_cache_hit, bool *r_needs_gpu, const LODPartition *p_lods) {
	if (r_needs_gpu) {
		*r_needs_gpu = false;
	}
	ShadowCullCpuTimer cpu_timer(1);
	Vector<Draw> draws;
	const Vector<Plane> &caster_planes = p_caster_planes;
	for (const Plane &plane : caster_planes) {
		if (!plane.is_finite()) {
			WARN_PRINT_ONCE("Non-finite shadow caster plane; retaining normal draws.");
			return draws;
		}
	}
	if (p_instances == 0 || p_draw_counts.is_empty() || !p_tree.active()) {
		return draws;
	}
	auto cpu_draws = [&](uint32_t p_decision) {
		Vector<Draw> result;
		for (uint32_t count : p_draw_counts) {
			Draw draw;
			draw.cpu_decision = p_decision;
			draw.geometry_count = count;
			result.push_back(draw);
		}
		return result;
	};
	const bool force_fine = p_sources != nullptr || p_lods != nullptr;
	if (!force_fine) {
		const AABB aggregate = p_tree.get_aggregate();
		const Vector3 center = aggregate.get_center();
		const Vector3 extent = aggregate.size * 0.5;
		for (const Plane &plane : caster_planes) {
			const real_t radius = plane.normal.abs().dot(extent);
			const real_t guard = 4e-5 * (plane.normal.abs().dot(center.abs()) + Math::abs(plane.d) + radius + 1);
			if (plane.distance_to(center) - radius > guard) {
				return cpu_draws(1);
			}
		}
	}
	bool contained = !force_fine && p_tree.fully_inside(p_projection, p_pancake);
	if (contained && !caster_planes.is_empty()) {
		AABB aggregate = p_tree.get_aggregate();
		Vector3 center = aggregate.get_center();
		Vector3 extent = aggregate.size * 0.5;
		for (const Plane &plane : caster_planes) {
			real_t radius = plane.normal.abs().dot(extent);
			real_t guard = 4e-5 * (plane.normal.abs().dot(center.abs()) + Math::abs(plane.d) + radius + 1);
			if (plane.distance_to(center) + radius >= -guard) {
				contained = false;
				break;
			}
		}
	}
	if (contained) {
		return draws;
	}
	Vector<uint32_t> candidates = p_tree.candidates(p_projection, p_instances, p_pancake);
	// An empty conservative query needs no GPU work, including merged/LOD
	// jobs.
	if (candidates.is_empty()) {
		return cpu_draws(2);
	}
	// One visible instance cannot benefit from compaction. Conservative CPU
	// rejection above handles empty batches; uncertain edges keep native draws.
	if (!force_fine && p_instances == 1) {
		// A rotated instance's parent-space AABB can overlap a receiver plane
		// even when its actual OBB is outside. Test the retained first record
		// directly; visible_instance_count=1 always draws original instance 0.
		const Transform3D &transform = p_tree.get_first_transform();
		const AABB bounds = p_tree.get_local_bounds();
		const Vector3 center = transform.xform(bounds.get_center());
		const Vector3 extent = bounds.size * 0.5;
		for (const Plane &plane : caster_planes) {
			const Vector3 normal = transform.basis.transposed().xform(plane.normal);
			const real_t radius = normal.abs().dot(extent);
			const real_t guard = 4e-5 * (plane.normal.abs().dot(center.abs()) + Math::abs(plane.d) + radius + 1);
			if (plane.distance_to(center) - radius > guard) {
				return cpu_draws(4);
			}
		}
		return cpu_draws(3);
	}
	if (r_needs_gpu) {
		*r_needs_gpu = true;
		return draws;
	}
	if (!initialize()) {
		return draws;
	}
	cpu_timer.next(2);
	uint64_t remap_capacity = uint64_t(p_instances) * (p_sources ? 2 : 1);
	Vector<uint32_t> lod_offsets;
	if (p_lods) {
		remap_capacity = 0;
		for (const LODBucket &bucket : p_lods->buckets) {
			lod_offsets.push_back(remap_capacity);
			remap_capacity += bucket.instances.size();
		}
	}
	uint64_t output_size = (uint64_t(p_instances) * p_vec4_stride * 4 + remap_capacity + 3) / 4 * 16;
	uint64_t command_size = uint64_t(p_draw_counts.size()) * 5 * sizeof(uint32_t);
	if (output_size > UINT32_MAX || command_size > UINT32_MAX) {
		WARN_PRINT_ONCE("Shadow instance culling allocation overflow; retaining normal draws.");
		return draws;
	}
	RD *rd = RD::get_singleton();
	uint64_t new_frame = rd->get_frames_drawn();
	if (pools.is_empty()) {
		pools.resize(rd->get_frame_delay() + 1);
	}
	if (new_frame != frame) {
		frame = new_frame;
		current_pool = frame % pools.size();
		Pool &pool = pools.write[current_pool];
		// Unused trailing jobs from the last rotation are retired after GPU delay.
		for (int i = pool.used; i < pool.jobs.size(); i++) {
			free_job(pool.jobs.write[i]);
		}
		pool.jobs.resize(pool.used);
		pool.used = 0;
	}
	Pool &pool = pools.write[current_pool];
	if (pool.used == uint32_t(pool.jobs.size())) {
		pool.jobs.push_back(Job());
	}
	Job &job = pool.jobs.write[pool.used++];
	uint32_t candidate_count = candidates.size();
	if (candidates.is_empty()) {
		candidates.push_back(0);
	}
	// One indirect-capable control buffer: commands, aligned candidate words,
	// then an aligned float4 plane header and planes. Offsets use live counts,
	// never retained pool capacities. Validate before narrowing or allocating.
	const uint64_t layout_base = (uint64_t(p_draw_counts.size()) * 5 + 3) & ~uint64_t(3);
	const uint64_t candidate_base = layout_base + (p_lods ? 4 : 0);
	const uint64_t candidate_stride = p_lods ? 1 + p_lods->surfaces : (p_sources ? 2 : 1);
	const uint64_t candidate_words = uint64_t(candidates.size()) * candidate_stride;
	const uint64_t lod_base = (candidate_base + candidate_words + 3) & ~uint64_t(3);
	const uint64_t plane_base = lod_base + (p_lods ? uint64_t(p_lods->buckets.size()) * 4 : 0);
	const uint64_t plane_words = (uint64_t(caster_planes.size()) + 1) * 4;
	const uint64_t control_size = (plane_base + plane_words) * sizeof(uint32_t);
	if (control_size > UINT32_MAX) {
		WARN_PRINT_ONCE("Shadow instance culling control allocation overflow; retaining normal draws.");
		return draws;
	}
	Vector<uint32_t> candidate_data = candidates;
	Vector<uint8_t> active_lod_buckets;
	if (p_sources) {
		candidate_data.resize(candidates.size() * 2);
		for (int i = 0; i < candidates.size(); i++) {
			candidate_data.write[i * 2] = candidates[i];
			candidate_data.write[i * 2 + 1] = (*p_original_ids)[candidates[i]];
		}
	}
	if (p_lods) {
		active_lod_buckets.resize_initialized(p_lods->buckets.size());
		Vector<uint32_t> bucket_for_instance;
		bucket_for_instance.resize(p_instances * p_lods->surfaces);
		bucket_for_instance.fill(UINT32_MAX);
		for (int bucket = 0; bucket < p_lods->buckets.size(); bucket++) {
			for (uint32_t id : p_lods->buckets[bucket].instances) {
				bucket_for_instance.write[id * p_lods->surfaces + p_lods->buckets[bucket].surface] = bucket;
			}
		}
		candidate_data.resize(candidates.size() * candidate_stride);
		for (int i = 0; i < candidates.size(); i++) {
			candidate_data.write[i * candidate_stride] = candidates[i];
			for (uint32_t surface = 0; surface < p_lods->surfaces; surface++) {
				const uint32_t bucket = bucket_for_instance[candidates[i] * p_lods->surfaces + surface];
				candidate_data.write[i * candidate_stride + 1 + surface] = bucket;
				// The zero-candidate sentinel is not a real source instance.
				if (uint32_t(i) < candidate_count && bucket != UINT32_MAX) {
					active_lod_buckets.write[bucket] = 1;
				}
			}
		}
	}
	Vector<float> plane_data;
	plane_data.resize_initialized(4 * (caster_planes.size() + 1));
	plane_data.write[0] = caster_planes.size();
	for (int i = 0; i < caster_planes.size(); i++) {
		for (int axis = 0; axis < 3; axis++) { plane_data.write[(i + 1) * 4 + axis] = caster_planes[i].normal[axis]; }
		plane_data.write[(i + 1) * 4 + 3] = caster_planes[i].d;
	}
	uint32_t plane_size = plane_data.size() * sizeof(float);
	if (job.output_bytes < output_size || job.command_bytes < control_size || (p_sources && job.gathered.is_null())) {
		free_job(job);
		job.output_bytes = output_size;
		job.command_bytes = control_size;
		job.output = rd->storage_buffer_create(job.output_bytes);
		if (p_sources) { job.gathered = rd->storage_buffer_create(job.output_bytes); }
		job.commands = rd->storage_buffer_create(job.command_bytes, Vector<uint8_t>(), RD::STORAGE_BUFFER_USAGE_DISPATCH_INDIRECT);
	}
	if (job.output.is_null() || job.commands.is_null() || (p_sources && job.gathered.is_null())) {
		free_job(job);
		WARN_PRINT_ONCE("Shadow instance culling resource failure; retaining normal draws.");
		return draws;
	}
	Vector<uint32_t> control;
	control.resize_initialized(control_size / sizeof(uint32_t));
	for (int i = 0; i < p_draw_counts.size(); i++) {
		control.write[i * 5] = p_draw_counts[i];
	}
	if (p_lods) {
		control.write[layout_base] = p_lods->surfaces;
		for (int i = 0; i < lod_offsets.size(); i++) {
			control.write[lod_base + i * 4] = lod_offsets[i];
		}
	}
	memcpy(control.ptrw() + candidate_base, candidate_data.ptr(), candidate_words * sizeof(uint32_t));
	memcpy(control.ptrw() + plane_base, plane_data.ptr(), plane_size);
	if (rd->buffer_update(job.commands, 0, control_size, control.ptr()) != OK) {
		return draws;
	}
	cpu_timer.next(3);
	RID source = p_source;
	uint32_t source_offset = p_source_offset;
	if (p_sources) {
		source = job.gathered;
		source_offset = 0;
		uint32_t destination = 0;
		for (const SourceSlice &slice : *p_sources) {
			if (rd->buffer_copy(slice.buffer, source, uint64_t(slice.offset) * p_vec4_stride * 16, uint64_t(destination) * p_vec4_stride * 16, uint64_t(slice.count) * p_vec4_stride * 16) != OK) {
				return draws;
			}
			destination += slice.count;
		}

	}
	// Fine culling copies only surviving records. Native vertex drawing never
	// reads other slots, so no whole-batch source-to-output transfer is needed.
	cpu_timer.next(4);
	RID shader_rid = shader.version_get_shader(version, 0);
	LocalVector<RD::Uniform> uniforms;
	const RID buffers[3] = { source, job.output, job.commands };
	for (int i = 0; i < 3; i++) {
		RD::Uniform u;
		u.uniform_type = RD::UNIFORM_TYPE_STORAGE_BUFFER;
		u.binding = i;
		u.append_id(buffers[i]);
		uniforms.push_back(u);
	}
	RID set = UniformSetCacheRD::get_singleton()->get_cache_vec(shader_rid, 0, uniforms);
	RD::Uniform transforms;
	transforms.uniform_type = RD::UNIFORM_TYPE_STORAGE_BUFFER;
	transforms.binding = 0;
	transforms.append_id(job.output);
	RID transform_set = UniformSetCacheRD::get_singleton()->get_cache(p_scene_shader, 2, transforms);
	if (set.is_null() || transform_set.is_null()) {
		return draws;
	}
	PushConstant push = {};
	for (int c = 0; c < 4; c++) {
		for (int r = 0; r < 4; r++) {
			push.projection[c * 4 + r] = p_projection[c][r];
		}
	}
	AABB bounds = p_tree.get_local_bounds();
	for (int i = 0; i < 3; i++) {
		push.center[i] = bounds.get_center()[i];
		push.extent[i] = bounds.size[i] * 0.5;
	}
	push.counts[0] = p_instances;
	push.counts[1] = source_offset;
	push.counts[2] = p_vec4_stride;
	push.counts[3] = p_draw_counts.size();
	push.options[0] = p_instances * p_vec4_stride * 4;
	push.options[1] = p_pancake ? 1 : 0;
	push.options[2] = candidate_count;
	push.options[3] = p_lods ? 3 : (p_sources ? 2 : 1);
	cpu_timer.next(5);
	RD::ComputeListID list = rd->compute_list_begin();
	rd->compute_list_bind_compute_pipeline(list, pipeline);
	rd->compute_list_bind_uniform_set(list, set, 0);
	rd->compute_list_set_push_constant(list, &push, sizeof(push));
	rd->compute_list_dispatch(list, MAX(1u, (candidate_count + 63) / 64), 1, 1);
	rd->compute_list_end();
	cpu_timer.next(6);
	// Diagnostic transfer only; asynchronous readback must not flush an open
	// native frame. Disabled completely for performance captures.
	static const bool trace_enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE") == "1";
	static const uint32_t trace_limit = []() {
		String value = OS::get_singleton()->get_environment("ANGE_SHADOW_CULL_TRACE_MAX");
		return value.is_valid_int() ? uint32_t(MAX(int64_t(1), MIN(int64_t(4096), value.to_int()))) : 96u;
	}();
	static uint32_t trace_jobs = 0;
	if (trace_enabled && !shadow_cull_trace_held() && trace_jobs++ < trace_limit) {
		Dictionary report;
		report["frame"] = int64_t(frame);
		report["command_buffer"] = int64_t(job.commands.get_id());
		report["input"] = int64_t(p_instances);
		report["allocated"] = int64_t(p_tree.get_transform_count());
		// The native hierarchy includes hidden records whose GPU upload regions
		// may deliberately be stale. Capture CPU hierarchy inputs separately;
		// only the visible prefix is authoritative on the GPU for this draw.
		Array hierarchy_records;
		for (uint32_t i = 0; i < p_tree.get_transform_count(); i++) {
			const Transform3D &transform = p_tree.get_transform(i);
			for (int row = 0; row < 3; row++) {
				for (int column = 0; column < 3; column++) { hierarchy_records.push_back(transform.basis[row][column]); }
				hierarchy_records.push_back(transform.origin[row]);
			}
		}
		report["hierarchy_records"] = hierarchy_records;
		report["stride"] = int(p_vec4_stride);
		report["source_offset"] = int64_t(source_offset);
		report["pancake"] = p_pancake;
		report["remap_base"] = int64_t(push.options[0]);
		report["original_id_base"] = int64_t(p_sources ? push.options[0] + p_instances : 0);
		report["source_batches"] = p_sources ? p_sources->size() : 1;
		if (p_sources) {
			// Independent current source snapshots: never derive oracle inputs from
			// the cached union/tree/ID map whose invalidation is being verified.
			Array source_snapshots;
			for (const SourceSlice &slice : *p_sources) {
				Dictionary snapshot;
				AABB fresh = slice.tree->get_local_bounds();
				Array fresh_bounds;
				for (int axis = 0; axis < 3; axis++) { fresh_bounds.push_back(fresh.position[axis]); }
				for (int axis = 0; axis < 3; axis++) { fresh_bounds.push_back(fresh.size[axis]); }
				Array fresh_records;
				for (uint32_t i = 0; i < slice.count; i++) {
					const Transform3D &transform = slice.tree->get_transform(i);
					for (int row = 0; row < 3; row++) {
						for (int col = 0; col < 3; col++) { fresh_records.push_back(transform.basis.rows[row][col]); }
						fresh_records.push_back(transform.origin[row]);
					}
				}
				snapshot["bounds"] = fresh_bounds;
				snapshot["records"] = fresh_records;
				snapshot["count"] = int64_t(slice.count);
				snapshot["offset"] = int64_t(slice.offset);
				snapshot["revision"] = int64_t(slice.tree->get_revision());
				snapshot["buffer"] = int64_t(slice.buffer.get_id());
				source_snapshots.push_back(snapshot);
			}
			report["source_snapshots"] = source_snapshots;
			report["merge_cache_hit"] = p_merge_cache_hit;
			report["merge_cache_entries"] = int64_t(merge_cache.size());
			report["merge_cache_instances"] = int64_t(merge_cache_instances);
		}
		if (p_original_ids) {
			Array local_ids;
			for (uint32_t id : *p_original_ids) { local_ids.push_back(int64_t(id)); }
			report["source_local_ids"] = local_ids;
		}
		Array geometry_counts;
		for (uint32_t count : p_draw_counts) { geometry_counts.push_back(int64_t(count)); }
		report["geometry_counts"] = geometry_counts;
		if (p_lods) {
			Array buckets;
			for (int i = 0; i < p_lods->buckets.size(); i++) {
				Dictionary bucket;
				bucket["surface"] = p_lods->buckets[i].surface;
				bucket["lod"] = p_lods->buckets[i].lod;
				bucket["remap_base"] = int64_t(push.options[0] + lod_offsets[i]);
				Array ids;
				for (uint32_t id : p_lods->buckets[i].instances) { ids.push_back(int64_t(id)); }
				bucket["ids"] = ids;
				buckets.push_back(bucket);
			}
			report["lod_buckets"] = buckets;
			report["lod_output_bytes"] = int64_t(output_size);
			report["lod_control_bytes"] = int64_t(control_size);
		}
		Array tight_planes;
		for (int i = 4; i < plane_data.size(); i++) { tight_planes.push_back(plane_data[i]); }
		report["caster_planes"] = tight_planes;
		Array matrix;
		for (float value : push.projection) { matrix.push_back(value); }
		report["projection"] = matrix;
		Array box;
		for (int i = 0; i < 3; i++) { box.push_back(bounds.position[i]); }
		for (int i = 0; i < 3; i++) { box.push_back(bounds.size[i]); }
		report["bounds"] = box;
		Array coarse;
		for (uint32_t i = 0; i < candidate_count; i++) { coarse.push_back(int64_t(candidates[i])); }
		report["candidates"] = coarse;
		rd->buffer_get_data_async(job.commands, callable_mp_static(shadow_cull_trace_readback).bind(report, 0), 0, command_size);
		rd->buffer_get_data_async(job.output, callable_mp_static(shadow_cull_trace_readback).bind(report, 1));
		rd->buffer_get_data_async(source, callable_mp_static(shadow_cull_trace_readback).bind(report, 2), uint64_t(source_offset) * p_vec4_stride * 16, uint64_t(p_instances) * p_vec4_stride * 16);
	}

	for (int i = 0; i < p_draw_counts.size(); i++) {
		Draw draw;
		if (p_lods && !active_lod_buckets[i]) {
			draw.cpu_decision = 2; // No coarse candidates can reach this LOD bucket.
		}
		draw.transforms = transform_set;
		draw.commands = job.commands;
		draw.command_offset = i * 5 * sizeof(uint32_t);
		draw.remap_base = push.options[0] + (p_lods ? lod_offsets[i] : 0);
		draw.original_id_base = p_sources ? push.options[0] + p_instances : 0;
		draw.merged_sources = p_sources ? p_sources->size() : 0;
		draw.geometry_count = p_draw_counts[i];
		draws.push_back(draw);
	}
	return draws;
}

ShadowInstanceCulling::MergeResult ShadowInstanceCulling::prepare_merged(const Vector<SourceSlice> &p_sources, RID p_scene_shader, const Projection &p_projection, uint32_t p_vec4_stride, const Vector<uint32_t> &p_draw_counts, bool p_pancake, const Vector<Plane> &p_caster_planes) {
	ShadowCullCpuTimer cpu_timer(0);
	MergeResult result;
	if (p_sources.size() < 2 || p_draw_counts.is_empty()) { return result; }
	static const bool enabled = OS::get_singleton()->get_environment("ANGE_SHADOW_MERGE_CACHE") != "0";
	Vector<MergeSourceKey> keys;
	uint32_t instance_count = 0;
	uint32_t hash = HASH_MURMUR3_SEED;
	for (const SourceSlice &slice : p_sources) {
		if (!slice.tree || !slice.tree->active() || slice.buffer.is_null() || slice.count == 0 || slice.count > slice.tree->get_transform_count() || uint64_t(instance_count) + slice.count > 65536) { return result; }
		keys.push_back({ slice.buffer, slice.tree->get_revision(), slice.count, slice.offset });
		instance_count += slice.count;
		hash = hash_murmur3_one_64(slice.buffer.get_id(), hash);
		hash = hash_murmur3_one_32(slice.count, hash);
		hash = hash_murmur3_one_32(slice.offset, hash);
	}
	const uint64_t current_frame = RD::get_singleton()->get_frames_drawn();
	if (enabled && merge_cache_prune_frame != current_frame) {
		Vector<uint32_t> expired;
		for (const KeyValue<uint32_t, CachedMerge> &entry : merge_cache) {
			if (current_frame > entry.value.last_used_frame && current_frame - entry.value.last_used_frame > 2) { expired.push_back(entry.key); }
		}
		for (uint32_t key : expired) {
			merge_cache_instances -= merge_cache[key].original_ids.size();
			merge_cache.erase(key);
		}
		merge_cache_prune_frame = current_frame;
	}
	CachedMerge uncached;
	CachedMerge *cached = enabled ? merge_cache.getptr(hash) : nullptr;
	bool hit = cached && cached->sources.size() == keys.size();
	if (hit) {
		for (int i = 0; i < keys.size(); i++) {
			const MergeSourceKey &a = cached->sources[i];
			const MergeSourceKey &b = keys[i];
			if (a.buffer != b.buffer || a.count != b.count || a.offset != b.offset || a.revision != b.revision) { hit = false; break; }
		}
	}
	if (!hit) {
		if (cached) {
			merge_cache_instances -= cached->original_ids.size();
			merge_cache.erase(hash);
		}
		if (enabled) {
			while (!merge_cache.is_empty() && (merge_cache.size() >= 128 || merge_cache_instances + instance_count > 65536)) {
				uint64_t oldest_frame = UINT64_MAX;
				uint32_t oldest_key = 0;
				for (const KeyValue<uint32_t, CachedMerge> &entry : merge_cache) {
					if (entry.value.last_used_frame < oldest_frame) { oldest_frame = entry.value.last_used_frame; oldest_key = entry.key; }
				}
				merge_cache_instances -= merge_cache[oldest_key].original_ids.size();
				merge_cache.erase(oldest_key);
			}
			merge_cache.insert(hash, CachedMerge());
			cached = merge_cache.getptr(hash);
			merge_cache_instances += instance_count;
		} else { cached = &uncached; }
		cached->sources = keys;
		Vector<Transform3D> transforms;
		transforms.resize(instance_count);
		cached->original_ids.resize(instance_count);
		AABB bounds;
		uint32_t at = 0;
		for (const SourceSlice &slice : p_sources) {
			bounds = at == 0 ? slice.tree->get_local_bounds() : bounds.merge(slice.tree->get_local_bounds());
			for (uint32_t i = 0; i < slice.count; i++) {
				transforms.write[at] = slice.tree->get_transform(i);
				cached->original_ids.write[at++] = i;
			}
		}
		cached->tree.set_bounds(bounds);
		cached->tree.set_transforms(transforms);
	}
	cached->last_used_frame = current_frame;
	cpu_timer.stop();
	result.draws = prepare(RID(), p_scene_shader, p_projection, cached->tree, instance_count, 0, p_vec4_stride, p_draw_counts, p_pancake, p_caster_planes, &p_sources, &cached->original_ids, hit);
	if (result.draws.size() != p_draw_counts.size()) { return result; }
	bool rejected = true;
	bool ready = true;
	for (const Draw &draw : result.draws) {
		rejected = rejected && draw.skip();
		ready = ready && draw.commands.is_valid() && draw.transforms.is_valid();
	}
	result.status = rejected ? REJECT_ALL : (ready ? READY : KEEP_ORIGINAL);
	return result;
}
