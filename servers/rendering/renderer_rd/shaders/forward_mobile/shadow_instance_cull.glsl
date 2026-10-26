#[compute]
#version 450

#VERSION_DEFINES
// Experimental per-shadow-pass output. Source remains untouched; output keeps
// original transform/color/custom records plus a compact original-ID list.
layout(local_size_x = 64) in;
layout(set = 0, binding = 0, std430) readonly buffer Source { uvec4 record[]; } source_data;
layout(set = 0, binding = 1, std430) writeonly buffer Output { uint word[]; } output_data;
// Commands first (5 words per draw), then 4-word-aligned candidates and planes.
// Plane header.x is the float count; the remaining header words are padding.
layout(set = 0, binding = 2, std430) buffer Control { uint word[]; } control_data;
layout(push_constant, std430) uniform Parameters {
 mat4 model_to_clip;
 vec4 bounds_center;
 vec4 bounds_extent;
 uvec4 counts; // instances, source instance offset, vec4 stride, draw commands
 uvec4 options; // original-ID word offset, skip near plane, candidate count, use candidates
} params;

void main() {
 uint work = gl_GlobalInvocationID.x;
 uint work_count = params.options.w != 0u ? params.options.z : params.counts.x;
 if (work >= work_count) return;
 bool lod_partition = params.options.w == 3u;
 uint layout_base = (params.counts.w * 5u + 3u) & ~3u;
 uint surfaces = lod_partition ? control_data.word[layout_base] : 0u;
 uint candidate_stride = lod_partition ? 1u + surfaces : (params.options.w == 2u ? 2u : 1u);
 uint candidate_base = layout_base + (lod_partition ? 4u : 0u);
 uint lod_base = (candidate_base + max(1u, params.options.z) * candidate_stride + 3u) & ~3u;
 uint plane_base = lod_base + (lod_partition ? params.counts.w * 4u : 0u);
 uint id = params.options.w != 0u ? control_data.word[candidate_base + work * candidate_stride] : work;
 if (id >= params.counts.x) return;
 uint src = (id + params.counts.y) * params.counts.z;
 vec4 row0 = uintBitsToFloat(source_data.record[src]);
 vec4 row1 = uintBitsToFloat(source_data.record[src + 1u]);
 vec4 row2 = uintBitsToFloat(source_data.record[src + 2u]);
 uint outside_all = params.options.y != 0u ? 47u : 63u;
 for (uint corner = 0u; corner < 8u; ++corner) {
  vec3 signs = vec3((corner & 1u) != 0u ? 1.0 : -1.0,
                    (corner & 2u) != 0u ? 1.0 : -1.0,
                    (corner & 4u) != 0u ? 1.0 : -1.0);
  vec4 point = vec4(params.bounds_center.xyz + signs * params.bounds_extent.xyz, 1.0);
  vec4 clip = params.model_to_clip * vec4(dot(row0,point),dot(row1,point),dot(row2,point),1.0);
  // Conservative homogeneous clipping, before renderer depth-range correction.
  // Retain a small boundary guard to avoid rejecting a touching caster.
  float epsilon = 1e-5 * max(1.0, abs(clip.w));
  uint outside = 0u;
  if (clip.x < -clip.w - epsilon) outside |= 1u;
  if (clip.x >  clip.w + epsilon) outside |= 2u;
  if (clip.y < -clip.w - epsilon) outside |= 4u;
  if (clip.y >  clip.w + epsilon) outside |= 8u;
  if (clip.z < -clip.w - epsilon) outside |= 16u;
  if (clip.z >  clip.w + epsilon) outside |= 32u;
  outside_all &= outside;
 }
 if (outside_all != 0u) return;
 // Native receiver-volume planes, transformed into MultiMesh parent space.
 // Project the transformed local bound directly; this is an OBB support test,
 // not main-camera visibility. Off-screen trees that can cast remain eligible.
 vec4 center = vec4(params.bounds_center.xyz, 1.0);
 vec3 transformed_center = vec3(dot(row0, center), dot(row1, center), dot(row2, center));
 for (uint i = 0u; i < uint(uintBitsToFloat(control_data.word[plane_base])); ++i) {
  uint plane_word = plane_base + (i + 1u) * 4u;
  vec4 plane = uintBitsToFloat(uvec4(control_data.word[plane_word], control_data.word[plane_word + 1u], control_data.word[plane_word + 2u], control_data.word[plane_word + 3u]));
  vec3 local_normal = vec3(dot(plane.xyz, vec3(row0.x, row1.x, row2.x)),
                          dot(plane.xyz, vec3(row0.y, row1.y, row2.y)),
                          dot(plane.xyz, vec3(row0.z, row1.z, row2.z)));
  float radius = dot(abs(local_normal), params.bounds_extent.xyz);
  float guard = 1e-5 * (dot(abs(plane.xyz), abs(transformed_center)) + abs(plane.w) + radius + 1.0);
  if (dot(plane.xyz, transformed_center) - plane.w - radius > guard) return;
 }
 // Only surviving records are consumed by the native vertex remap. Preserve
 // arbitrary attribute bit patterns and source offsets without a whole-batch
 // transfer command. Each unique global ID owns a disjoint output record.
 for (uint vector_index = 0u; vector_index < params.counts.z; ++vector_index) {
  uvec4 value = source_data.record[src + vector_index];
  uint dst = (id * params.counts.z + vector_index) * 4u;
  for (uint component = 0u; component < 4u; ++component) output_data.word[dst + component] = value[component];
 }
 if (lod_partition) {
  for (uint surface = 0u; surface < surfaces; ++surface) {
   uint bucket = control_data.word[candidate_base + work * candidate_stride + 1u + surface];
   if (bucket == 0xffffffffu) continue;
   uint slot = atomicAdd(control_data.word[bucket * 5u + 1u], 1u);
   uint base = control_data.word[lod_base + bucket * 4u];
   output_data.word[params.options.x + base + slot] = id;
  }
 } else {
  uint slot = atomicAdd(control_data.word[1], 1u);
  output_data.word[params.options.x + slot] = id;
  if (params.options.w == 2u) output_data.word[params.options.x + params.counts.x + slot] = control_data.word[candidate_base + work * 2u + 1u];
  for (uint draw = 1u; draw < params.counts.w; ++draw) atomicAdd(control_data.word[draw * 5u + 1u], 1u);
 }
}
