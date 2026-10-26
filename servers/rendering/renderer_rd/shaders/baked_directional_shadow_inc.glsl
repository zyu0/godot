// Fixed-sun static visibility, evaluated by native directional lighting.
// All projection/reference math is float even in Mobile's half-lighting variant.
float sample_baked_directional_shadow(uint index, vec3 view_position, uint layer_mask) {
	DirectionalLightData light = directional_lights.data[index];
	if (light.baked_flags.x == 0u) {
		return 1.0;
	}
	vec3 world = (light.baked_view_to_world * vec4(view_position, 1.0)).xyz;
	vec2 ground_uv = (world.xz - light.baked_coverage.xy) / light.baked_coverage.zw;
	if (any(lessThan(ground_uv, vec2(0.0))) || any(greaterThan(ground_uv, vec2(1.0)))) {
		return 1.0;
	}
	float blend = light.baked_options.w > light.baked_options.z
			? smoothstep(light.baked_options.z, light.baked_options.w, -view_position.z)
			: 1.0;
	if (blend == 0.0) {
		return 1.0;
	}
	// Final ground visibility never runs a depth comparison or blocker search.
	if ((layer_mask & light.baked_flags.y) != 0u) {
		return mix(1.0, texture(sampler2D(baked_directional_visibility[index], DEFAULT_SAMPLER_LINEAR_WITH_MIPMAPS_CLAMP), ground_uv).r, blend);
	}
	vec3 pos = (light.baked_view_to_light * vec4(view_position, 1.0)).xyz;
	vec2 uv = vec2(pos.x / light.baked_projection.x + 0.5, 0.5 - pos.y / light.baked_projection.x);
	float forward_depth = (-pos.z - light.baked_projection.y - light.baked_options.x) / light.baked_projection.z;
	// Strict GREATER and quantized hardware references need a rounding margin.
	// 3/4 quantum lights equality while keeping the next D16 depth occluded.
	float reference = 1.0 - forward_depth + (0.75 / 65535.0);
	float spacing = light.baked_options.y * light.baked_projection.w;
	float border = spacing + light.baked_projection.w;
	if (forward_depth < 0.0 || forward_depth > 1.0 || any(lessThan(uv, vec2(border))) || any(greaterThan(uv, vec2(1.0 - border)))) {
		return 1.0;
	}
	float visibility;
	if (spacing == 0.0) {
		visibility = texture(sampler2DShadow(baked_directional_depth[index], shadow_sampler), vec3(uv, reference));
	} else {
		// Four hardware bilinear comparisons; fixed work, independent of scene size.
		visibility = texture(sampler2DShadow(baked_directional_depth[index], shadow_sampler), vec3(uv + vec2(-spacing, -spacing), reference));
		visibility += texture(sampler2DShadow(baked_directional_depth[index], shadow_sampler), vec3(uv + vec2(spacing, -spacing), reference));
		visibility += texture(sampler2DShadow(baked_directional_depth[index], shadow_sampler), vec3(uv + vec2(-spacing, spacing), reference));
		visibility += texture(sampler2DShadow(baked_directional_depth[index], shadow_sampler), vec3(uv + vec2(spacing, spacing), reference));
		visibility *= 0.25;
	}
	return mix(1.0, visibility, blend);
}
