# Jolt soft-body test data

`volume_physics_input.json` contains the pressure-shell geometry used by
`extended volume fixture construction` in
`modules/jolt_physics/tests/test_soft_body_remaining.h`.

This extended test is skipped by default. Run it explicitly with a test-enabled
build (substitute the binary name for your platform):

```sh
bin/godot.macos.editor.dev.arm64.stable472 --headless --test \
  '--test-suite=[JoltSoftBodyRemaining]' \
  '--test-case=extended volume fixture construction' --no-skip
```

The default suite's `remaining V13 compression response` uses a minimal tetrahedron
and does not load this file.

The fixture retains the original 387 vertices, 768 tetrahedra, 194 fixed vertex
indices, pressure and damping values. Coordinates and index order are unchanged.
It exercises multi-cell construction, boundary orientation, rest volume, masses
and parameter propagation. It checks construction only, not large-body dynamics.

`boundary_face_count` is the original expected boundary triangle count (768).
The test derives boundary faces through the production code and checks their
count, orientation and enclosed volume. It does not consume a reference triangle
list, so that list is omitted. Unused cloth bodies and unused demo fields are
also omitted.

Each vertex or tetrahedron occupies one line to keep the data reviewable.
This fixture is test input, not a complete demo scene or a claim of general
volumetric-material stability.
