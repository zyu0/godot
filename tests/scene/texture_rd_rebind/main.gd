extends SceneTree
# Run with a real RD renderer: --path tests/scene/texture_rd_rebind --script res://main.gd
# No SceneTree dummy RenderingServer: the shared Metal/Vulkan texture path is required.

func _initialize():
    RenderingServer.call_on_render_thread(prepare)

func prepare():
    var rd = RenderingServer.get_rendering_device()
    if rd == null:
        finish.call_deferred({"failures": ["A real RenderingDevice is required"]})
        return
    var format = RDTextureFormat.new()
    format.width = 8
    format.height = 4
    format.format = RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM
    format.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
    var a = rd.texture_create(format, RDTextureView.new())
    format.width = 16
    format.height = 8
    var b = rd.texture_create(format, RDTextureView.new())
    var texture = Texture2DRD.new()
    var state = {"changes": 0, "failures": []}
    texture.changed.connect(func(): state.changes += 1)
    queue_rebind.call_deferred(texture, a, b, state)

func queue_rebind(texture, a, b, state):
    # The first A may still be queued when the last A is requested. An eager
    # caller-thread equality guard must not drop that last request.
    texture.texture_rd_rid = a
    texture.texture_rd_rid = a
    texture.texture_rd_rid = b
    texture.texture_rd_rid = a
    var orphan = Texture2DRD.new()
    state.orphan_changes = 0
    orphan.changed.connect(func(): state.orphan_changes += 1)
    orphan.texture_rd_rid = a
    orphan.texture_rd_rid = RID()
    orphan = null
    RenderingServer.call_on_render_thread(verify.bind(texture, a, b, state))

func verify(texture, a, b, state):
    var rd = RenderingServer.get_rendering_device()
    if state.orphan_changes != 2:
        state.failures.append("queued clear lost the last resource reference")
    if texture.texture_rd_rid != a or texture.get_width() != 8 or texture.get_height() != 4:
        state.failures.append("queued A B A did not end at A")
    if state.changes != 3:
        state.failures.append("duplicate A rebuilt or notified: %s" % state.changes)
    var alias = texture.get_rid()
    var bytes = PackedByteArray()
    bytes.resize(8 * 4 * 4)
    bytes.fill(255)
    rd.texture_update(a, 0, bytes)
    texture.texture_rd_rid = a
    if state.changes != 3 or texture.get_rid() != alias:
        state.failures.append("content update replaced the wrapper")
    texture.texture_rd_rid = RID()
    texture.texture_rd_rid = RID()
    if texture.texture_rd_rid.is_valid() or texture.get_width() != 0 or state.changes != 4:
        state.failures.append("clear was not ordered and idempotent")
    texture.texture_rd_rid = a
    if texture.texture_rd_rid != a or texture.get_width() != 8 or state.changes != 5:
        state.failures.append("same source could not be rebound after clear")
    texture.texture_rd_rid = b
    if texture.get_width() != 16 or texture.get_height() != 8:
        state.failures.append("replacement did not publish resized dimensions")
    texture.texture_rd_rid = RID()
    rd.free_rid(a)
    rd.free_rid(b)
    finish.call_deferred(state)

func finish(state):
    print("TEXTURE-RD-REBIND ", JSON.stringify(state))
    quit(0 if state.failures.is_empty() else 2)
