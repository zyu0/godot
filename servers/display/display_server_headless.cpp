/**************************************************************************/
/*  display_server_headless.cpp                                           */
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

#include "display_server_headless.h"

#include "core/input/input.h"
#include "core/input/input_event.h"
#include "servers/display/native_menu.h"
#include "servers/rendering/dummy/rasterizer_dummy.h"
#if defined(TOOLS_ENABLED) && defined(RD_ENABLED)
#include "servers/rendering/renderer_rd/renderer_compositor_rd.h"
#include "servers/rendering/rendering_device.h"
#ifdef VULKAN_ENABLED
#include "drivers/vulkan/rendering_context_driver_vulkan.h"
#endif
#ifdef METAL_ENABLED
#include "drivers/metal/rendering_context_driver_metal.h"
#endif
#endif

DisplayServer *DisplayServerHeadless::create_func(const String &p_rendering_driver, DisplayServerEnums::WindowMode p_mode, DisplayServerEnums::VSyncMode p_vsync_mode, uint32_t p_flags, const Vector2i *p_position, const Vector2i &p_resolution, int p_screen, DisplayServerEnums::Context p_context, int64_t p_parent_window, Error &r_error) {
	r_error = OK;
	DisplayServerHeadless *server = memnew(DisplayServerHeadless());
#if defined(TOOLS_ENABLED) && defined(RD_ENABLED)
	// Keep ordinary headless runs on dummy. An explicit GPU driver opts into
	// offscreen rendering without creating a native window, surface or swapchain.
#ifdef VULKAN_ENABLED
	if (p_rendering_driver == "vulkan") {
		server->rendering_context = memnew(RenderingContextDriverVulkan);
	}
#endif
#ifdef METAL_ENABLED
	if (p_rendering_driver == "metal") {
		server->rendering_context = memnew(RenderingContextDriverMetal);
	}
#endif
	if (server->rendering_context) {
		server->offscreen_size = p_resolution;
		r_error = server->rendering_context->initialize();
		if (r_error == OK) {
			server->rendering_device = memnew(RenderingDevice);
			r_error = server->rendering_device->initialize(server->rendering_context);
		}
		if (r_error != OK) {
			memdelete(server);
			return nullptr;
		}
		RendererCompositorRD::make_current();
		print_line("HEADLESS_GPU_READY driver=" + p_rendering_driver + " windows=0");
		return server;
	}
#endif
	RasterizerDummy::make_current();
	return server;
}

void DisplayServerHeadless::_dispatch_input_events(const Ref<InputEvent> &p_event) {
	static_cast<DisplayServerHeadless *>(get_singleton())->_dispatch_input_event(p_event);
}

void DisplayServerHeadless::_dispatch_input_event(const Ref<InputEvent> &p_event) {
	if (input_event_callback.is_valid()) {
		input_event_callback.call(p_event);
	}
}

void DisplayServerHeadless::process_events() {
	Input::get_singleton()->flush_buffered_events();
}

DisplayServerHeadless::DisplayServerHeadless() {
	native_menu = memnew(NativeMenu);
	Input::get_singleton()->set_event_dispatch_function(_dispatch_input_events);
}

DisplayServerHeadless::~DisplayServerHeadless() {
#if defined(TOOLS_ENABLED) && defined(RD_ENABLED)
	if (rendering_device) {
		memdelete(rendering_device);
		rendering_device = nullptr;
	}
	if (rendering_context) {
		memdelete(rendering_context);
		rendering_context = nullptr;
	}
#endif
	if (native_menu) {
		memdelete(native_menu);
		native_menu = nullptr;
	}
}
