from OpenGL import GL
import xr


# ContextObject is a high level pythonic class meant to keep simple cases simple.
with xr.ContextObject(
	instance_create_info=xr.InstanceCreateInfo(
		enabled_extension_names=[
			# A graphics extension is mandatory (without a headless extension)
			xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
		],
	),
) as context:
	for frame_index, frame_state in enumerate(context.frame_loop()):
		views = context.view_loop(frame_state)
		for view_index, view in enumerate(views):
			# Left
			if view_index == 0:
				GL.glClearColor(1, 0.7, 0.7, 1)  # pink
				GL.glClear(GL.GL_COLOR_BUFFER_BIT)
			# Right
			else:
				GL.glClearColor(1, 0.7, 0, 1)  # orange
				GL.glClear(GL.GL_COLOR_BUFFER_BIT)
		if frame_index > 100:  # Don't run forever
			break