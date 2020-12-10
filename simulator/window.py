import glfw

from OpenGL.GL import *

import numpy as np


class Window:
    def __init__(self, title, width, height):
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.set_error_callback(self._glfw_error_callback)

        self.width, self.height = width, height
        self.window = glfw.create_window(self.width, self.height, title, None,None)
        assert self.window, "failed to create window"
        glfw.set_window_size_callback(self.window, self._glfw_window_size_callback)

        self.x, self.y = glfw.get_window_pos(self.window)

        glfw.make_context_current(self.window)

        glfw.set_input_mode(self.window, glfw.STICKY_KEYS, True)

        glClearColor(0.3, 0.3, 0.3, 0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        self.left_arrow_pressed = False
        self.right_arrow_pressed = False

    def _glfw_window_size_callback(self, window, width, height):
        self.width, self.height = width, height
        glViewport(0,0,self.width,self.height)

    def _glfw_window_position_callback(self, window, x, y):
        self.x, self.y = x, y

    def _glfw_error_callback(self, errorint, errorstr):
        print("ERROR:", errorint, errorstr)

    def capture(self):
        """
        Read pixel values from framebuffer and return as RGB image array
        """
        data = glReadPixels(0,0,self.width,self.height, GL_RGB, GL_UNSIGNED_BYTE)
        data = np.frombuffer(data, np.uint8)
        data = data.reshape((self.height,self.width,3))
        data = np.flipud(data)
        return data


    def clear(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    def swap(self):
        glfw.swap_buffers(self.window)

    def poll_events(self):
        glfw.poll_events()


    def shouldClose(self):
        if glfw.get_key(self.window, glfw.KEY_ESCAPE) == glfw.PRESS or glfw.window_should_close(self.window):
            return True
        else:
            return False

    def terminate(self):
        glfw.terminate()

