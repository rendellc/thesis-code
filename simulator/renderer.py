
# interface
# class Renderable:

from OpenGL.GL import *
import OpenGL.GLUT
import OpenGL.GLU
import glfw
import glm

import sys
import numpy as np
import math

import shader
import program

from ctypes import c_void_p

class BoxRenderer:
    def __init__(self, box):
        self.box = box
        lx,ly,lz = box.lengths

        data = np.array([
            # pos color
            [[-lx/2,-ly/2,-lz/2],[0, 0, 0]],
            [[-lx/2,-ly/2, lz/2], [0, 0, 1]],
            [[-lx/2, ly/2,-lz/2], [0, 1, 0]],
            [[-lx/2, ly/2, lz/2],  [0, 1, 1]],
            [[lx/2,-ly/2,-lz/2], [1, 0, 0]],
            [[lx/2,-ly/2, lz/2],  [1, 0, 1]],
            [[lx/2, ly/2,-lz/2],  [1, 1, 0]],
            [[lx/2, ly/2, lz/2],   [1, 1, 1]]
        ], dtype=np.float32)
        indices = np.array([
            [0,1,3],
            [0,2,6],
            [0,3,2],
            [0,4,1],
            [0,6,4],
            [1,4,5],
            [1,5,7],
            [1,7,3],
            [2,3,7],
            [2,7,6],
            [4,6,7],
            [4,7,5]
        ], dtype=np.uint32)
        self.indices_size = indices.size

        self.vao = glGenVertexArrays(1)
        glBindVertexArray(self.vao)
        self.vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

        glVertexAttribPointer(0, data[0,0].size, GL_FLOAT, GL_FALSE, data[0].nbytes, c_void_p(0))
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(1, data[0,1].size, GL_FLOAT, GL_FALSE, data[0].nbytes, c_void_p(data[0,0].nbytes))
        glEnableVertexAttribArray(1)

        glBufferData(GL_ARRAY_BUFFER, data.nbytes, data.flatten(), GL_STATIC_DRAW)

        self.ebo = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices.flatten(), GL_STATIC_DRAW)
    
    def draw(self, shader_program, projview):
        model = glm.translate(glm.mat4(1), self.box.pos)
        model = glm.rotate(model, self.box.rpy[2],glm.vec3(0,0,1))
        model = glm.rotate(model, self.box.rpy[1],glm.vec3(0,1,0))
        model = glm.rotate(model, self.box.rpy[0],glm.vec3(1,0,0))
        mvp = projview*model

        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)

        loc = glGetUniformLocation(shader_program, "mvp")
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm.value_ptr(mvp))
        glDrawElements(GL_TRIANGLES, self.indices_size, GL_UNSIGNED_INT, c_void_p(0))

    def __del__(self):
        glDeleteBuffers(2, [self.vbo, self.ebo])
        glDeleteVertexArrays(1, [self.vao])




class Renderer:
    def __init__(self):
        print("Initializing renderer")
        glfw.init()
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, True)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.set_error_callback(self._glfw_error_callback)

        self.window = glfw.create_window(500,400,"3D view",None,None)
        assert self.window, "failed to create window"

        glfw.make_context_current(self.window)

        glfw.set_input_mode(self.window, glfw.STICKY_KEYS, True)
        glClearColor(0.7, 0.7, 0.7, 0)

        self.objects = []

    def add(self, obj, *objs):
        self.objects.extend([obj, *objs])
        print(self.objects)


    def render(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glDrawArrays(GL_TRIANGLES, 0, 3)
        glfw.swap_buffers(self.window)



    def _glfw_error_callback(self, errorint, errorstr):
        print("ERROR:", errorint, errorstr)

    def __del__(self):
        glfw.terminate()


class Box:
    def __init__(self, pos, rpy, length):
        self.pos = glm.vec3(*pos)
        self.rpy = glm.vec3(*rpy)
        self.lengths = length

if __name__=="__main__":
    r = Renderer()

    b1 = Box((0,0,-1), (0,0,0), (0.2,0.2,0.8))
    b1_renderer = BoxRenderer(b1)

    b2 = Box((0,0,1), (0,0,0), (0.2,1.5,0.4))
    b2_renderer = BoxRenderer(b2)

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    prog = program.Program(shader.COLOR_SHADERS)
    prog.use()

    import time
    theta = 0
    while (
            glfw.get_key(r.window, glfw.KEY_ESCAPE) != glfw.PRESS and
            not glfw.window_should_close(r.window)
        ):
        # setup drawing
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        target = glm.vec3(0,0,0)
        radius = 5
        eye = glm.vec3(radius*math.cos(theta), radius*math.sin(theta), radius)
        view = glm.lookAt(eye, target, glm.vec3(0,0,1))
        proj = glm.perspective(glm.radians(80), 500/400, 0.01, 100.0)

        # draw boxes
        projview = proj*view
        b1_renderer.draw(prog.id, projview)
        b2_renderer.draw(prog.id, projview)

        # flush drawing to screen
        glfw.swap_buffers(r.window)

        glfw.poll_events()
        time.sleep(0.05)
        theta += 0.1
        b1.pos[0] += 0.1*math.cos(theta)
        b1.pos[2] += 0.25*math.sin(theta)
        b2.rpy[2] += 0.5

    # cleanup
    for s in shader.COLOR_SHADERS:
        glDetachShader(prog.id, s.id)

    glUseProgram(0)
    glDeleteProgram(prog.id)
    glDisableVertexAttribArray(0)
    glDisableVertexAttribArray(1)

