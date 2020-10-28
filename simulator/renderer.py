
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
import window

from ctypes import c_void_p

def random_color():
    return np.random.uniform(0,1,3)

class BoxRenderer:
    def __init__(self, box):
        self.box = box
        lx,ly,lz = box.lengths
        
        data = np.array([
            # pos color
            [[-lx/2,-ly/2,-lz/2], random_color()],
            [[-lx/2,-ly/2, lz/2], random_color()],
            [[-lx/2, ly/2,-lz/2], random_color()],
            [[-lx/2, ly/2, lz/2], random_color()],
            [[lx/2,-ly/2,-lz/2],  random_color()],
            [[lx/2,-ly/2, lz/2],  random_color()],
            [[lx/2, ly/2,-lz/2],  random_color()],
            [[lx/2, ly/2, lz/2],  random_color()]
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
    
    def draw(self, prog, projview):
        pos = self.box.position
        rpy = self.box.rpy

        model = glm.translate(glm.mat4(1), pos)
        model = glm.rotate(model, rpy[2], glm.vec3(0,0,1))
        model = glm.rotate(model, rpy[1], glm.vec3(0,1,0))
        model = glm.rotate(model, rpy[0], glm.vec3(1,0,0))
        mvp = projview*model

        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)

        loc = glGetUniformLocation(prog.id, "mvp")
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm.value_ptr(mvp))
        glDrawElements(GL_TRIANGLES, self.indices_size, GL_UNSIGNED_INT, c_void_p(0))

    def __del__(self):
        # TODO: need to do proper cleanup
        pass
        # glDeleteBuffers(1, [self.ebo])
        # glDeleteBuffers(1, [self.vbo])
        # glDeleteVertexArrays(1, [self.vao])

class HorizontalPlaneRenderer:
    def __init__(self, grid_size, prog):
        self.program = prog
        xs = np.arange(-grid_size,grid_size+1)
        ys = np.arange(-grid_size,grid_size+1)
        zs = np.zeros_like(xs)
        N = len(xs)

        vertices = []
        for i in range(N):
            vertices.extend([
                [[xs[i], ys[0], zs[0]], [xs[i], ys[~0], zs[0]]],
                [[xs[0], ys[i], zs[0]], [xs[~0], ys[i], zs[0]]],
            ])
        
        vertices = np.array(vertices, dtype=np.float32)
        n_lines = vertices.shape[0]
        self.n_points = n_lines*2

        self.vao = glGenVertexArrays(1)
        glBindVertexArray(self.vao)
        self.vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices.flatten(), GL_STATIC_DRAW)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, vertices[0,0].size, GL_FLOAT, GL_FALSE, 0, c_void_p(0))

        # self.ebo = glGenBuffers(1)
        # glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)


    def draw(self, projview):
        self.program.use()
        mvp = projview # no model matrix

        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

        loc = glGetUniformLocation(self.program.id, "mvp")
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm.value_ptr(mvp))
        glDrawArrays(GL_LINES, 0, self.n_points)

class CylinderRenderer:
    def __init__(self, cylinder, sectors=8):
        self.cylinder = cylinder

        # generate vertex data
        h = cylinder.height
        r = cylinder.radius

        N = 2*sectors # two points per sector (low z and high z)
        thetas = np.linspace(0, 2*np.pi, sectors)

        pxs = r*np.cos(thetas)
        pys = r*np.sin(thetas)

        vertices = []
        for px, py in zip(pxs, pys):
            vertices.append([
                [px, py, -h/2], random_color(),
            ])
            vertices.append([
                [px, py, h/2], random_color(),
            ])

        vertices = np.array(vertices, dtype=np.float32)

        indices = []
        for even in range(0, N, 2):
            indices.append([
                even%N, (even+2)%N, (even+1)%N
            ])
        for odd in range(1, N, 2):
            indices.append([
                odd%N, (odd+1)%N, (odd+2)%N
            ])

        indices = np.array(indices, dtype=np.uint32)

        self.indices_size = indices.size
        
        # TODO: top and bottom not created above

        self.vao = glGenVertexArrays(1)
        glBindVertexArray(self.vao)
        self.vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

        glVertexAttribPointer(0, vertices[0,0].size, GL_FLOAT, GL_FALSE, vertices[0].nbytes, c_void_p(0))
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(1, vertices[0,1].size, GL_FLOAT, GL_FALSE, vertices[0].nbytes, c_void_p(vertices[0,0].nbytes))
        glEnableVertexAttribArray(1)

        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices.flatten(), GL_STATIC_DRAW)

        self.ebo = glGenBuffers(1)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices.flatten(), GL_STATIC_DRAW)



    def draw(self, prog, projview):
        pos = self.cylinder.position
        rpy = self.cylinder.rpy

        model = glm.translate(glm.mat4(1), pos)
        model = glm.rotate(model, rpy[2], glm.vec3(0,0,1))
        model = glm.rotate(model, rpy[1], glm.vec3(0,1,0))
        model = glm.rotate(model, rpy[0], glm.vec3(1,0,0))
        mvp = projview*model

        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)

        loc = glGetUniformLocation(prog.id, "mvp")
        glUniformMatrix4fv(loc, 1, GL_FALSE, glm.value_ptr(mvp))
        glDrawElements(GL_TRIANGLES, self.indices_size, GL_UNSIGNED_INT, c_void_p(0))



class RendererCollection:
    """
    Collection of renderers that use the same shader program.
    """
    def __init__(self, prog, objects=[]):
        self.prog = prog
        self.objects = objects

    def add(self, obj, *objs):
        self.objects.extend([obj, *objs])

    def draw(self, projview):
        self.prog.use()

        for obj in self.objects:
            obj.draw(self.prog, projview)


class DuckBox:
    def __init__(self, pos, rpy, length):
        self.pos = glm.vec3(*pos)
        self.rpy = glm.vec3(*rpy)
        self.lengths = length

    @property
    def position(self):
        return self.pos

class DuckCylinder:
    def __init__(self, pos, rpy, radius, height):
        self.pos = glm.vec3(*pos)
        self.rpy = glm.vec3(*rpy)
        self.radius = radius
        self.height = height

    @property
    def position(self):
        return self.pos


if __name__=="__main__":
    b1 = DuckBox((0,0,-1), (0,0,0), (0.2,0.2,0.8))
    b2 = DuckBox((0,0,1), (0,0,0), (0.2,1.5,0.4))

    w = window.Window("3D view", 500,400)
    boxes = RendererCollection(
            program.Program(shader.COLOR_SHADERS),
            [BoxRenderer(b1), BoxRenderer(b2)]
    )

    c1 = DuckCylinder((0,0,1), (0,0,0), 1, 2)
    cylinders = RendererCollection(
            program.Program(shader.COLOR_SHADERS),
            [CylinderRenderer(c1)]
    )


    line_program = program.Program(shader.LINE_SHADERS)
    plane = HorizontalPlaneRenderer(6, line_program)

    import time
    theta = 0
    while not w.shouldClose():
        # setup drawing
        w.clear()

        # camera and projection
        target = glm.vec3(0,0,0)
        radius = 7
        eye = glm.vec3(radius*math.cos(theta), radius*math.sin(theta), 0.7*radius)
        view = glm.lookAt(eye, target, glm.vec3(0,0,1))
        proj = glm.perspective(glm.radians(80), w.width/w.height, 0.01, 100.0)
        projview = proj*view

        # draw 
        boxes.draw(projview)
        cylinders.draw(projview)
        plane.draw(projview)

        # update window
        w.swap()

        # check events
        w.poll_events()
        time.sleep(0.05) # TODO: move into window

        # misc
        theta += 0.03
        b1.pos[0] += 0.1*math.cos(theta)
        b1.pos[2] += 0.25*math.sin(theta)
        b2.rpy[2] += 0.1
        c1.pos[2] += 0.1*math.cos(theta)
        c1.rpy[1] += 0.05
        c1.rpy[2] += 0.2

    w.terminate()
