
from OpenGL.GL import *

import sys

class Shader:
    def __init__(self, name, shader_type, shader_src):
        self._name = name
        self._type = shader_type
        self._src = shader_src

    def compile(self):
        self._id = glCreateShader(self._type)
        glShaderSource(self._id, self._src)
        glCompileShader(self._id)

        # check if compilation succeeded
        success = glGetShaderiv(self._id, GL_COMPILE_STATUS)
        if not success: 
            msg = glGetShaderInfoLog(self._id)
            print(msg)
            sys.exit(10)

    def attach(self, program_id):
        glAttachShader(program_id, self._id)
    
    @property
    def name(self): return self._name

    @property
    def type(self): return self._type

    @property
    def id(self): return self._id



SIMPLEVERT = Shader("simplevert", GL_VERTEX_SHADER, """\
        #version 330 core
        layout(location = 0) in vec3 vertexPosition_modelSpace;
        void main(){
            gl_Position.xyz = vertexPosition_modelSpace;
            gl_Position.w = 1.0;
        }""")

SIMPLEFRAG = Shader("simplefrag", GL_FRAGMENT_SHADER, """\
        #version 330 core
        out vec3 color;
        void main(){
            color = vec3(1.0,0,0);
        }""")

SIMPLE_SHADERS = [SIMPLEVERT, SIMPLEFRAG]


CVERT = Shader("colorvert", GL_VERTEX_SHADER, """\
        #version 330 core
        layout(location = 0) in vec3 vertexPosition_modelSpace;
        layout(location = 1) in vec3 vertexColor;
        out vec3 fragmentColor;

        uniform mat4x4 mvp;

        void main(){
            gl_Position = mvp * vec4(vertexPosition_modelSpace, 1);
            fragmentColor = vertexColor;
        }""")

CFRAG = Shader("colorfrag", GL_FRAGMENT_SHADER, """\
        #version 330 core
        in vec3 fragmentColor;
        out vec3 color;
        void main(){
            color = fragmentColor;
        }""")

COLOR_SHADERS = [CVERT, CFRAG]

