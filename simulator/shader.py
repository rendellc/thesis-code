from OpenGL.GL import *

class Shader:
    def __init__(self, name, shader_type, shader_src):
        self._name = name
        self._type = shader_type
        self._src = shader_src

    @classmethod
    def from_filename(cls, name, shader_type, filename):
        with open(filename, "r") as f:
            src = f.read()
            return cls(name, shader_type, src)

    def compile(self):
        self._id = glCreateShader(self._type)
        glShaderSource(self._id, self._src)
        glCompileShader(self._id)

        # check if compilation succeeded
        success = glGetShaderiv(self._id, GL_COMPILE_STATUS)
        if not success: 
            msg = glGetShaderInfoLog(self._id)
            raise RuntimeError(msg)

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
            color = vec3(1,1,1);
        }""")

SIMPLE_SHADERS = [SIMPLEVERT, SIMPLEFRAG]


CVERT = Shader.from_filename("colorvert", GL_VERTEX_SHADER, "simulator/shaders/color.vert")
CFRAG = Shader.from_filename("colorfrag", GL_FRAGMENT_SHADER, "simulator/shaders/color.frag")

COLOR_SHADERS = [CVERT, CFRAG]


LINEVERT = Shader("linevert", GL_VERTEX_SHADER, """\
        #version 330 core
        layout(location = 0) in vec3 vertexPosition_modelSpace;

        uniform mat4x4 mvp;

        void main(){
            gl_Position = mvp * vec4(vertexPosition_modelSpace, 1);
        }""")

LINE_SHADERS = [LINEVERT, SIMPLEFRAG]

