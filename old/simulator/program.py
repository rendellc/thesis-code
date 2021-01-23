from OpenGL.GL import *


class Program:
    def __init__(self, shaders):
        self._id = glCreateProgram()
        for s in shaders:
            s.compile()
            s.attach(self._id)

        glLinkProgram(self._id)
        result = glGetProgramiv(self._id, GL_LINK_STATUS)
        info_log_len = glGetProgramiv(self._id, GL_INFO_LOG_LENGTH)
        if info_log_len:
            msg = glGetProgramInfoLog(self._id)
            print(msg)
            sys.exit(11)

        for s in shaders:
            glDeleteShader(s.id)


    def use(self):
        glUseProgram(self._id)

    def setUniform4x4(self, name, value_ptr):
        loc = glGetUniformLocation(self.id, name)
        glUniformMatrix4fv(loc, 1, GL_FALSE, value_ptr)

    def setUniformVec3(self, name, vec):
        loc = glGetUniformLocation(self.id, name)
        glUniform3f(loc, vec[0],vec[1],vec[2])

    @property
    def id(self): 
        return self._id

