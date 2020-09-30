from OMPython import OMCSessionZMQ
omc = OMCSessionZMQ()

def runcmds(cmds):
    for cmd in cmds:
        answer = omc.sendExpression(cmd)
        print(f"\n{cmd}: \n{answer}")

    return answer



def build(modelname, filenames):
    cmds = []
    for filename in filenames:
        cmds.append(f'loadFile("{filename}")')

    cmds.append('loadModel(Modelica)')
    cmds.append(f'buildModelFMU({modelname}, fmuType="cs")')

    fmupath = runcmds(cmds)
    return fmupath


if __name__=="__main__":
    build_dir = "build"
    print(omc.sendExpression('getVersion()'))
    print(omc.sendExpression(f'cd("{build_dir}")'))

    from collections import namedtuple
    BuildSpec = namedtuple("BuildSpec", ["modelname", "filenames"])

    buildspecs = [
            BuildSpec("AutoagriModel", ["AutoagriModel.mo", "Body.mo", "Wheel.mo"])
    ]

    # get out of build dir
    for i, (modelname, filenames) in enumerate(buildspecs):
        buildspecs[i] = BuildSpec(modelname, ["../" + fn for fn in filenames])

    for bs in buildspecs:
        build(bs.modelname, bs.filenames)



