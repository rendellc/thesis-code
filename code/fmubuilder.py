from OMPython import OMCSessionZMQ
omc = OMCSessionZMQ()

def runcmds(cmds):
    for cmd in cmds:
        answer = omc.sendExpression(cmd)
        print(f"\n{cmd}: \n{answer}")

        if not answer:
            return False

    return answer



def build(modelname, filenames):
    cmds = []
    for filename in filenames:
        cmds.append(f'loadFile("{filename}")')

    cmds.append('loadModel(Modelica)')
    cmds.append(f'buildModelFMU({modelname}, fmuType="cs")')

    fmupath = runcmds(cmds)
    return fmupath

def build_autoagri(build_dir):

    #print(omc.sendExpression('getVersion()'))
    omc.sendExpression(f'cd("{build_dir}")')

    from collections import namedtuple
    BuildSpec = namedtuple("BuildSpec", ["modelname", "filenames"])

    buildspecs = [
            BuildSpec("AutoagriModel", ["AutoagriModel.mo", "Body.mo", "Wheel.mo"])
    ]

    # get out of build dir
    for i, (modelname, filenames) in enumerate(buildspecs):
        buildspecs[i] = BuildSpec(modelname, ["../" + fn for fn in filenames])

    successes = []
    for bs in buildspecs:
        didsucceed = build(bs.modelname, bs.filenames)
        successes.append(didsucceed)
        if not didsucceed:
            break

    return all(successes)


if __name__=="__main__":
    build_dir = "modelica/build"
    build_autoagri(build_dir)
