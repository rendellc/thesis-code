from OMPython import OMCSessionZMQ
omc = OMCSessionZMQ()

def runcmds(cmds):
    for cmd in cmds:
        answer = omc.sendExpression(cmd)
        print(f"\n{cmd}: \n{answer}")

    return answer



def build(filepath, modelname):
    cmds = [
        f'loadFile("{filepath}")',
        f'buildModelFMU({modelname}, fmuType="cs")']

    fmupath = runcmds(cmds)
    return fmupath


if __name__=="__main__":
    build_dir = "build"
    print(omc.sendExpression('getVersion()'))
    print(omc.sendExpression(f'cd("{build_dir}")'))


    import glob
    model_files = glob.glob("*.mo")
    for filename in model_files:
        # assume model is same as filename without extension
        modelname = filename[:~2]
        build("../" + filename, modelname)



