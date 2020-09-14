from OMPython import OMCSessionZMQ
omc = OMCSessionZMQ()

print(omc.sendExpression("getVersion()"))
print(omc.sendExpression("cd()"))

cmds = [
        'loadFile("TwoTrackVehicle.mo")',
        'buildModelFMU(TwoTrackVehicle, fmuType="cs")',
        # 'simulate(MassSpringDamper)',
        # 'plot(x)'
        ]

for cmd in cmds:
    answer = omc.sendExpression(cmd)
    print(f"\n{cmd}: \n{answer}")
