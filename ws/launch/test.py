import subprocess


command = ["python3",
           "/home/cale/thesis-code/ws/src/simulator/simulator/spawn_vehicle.py"]
subprocess.Popen(command, stdout=subprocess.PIPE)
