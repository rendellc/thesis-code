from fmpy import dump, read_model_description, extract
from fmpy.fmi2 import FMU2Slave
import shutil

class SimulatorModel:
    def __init__(self, start_time, fmufilepath):
        self.fmu_filename = fmufilepath

        model_description = read_model_description(self.fmu_filename)
        self.vrs = {}
        for variable in model_description.modelVariables:
            self.vrs[variable.name] = variable.valueReference


        self.unzipdir = extract(self.fmu_filename)

        self.fmu = FMU2Slave(guid=model_description.guid,
                unzipDirectory=self.unzipdir,
                modelIdentifier=model_description.coSimulation.modelIdentifier,
                instanceName="instance1")

        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=start_time)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()

    def _get_vector_names(self, name, indices):
        names = [f"{name}[{i}]" for i in indices]
        return names


    def getVec(self, name, length):
        indices = range(1,length+1) # FMUs are 1-indexed
        names = self._get_vector_names(name, indices)
        return self.get(names)

    def setVec(self, name, values):
        indices = range(1,len(values)+1) # FMUs are 1-indexed
        names = self._get_vector_names(name, indices)
        assert len(names) == len(values)
        self.fmu.setReal([self.vrs[name] for name in names], values)

    
    def get(self, names):
        values = self.fmu.getReal([self.vrs[name] for name in names])
        return values
        
    def set(self, names, values):
        assert len(names) == len(values)
        self.fmu.setReal([self.vrs[name] for name in names], values)

    def step(self, time, stepsize):
        self.fmu.doStep(currentCommunicationPoint=time, communicationStepSize=stepsize)

    def cleanup(self):
        if hasattr(self, "fmu"):
            self.fmu.terminate()
            self.fmu.freeInstance()

        # clean up
        if hasattr(self, "unzipdir"):
            shutil.rmtree(self.unzipdir, ignore_errors=True)

    
    def __del__(self):
        self.cleanup()










