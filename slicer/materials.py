class Material(object):
    def __init__(self, file_path):
        self.file_path = file_path

        with open(file_path, 'r') as f:
            for line in f:
                split = line.split('=')
                if 'ThermalConductance' in split[0]:
                    self.heat_conductance = float(split[1])
                elif 'MeltTemp' in split[0]:
                    self.melt_temp = float(split[1])
