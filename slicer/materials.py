class Material(object):
    def __init__(self, file_path):
        self.file_path = file_path

        self.density = None
        # Heat capacity
        self.c_p = None
        # Thermal Conductivity
        self.q = None
        # Temperature where material begins to melt and vaporize
        self.transition_temp = None
