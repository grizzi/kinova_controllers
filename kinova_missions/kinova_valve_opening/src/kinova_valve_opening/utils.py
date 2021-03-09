import platform
from scipy.spatial.transform import Rotation


def is_python3():
    return platform.python_version()[0] == '3'

if is_python3():
    class PortableRotation(Rotation):
        def __init__(self, *args, **kwargs):
            Rotation.__init__(self, *args, **kwargs)

        def as_dcm(self):
            return self.as_matrix()
        
        def from_dcm(self):
            return self.from_matrix()
        
else:
    class PortableRotation(Rotation):
        def __init__(self, *args, **kwargs):
            Rotation.__init__(self, *args, **kwargs)

        def as_matrix(self):
            self.as_dcm()
        
        def from_matrix(self):
            self.from_dcm()
