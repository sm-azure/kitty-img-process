import pickle

class Lanes(object):

    def __init__(self, calibration_file):
        self._calibration_file = calibration_file
        serialize = pickle.load( open( _calibration_file, "rb" ) )
        self._mtx = serialize["mtx"]
        self._dist = serialize["dist"]
    
    def 
    
