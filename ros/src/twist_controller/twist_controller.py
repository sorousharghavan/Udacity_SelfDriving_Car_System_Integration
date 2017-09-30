
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, **kwargs):
        self.map_carFeatures = {}
        for key in kwargs:
            self.map_carFeatures[key] = kwargs[key]


    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
