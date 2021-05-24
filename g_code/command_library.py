class GCodeCommands():

    @staticmethod
    def comment(comment):
        return '( %s )' % comment

    class MovementCommand():
        @staticmethod
        def g0_rapid_positioning(axis_with_feed):
            return 'G0 %s' % axis_with_feed

        @staticmethod
        def g1_linear_move(axis_with_feed):
            return 'G1 %s' % axis_with_feed

    class Dwell():
        @staticmethod
        def dwell(time):
            return 'G4 P%s' % time

    class FeedRate():
        """
        InverseTime: Feed word means the move should be completed in 1/F minutes
        UnitsPerMinute: Feed word means the gantry should move at F units/minute
        """
        INVERSE_TIME = 93
        UNITS_PER_MINUTE = 94
        UNITS_PER_REV = 95
        RANGE = [INVERSE_TIME, UNITS_PER_MINUTE, UNITS_PER_REV]

        @classmethod
        def set_mode(cls, mode):
            if mode in cls.RANGE:
                return 'G%s' % mode
            else:
                raise ValueError('FeedRateMode G%s does not exist' % mode)

        @staticmethod
        def set_feed_rate(rate):
            return 'F%s' % rate

    class WorkSpacePlane():
        XY = 17
        ZX = 18
        YZ = 19
        RANGE = [XY, ZX, YZ]

        @classmethod
        def set_plane(cls, plane):
            if plane in cls.RANGE:
                return 'G%s' % plane
            else:
                raise ValueError('WorkSpacePlane G%s does not exist' % plane)

    class UnitMode():
        IN = 20
        MM = 21
        RANGE = [IN, MM]

        @classmethod
        def set_units(cls, units):
            if units in cls.RANGE:
                return 'G%s' % units
            else:
                raise ValueError('Units G%s does not exist' % units)

    class PositionMode():
        ABSOLUTE = 90
        RELATIVE = 91
        RANGE = [ABSOLUTE, RELATIVE]

        @classmethod
        def set_positioning_mode(cls, pos_mode):
            if pos_mode in cls.RANGE:
                return 'G%s' % pos_mode
            else:
                raise ValueError('Positioning Mode G%s does not exist' % pos_mode)
