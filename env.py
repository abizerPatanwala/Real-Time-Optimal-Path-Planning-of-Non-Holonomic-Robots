"""
Environment for rrt_2D
"""


class Env:
    def __init__(self):
        self.x_range = (0, 50)
        self.y_range = (0, 30)
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary
    '''
    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2]
        ]
        return obs_rectangle
    '''

    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [2, 27, 8, 3],
            [12, 27, 8, 3],
            [22, 27, 8, 3],
            [32, 27, 8, 3],
            [42, 27, 6, 3],
            [40, 1, 6, 8],
            [33, 1, 6, 8],
            [26, 1, 6, 8],
            [42, 12, 6, 8],
            [35, 16, 6, 4],
            [4, 12, 6, 10],
            [12, 15, 2, 2],
            [12, 18, 2, 2],
            [15, 15, 2, 2],
            [15, 18, 2, 2],
            [2, 1, 20, 8],
            [14, 23, 10, 2]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [46, 23, 2],
            [38, 13, 2]
        ]

        return obs_cir
