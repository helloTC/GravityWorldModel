"""
Define all macrovarible in this file
"""
import numpy as np

class Const(object):
    """
    Define Macro Const(Variable) in a class 
    """
    PI = np.pi
    INF = np.inf
    NULL_OBJ = -1

    GRAVITY = -9.8
    TIME_STEP = 1.0/240.0

    # Color
    RED = (1, 0, 0, 1)
    GREEN = (0, 1, 0, 1)
    BLUE = (0, 0, 1, 1)
    BLACK = (0, 0, 0, 1)
    YELLOW = (1, 1, 0, 1)
    PURPLE = (1, 0, 1, 1)
    CYAN = (0, 1, 1, 1)
    BROWN = (0.396, 0.263, 0.129, 1)
    TAN = (0.824, 0.706, 0.549, 1)
    GREY = (0.5, 0.5, 0.5, 1)
    WHITE = (1, 1, 1, 0)
    ORIGIN = (1, 1, 1, 1)
    LIGHT_GREY = (0.7, 0.7, 0.7, 1)
    TRANSPARENT = (0, 0, 0, 0)

    # Link
    BASE_LINK = -1



