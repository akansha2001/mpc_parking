# Import do_mpc package
import importlib.util # Python3.11 onwards an explicit import importlib.util is needed.
import do_mpc
from casadi import *

print(do_mpc.__version__)