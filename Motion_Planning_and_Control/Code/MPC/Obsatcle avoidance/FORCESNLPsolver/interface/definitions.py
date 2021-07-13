import numpy
import ctypes

name = "FORCESNLPsolver"
requires_callback = True
lib = "lib/FORCESNLPsolver.dll"
lib_static = "lib/FORCESNLPsolver_static.lib"
c_header = "include/FORCESNLPsolver.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (350,   1),  350),
 ("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (100,   1),  100)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x22"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x23"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x24"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x25"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x26"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x27"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x28"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x29"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x31"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x32"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x33"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x34"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x35"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x36"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x37"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x38"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x39"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x40"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x41"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x42"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x43"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x44"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x45"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x46"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x47"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x48"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x49"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7),
 ("x50"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]