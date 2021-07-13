import numpy
import ctypes

name = "FORCESNLPsolver"
requires_callback = True
lib = "lib/FORCESNLPsolver.dll"
lib_static = "lib/FORCESNLPsolver_static.lib"
c_header = "include/FORCESNLPsolver.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  5,   1),    5),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (210,   1),  210),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, ( 60,   1),   60),
 ("reinitialize"        , ""      , "FORCESNLPsolver_int", ctypes.c_int   , numpy.int32  , (  0,   1),    1)]

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
 ("x30"                 , ""      , ""               , ctypes.c_double, numpy.float64,     (  7,),    7)]

# Info Struct Fields
info = \
[("it", ctypes.c_int),
 ("res_eq", ctypes.c_double),
 ("rsnorm", ctypes.c_double),
 ("pobj", ctypes.c_double),
 ("solvetime", ctypes.c_double),
 ("fevalstime", ctypes.c_double),
 ("QPtime", ctypes.c_double)]