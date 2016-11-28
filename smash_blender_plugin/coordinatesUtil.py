import bpy
from mathutils import Vector,Quaternion

# in
def jsonVector3ToTuple( obj ):
    return (obj["x"],obj["y"],obj["z"])
def vec3ToBlender( v ):
    return Vector( (v.x, v.z, -v.y) );
def jsonVector4ToTuple( obj ):
    return (obj["x"],obj["y"],obj["z"],obj["w"])
def readQuat( obj ):
    xyzw = jsonVector4ToTuple( obj )
    return Quaternion( (xyzw[3], xyzw[0], xyzw[1], xyzw[2]) )
def quatToBlender( q ):
    return (Quaternion( (q.w, q.x, q.z, -q.y) ) * Quaternion((0.7071067690849304, -0.7071067690849304, 0.0, 0.0))).normalized()
def quatFromBlender( q ):
    return (Quaternion( (q.w, q.x, -q.z, q.y) ) * Quaternion((0.7071067690849304, 0.7071067690849304, 0.0, 0.0))).normalized()


# used: p
#def poseFromInput( pose ):
#    return pose * Quaternion((0.7071067690849304, 0.7071067690849304, 0.0, 0.0))


# out
def vector3ToJson( v ):
    return { 'x': v[0], 'y': v[1], 'z': v[2] };
def quatToJson( v ):
    return { 'w': v[0], 'x': v[1], 'y': v[2], 'z': v[3] };