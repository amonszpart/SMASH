import bge
import bpy
import os;
import copy;
import mathutils;

scene   = bpy.context.scene
cont    = bge.logic.getCurrentController() # bge.logic is automatically imported
obj     = cont.owner
doInit  = False;

def onCollision( other, pointList, normal ):
    print( "%s collided with %s" % ( obj.name, other.name) );
    #print( self.sensors[0] );
#    for i in range(pointList.numPoints):
#        point = pointList.getContactPoint(i)
#        print(point.localPointA, point.localPointB, point.worldPoint, point.normal, point.appliedImpulse)
    timestampKey = "%05d" % scene['timestamp']
    print( "collTimestamp %s" % timestampKey );
    obj['lastCollision'] = { 'timestamp': timestampKey, 'point': pointList.freeze(), 'normal': normal.freeze() };
    print( "[onCollision]: v: ", obj.linearVelocity )    
    print( "[onCollision]: a: ", obj.angularVelocity )    
    #const Scalar numerator  = -(Scalar(1.) + body0.getElasticity() * body1.getElasticity()) * relVelOnNormal;
    #const Scalar inverseMasses = body0.m_massInverse + body1.m_massInverse;

    #const Vector4 rma         = cross3( vector3TransformNormal( cross3( xaWorld, info.normalWorld ), currentInertiaTensorInverse0 ), xaWorld ); //XMVector3Cross(XMVector3TransformNormal(XMVector3Cross(xaWorld, info.normalWorld), currentInertiaTensorInverse0), xaWorld);
    #const Vector4 rmb         = cross3( vector3TransformNormal( cross3( xbWorld, info.normalWorld ), currentInertiaTensorInverse1 ), xbWorld ); // XMVector3Cross(XMVector3TransformNormal(XMVector3Cross(xbWorld, info.normalWorld), currentInertiaTensorInverse1), xbWorld);
    #const float   rmab        = vector3Dot( rma+rmb, info.normalWorld ); // XMVectorGetX(XMVector3Dot(rma + rmb, info.normalWorld));
    #const float   denominator = inverseMasses + rmab;
    #std::cout << "xaWorld: " << xaWorld.transpose() << ", xbWorld: " << xbWorld.transpose() << std::endl; fflush(stdout);

    #const float impulse = numerator / denominator;

if ( len(obj.collisionCallbacks) == 0 ):
    obj.collisionCallbacks = [onCollision]

if obj.getPhysicsId() != 0:
    obj['p'] = obj.linearVelocity * obj.mass;

    if not 'data' in obj.attrDict:
        blendObj = bpy.data.objects[obj.name]
        dims     = blendObj.dimensions
        shape    = blendObj.game.collision_bounds_type
        if ( shape == "CONVEX_HULL" ):
            shape = obj.name.upper();
        inertia  = obj.localInertia
        obj['data'] = { 'objId' : id(obj), \
                        'log' : {}, \
                        'mass' : obj.mass,\
                        'name' : obj.name, \
                        'size'   : { 'x': dims[0]   , 'y': dims[1]   , 'z': dims[2]   }, \
                        'inertia': { 'x': inertia[0], 'y': inertia[1], 'z': inertia[2]}, \
                        'shape': shape \
                        }

        for mesh in obj.meshes:
            for i,m in enumerate(mesh.materials):
                matName = mesh.getMaterialName( i )
                print( matName[2:] )
                mat = bpy.data.materials[ matName[2:] ];
                obj['data']['c'] = mat.physics.elasticity;
                break;

    v    = obj.linearVelocity
    a    = obj.angularVelocity
    pose = obj.worldOrientation.to_quaternion()
    pos  = obj.worldPosition

    timestampKey = "%05d" % scene['timestamp']
    obj['data']['log'][ timestampKey ] = { 'linVel' : {'x': v[0]  , 'y': v[1]  , 'z':v[2]                 }, \
                                           'angVel' : {'x': a[0]  , 'y': a[1]  , 'z':a[2]                 }, \
                                           'pose'   : {'w': pose.w, 'x': pose.x, 'y': pose.y, 'z': pose.z }, \
                                           'pos'    : {'x': pos[0], 'y': pos[1], 'z': pos[2]              } \
                                         }
    if 'lastCollision' in  obj.attrDict:
        tmp = copy.deepcopy(obj['lastCollision']);
        del( obj['lastCollision'] )
        print( "timestamp %s vs. %s" % ( timestampKey, tmp['timestamp'] ) );
        print( tmp )
        if ( abs(scene['timestamp'] - int(tmp['timestamp'])) < 5 ):
            obj['data']['log'][ timestampKey ][ 'collNormal' ] = { 'x': tmp['normal'][0], 'y': tmp['normal'][1], 'z': tmp['normal'][2] };
            obj['data']['log'][ timestampKey ][ 'collPoint'  ] = { 'x': tmp['point'][0], 'y': tmp['point'][1], 'z': tmp['point'][2] };
#    print( "%s: %s" % (timestampKey, obj['data']['log'][ timestampKey ].__repr__()) )

    print( "[%s] v: %s" % (timestampKey, v.__repr__()) )
    print( "[%s] a: %s" % (timestampKey, a.__repr__()) )