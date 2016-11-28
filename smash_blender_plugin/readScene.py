import bpy
import math
import json
from mathutils import Vector,Quaternion
#from coordinatesUtil import quatToBlender,jsonVector3ToTuple
import coordinatesUtil
import imp
imp.reload(coordinatesUtil)


# def readScene( path, frame ):
#     data = json.load(open(path))
#     input = {}
    
#     for entry in data:
#         name = entry["name"]
#         print( name )
#         input[name] = {}
#         # c 
#         input[name]["c"] = entry["c"]
#         if entry["c"] < 0.05: 
#             print( "Warning, c is low (unset?): %f" % entry["c"] )
#         # mass
#         input[name]["mass"] = entry["mass"]
#         input[name]["size"] = Vector( jsonVector3ToTuple(entry["size"]) )
#         state = entry["log"][str(frame)]
#         input[name]["linVel"] = Vector( jsonVector3ToTuple(state["linVel"]) )
#         input[name]["angVel"] = Vector( jsonVector3ToTuple(state["angVel"]) )
#         input[name]["pos"] = Vector( jsonVector3ToTuple(state["pos"]) )
#         xyzw = jsonVector4ToTuple(state["pose"])
#         input[name]["pose"] = Quaternion( (xyzw[3], xyzw[0], xyzw[1], xyzw[2]) )
#     return input

def readFullScene( path ):
    print('reading path %s' % path )
    data = json.load(open(path))
    input = {}
    
    for entry in data:
        name = entry["name"]
        print( "loading %s" % name )
        input[name] = {}

        for key in entry.keys():
            if key == 'c':
                input[name]["c"] = entry["c"]
                if entry["c"] < 0.05: 
                    print( "Warning, c is low (unset?): %f" % entry["c"] )
            elif key == 'mass':
                input[name]["mass"] = entry["mass"]
            elif key == 'size':
                input[name]["size"] = Vector( coordinatesUtil.jsonVector3ToTuple(entry["size"]) )
            elif key == 'log':
                input[name]['log'] = {}
                for sKey2 in entry['log'].keys():
                    key2 = int(sKey2)
                    input[name]['log'][key2] = {}
                    state = entry['log'][sKey2]
                    if 'pos' in state:
                        input[name]['log'][key2]['pos' ] = coordinatesUtil.vec3ToBlender( Vector(coordinatesUtil.jsonVector3ToTuple(state['pos'])) )
                    #input[name]['log'][key2]['pose'] = quatToBlender( readQuat(state['pose']) )
                    if 'pose' in state:
                        print("pose in state: %s" %  state['pose'].__repr__())
                        input[name]['log'][key2]['pose'] = coordinatesUtil.quatToBlender( coordinatesUtil.readQuat(state['pose']) )
                    if 'linVel' in state:
                        input[name]['log'][key2]['linVel' ] = coordinatesUtil.vec3ToBlender( Vector(coordinatesUtil.jsonVector3ToTuple(state['linVel'])) )
                    if 'angVel' in state:
                        input[name]['log'][key2]['angVel' ] = coordinatesUtil.vec3ToBlender( Vector(coordinatesUtil.jsonVector3ToTuple(state['angVel'])) )
            elif key == 'name':
                pass
            elif key == 'shape':
                input[name]['shape'] = entry[ key ]
            elif key == 'objId':
                pass
            else:
                print( "could not parse key %s" % key )
        print( "[%s] loaded %d keyFrames" % (name,len(input[name]['log']) ) )
    return input

def addKeyFrames( input, context ):
    for name in input.keys():
        entry = input[name];
        bpy.ops.object.select_pattern(pattern=name+"*", extend=False )
        #print( "pattern: %s" % name+'*')
        obj = context.selected_objects[0]
        print("Keyframing %s" % obj.name )
        
        prevPose = None
        prevEuler = None
        offset = Vector((0,0,0))
        for sFrameId in entry['log'].keys():
            state = entry['log'][sFrameId]
            frameId = int(sFrameId)
            #print( "Adding keyFrame %d" % frameId )
            context.scene.frame_set( frameId )
            hasPos = False
            hasPose = False
            if 'pos' in state:
                obj.location = state['pos']
                hasPos = True
            if 'pose' in state:
                obj.rotation_mode = 'QUATERNION'
                q = Quaternion(state['pose'])
                #if prevPose:
                    # if q.dot(prevPose) < 0:
                        # q.negate()
                    #     print('newq:',q)
                    # else:
                    #     print('dotok:', prevPose, q, q.dot(prevPose))
                obj.rotation_quaternion = q
                hasPose = True
            #obj.rotation_euler = q.to_euler('XYZ')
            #obj.rotation_euler.x += offset.x
            #obj.rotation_euler.y += offset.y
            #obj.rotation_euler.z += offset.z
            #if prevEuler:
            #    d = Vector(obj.rotation_euler) - Vector(prevEuler)
            #    if d.length > 1:
            #        print("[",sFrameId,"] d: ", d )
            #        print('prev: ', prevEuler, 'new: ', obj.rotation_euler)
           #     if abs(d.x) > math.pi/2:
           #         offset.x += 2*math.pi
           #         obj.rotation_euler.x += 2*math.pi
           #         print( "offset.x:" , offset.x, 'newx: ', obj.rotation_euler.x )
           #     if abs(d.y) > math.pi/2:
            #       offset.y += 2*math.pi
            #        obj.rotation_euler.y += 2*math.pi
            #        print( "offset.y:" , offset.y, 'newy: ', obj.rotation_euler.y  )
            #    if abs(d.z) > math.pi/2:
            #        offset.z += 2*math.pi
            #        obj.rotation_euler.z += 2*math.pi
            #        print( "offset.z:" , offset.z, 'newz: ', obj.rotation_euler.z  )

            #obj.rotation_euler.x -= math.pi / 2.
            #bpy.ops.anim.keyframe_insert()
            #obj.keyframe_insert( data_path="LocRot", index=-1,frame=frameId)
            # try:
            #     if hasPose and hasPos:
            #         bpy.ops.anim.keying_set_active_set( type='BUILTIN_KSI_LocRot' )
            #     elif hasPose:
            #         bpy.ops.anim.keying_set_active_set( type='BUILTIN_KSI_Rot' )
            #     elif hasPos:
            #         bpy.ops.anim.keying_set_active_set( type='BUILTIN_KSI_Loc')
            #     bpy.ops.anim.keyframe_insert()
            # except:
            if hasPos:
                obj.keyframe_insert( data_path="location", index=-1,frame=frameId)
            if hasPose:
                #print('adding pose keyframe')
                obj.keyframe_insert( data_path="rotation_quaternion", index=-1,frame=frameId)
            
            #obj.keyframe_insert(index=-1)
            #obj.keyframe_insert(data_path="location", index=-1, frame=frameId)
            #obj.keyframe_insert(data_path="rotation", index=-1, frame=frameId)
            if hasPose:
                prevPose = q.copy()
                prevEuler = obj.rotation_euler.copy()
            
        #context.scene.frame_set(current_frame)
        #bpy.context.active_object.hide = doHide
        #bpy.context.active_object.hide_render = doHide
        #bpy.context.active_object.keyframe_insert( data_path="hide",
        #                                           index=-1, 
        #                                         frame=current_frame)