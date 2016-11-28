import bpy
import json
import math
from mathutils import Vector,Quaternion,Euler, Matrix
from coordinatesUtil import *


def matchInput( input, scene ):
    names2input = {}
    for ob in scene.objects:
        if not ob.type == 'MESH':
            continue
        for inputName in input.keys():
            if ob.name.find( inputName ) == 0:
                print( "match: %s - %s" % (ob.name, inputName) )
                names2input[ob.name] = inputName
                break
    return dict(zip(names2input.values(), names2input.keys()))


def matchNames(parabolas, scene):
    taken = []
    names2input = {}
    for parabola in parabolas:
        name = parabola.physacq.objName
        found = False
        for ob in scene.objects:
            if not ob.type == 'MESH' or ob.name.find('pa') >= 0:
                continue
            if ob.name.find(name) >= 0 and ob.name not in taken:
                print("match: %s - %s" % (ob.name, name))
                names2input[ob.name] = parabola
                taken.append(ob.name)
                found = True
                break
        if not found:
            toCopy = next((x for x in taken if x.find(name) >= 0), None)
            if toCopy:
                bpy.ops.object.select_pattern(pattern=toCopy, extend=False)
                bpy.ops.object.duplicate_move_linked()
                copied = bpy.context.selected_objects[0]
                names2input[copied.name] = parabola
                taken.append(copied.name)
                print("copied.name: %s for query %s" % (copied.name, name))
            else:
                print("This object is not there at all: %s!" % name)
    return dict(zip(names2input.values(), names2input.keys()))


def createMaterialName(objName):
    return objName + "Material"


def createMaterial(targetObj):
    mat = bpy.data.materials.new( createMaterialName(targetObj.name) )
    print(mat.name)
    mat.diffuse_color = (0.8, 0.8, 0.8)
    mat.diffuse_shader = 'LAMBERT'
    mat.diffuse_intensity = 0.
    mat.specular_color = (1.0,1.0,1.0)
    mat.specular_shader = 'COOKTORR'
    mat.specular_intensity = 0.1
    mat.alpha = 1.0
    mat.ambient = 1.0
    mat.physics.elasticity = 1.0
    mat.physics.friction = 0.1
    return mat


def clearActuators(newObj):
    for act in newObj.game.actuators:
        bpy.ops.logic.actuator_remove(actuator=act.name, object=newObj.name)
    for sensor in newObj.game.sensors:
        bpy.ops.logic.sensor_remove(sensor=sensor.name, object=newObj.name)
    for controller in newObj.game.controllers:
        bpy.ops.logic.controller_remove(controller=controller.name, object=newObj.name)


def addActuators(newObj):
    # --------------- #
    # 1 Init          #
    # 1    Sensor     #
    bpy.ops.logic.sensor_add(type='ALWAYS', object=newObj.name)
    initSensor = newObj.game.sensors[-1]
    initSensor.use_tap = True
    initSensor.name = 'Initialize'
    # 1    Controller #
    bpy.ops.logic.controller_add(type="LOGIC_AND", object=newObj.name)
    initController = newObj.game.controllers[-1]
    initController.name = 'AndInit'
    initController.link(initSensor)
    # 1    Actuator   #
    bpy.ops.logic.actuator_add(type='EDIT_OBJECT', object=newObj.name)
    initActuator = newObj.game.actuators[-1]
    initActuator.mode = 'DYNAMICS'
    initActuator.dynamic_operation = 'SUSPENDDYN'
    initActuator.name = 'Suspend'
    initActuator.link(initController)
    # --------------- #

    # --------------- #
    # 2 Start         #
    # 2    Sensor     #
    bpy.ops.logic.sensor_add(type='DELAY', object=newObj.name)
    startSensor = newObj.game.sensors[-1]
    startSensor.use_tap = True
    startSensor.name = 'Start'
    # 2    Controller #
    bpy.ops.logic.controller_add(type="LOGIC_AND", object=newObj.name)
    startController = newObj.game.controllers[-1]
    startController.name = 'AndStart'
    startController.link(startSensor)
    # 2    Actuator   #
    bpy.ops.logic.actuator_add(type='EDIT_OBJECT', object=newObj.name)
    startActuator = newObj.game.actuators[-1]
    startActuator.name = 'Restore'
    startActuator.mode = 'DYNAMICS'
    startActuator.dynamic_operation = 'RESTOREDYN'
    startActuator.link(startController)
    # --------------- #

    # --------------- #
    # 3 Motion        #
    # 3    Actuator   #
    bpy.ops.logic.actuator_add(type='MOTION', object=newObj.name)
    motionActuator = newObj.game.actuators[-1]
    motionActuator.link(startController)
    # --------------- #

    # --------------- #
    # 4 Collision          #
    # 1    Sensor     #
    bpy.ops.logic.sensor_add(type='COLLISION', object=newObj.name)
    collSensor = newObj.game.sensors[-1]
    collSensor.name = 'Collision'
    # 1    Controller #
    bpy.ops.logic.controller_add(type="PYTHON", object=newObj.name)
    pyController = newObj.game.controllers[-1]
    pyController.name = 'Python'
    pyController.link(collSensor)
    pyController.text = bpy.data.texts['collisionGroup.py']
    # --------------- #


def rigScene(input, frameId, scene):
    input2names = matchInput(input, scene)
    sFrameId = str(frameId)

    for inputName in input.keys():
        name = input2names[ inputName ]
        cube = scene.objects[ name ]
        cube.animation_data_clear()
        cube.game.physics_type = 'RIGID_BODY'
        cube.game.mass = input[inputName]["mass"]
        cube.game.damping = 0.0
        cube.game.rotation_damping = 0.0
        if len(cube.data.materials) != 1:
            materialName = createMaterialName( name )
            material = createMaterial( cube )
            cube.data.materials.clear()
            cube.data.materials.append( material )    
        else:
            material = bpy.data.materials[ 0 ]
            materialName = material.name

        material.physics.elasticity = input[ inputName ]["c"]
        print("mass[%s]: %f" % (name,cube.game.mass))
        print("c[%s]: %f" % (materialName, material.physics.elasticity))

        log = input[inputName]['log']
        entry = log[frameId]
        cube.location = entry["pos"]
        cube.rotation_quaternion = entry['pose']

        #if len(cube.game.actuators) < 1:
        #    addActuators(cube)
        clearActuators(cube)
        addActuators(cube)

        act = cube.game.actuators['Motion']

        act.linear_velocity = entry["linVel"]
        act.angular_velocity = entry["angVel"]
        
        cube.game.use_collision_bounds = True
        cube.game.collision_margin = 0.001
        cube.game.collision_bounds_type = 'CONVEX_HULL'
        cube.rotation_mode = 'XYZ'


def rigParabolas(context, inFrameId):
    frameId = int(inFrameId * 3)
    context.scene.frame_set(inFrameId)
    bpy.ops.object.select_pattern(pattern='pa.*', extend=False)
    parabolas = []
    groups = []
    for ob in context.selected_objects:
        if ob.type == 'EMPTY':
            groups.append(ob)
        elif ob.type == 'CURVE':
            parabolas.append(ob)

    parabolas2names = matchNames(parabolas, context.scene)
    print("parabolas2names: ", parabolas2names)
    
    for parabola in parabolas2names.keys():
        group = parabola.parent
        transl = group.location - Vector(group.physacq.groupOrigin) # + parabola.location
        timeOffset = parabola.parent.physacq.timeOffset * 3
        loc, rot, sca = group.matrix_world.decompose()
        print("timeOffset: ", timeOffset, ", translation: ", transl, "rot: ", rot)
        
        name = parabolas2names[parabola]
        print(parabola.physacq.objName, parabolas2names[parabola])

        cube = context.scene.objects[name]
        print(cube.name)
        cube.animation_data_clear()
        cube.game.physics_type = 'RIGID_BODY'
        cube.game.mass = parabola.physacq.mass
        cube.game.damping = 0.1
        cube.game.rotation_damping = 0.1
        if len(cube.data.materials) != 1:
            print('[ERROR]: physics object %s should only have one material!' % (cube.name))
        else:
            material = cube.data.materials[0]

        print('setting material %s elasticity for object %s' % (material.name, name))
        material.physics.elasticity = parabola.physacq.c * parabola.physacq.c
        
        backlog = frameId + timeOffset
        print('backlog: ', backlog)
        entry = parabola.physacq.log[str(max(0,frameId+timeOffset))]

        # Position
        cube.location = rot * (Vector(entry.pos) - Vector(group.physacq.groupOrigin)) + group.location

        # Pose
        cube.rotation_mode = 'QUATERNION'
        cube.rotation_quaternion = rot * Quaternion(entry.pose)

        # Initial conditions
        if len(cube.game.sensors) < 3:
            clearActuators(cube)
            addActuators(cube)
        act = cube.game.actuators['Motion']
        i = 0
        while act.type != 'MOTION':
            i += 1
            act = cube.game.actuators[i]
        
        # Initial velocity
        act.linear_velocity = rot * Vector(entry.linVel)

        # Initial angular velocity
        act.angular_velocity = rot * Vector(entry.angVel)

        # if backlog < 0:
        #     cube.game.sensors['Start'].delay = backlog * -3
        cube.game.sensors['Start'].delay = -parabola.parent.physacq.delay

        if backlog < 0:
            print('fixing backlog')
            fps = context.scene.game_settings.fps * 3.
            g = Vector((0.,0.,-context.scene.game_settings.physics_gravity))
            dt = backlog / fps # negative
            cube.location += act.linear_velocity * dt + g/2. * dt * dt
            print('loc: ', cube.location )
            act.linear_velocity += dt * g
            print( 'fps: ', context.scene.game_settings.fps, 'g: ', g )

        cube.game.use_collision_bounds = True
        cube.game.collision_margin = 0.00
        cube.game.collision_bounds_type = 'CONVEX_HULL'
        cube.rotation_mode = 'XYZ'
        for i in range(0, 16):
            cube.game.collision_mask[i] = 0
            cube.game.collision_group[i] = 0
        cube.game.collision_mask[0] = 1
        cube.game.collision_group[0] = 1
        cube.game.collision_group[1] = 1


def MakePolyLine(scene, curveName, cList):
    curvedata = bpy.data.curves.new(name=curveName, type='CURVE')
    curvedata.dimensions = '3D'
    curvedata.bevel_resolution = 6 
    curvedata.bevel_depth = 0.01
    curvedata.fill_mode = 'FULL'

    objectdata = bpy.data.objects.new(curveName+'Object', curvedata)
    objectdata.location = (0,0,0) #object origin
    scene.objects.link(objectdata)

    polyline = curvedata.splines.new('POLY')
    polyline.points.add(len(cList)-1)

    for num in range(len(cList)):
        if len(cList[num]) < 4:
            polyline.points[num].co = (cList[num][0],cList[num][1],cList[num][2],1)
        else:
            polyline.points[num].co = (cList[num])
        
    polyline.order_u = len(polyline.points)-1  
    polyline.use_endpoint_u = True
    polyline.use_cyclic_u = False

    return objectdata


def getCollisionTime( p0, p1 ):
    d = 10000
    tcoll = 0
    
    # for e in p0.physacq.log:
    #     print e
    #     print( e.name )
    #     #f = filter(lambda x: x.name == e.name, p1.physacq.log)
    #     print(f.name)
    #     if e.name in p1.physacq.log:
    #         e2 = p1.physacq.log[e.name]
    #         dist = Vector(e.pos) - Vector(e2.pos)
    #         if dist < d:
    #             d = dist
    #             tcoll = e.frameId
    #     else:
    #         print("could not find %s in e2" % e.name)
    # return tcoll


def updateTimeIndicator( context, parabola, sphere, timeOffset=0 ):
    frame_current = context.scene.frame_current
    bpy.ops.object.select_pattern( pattern = sphere.name, extend=False )
    sphere.animation_data_clear()
    log = parabola.physacq.log
    for state in log:
        frameId = (state.frameId - timeOffset)/3
        if frameId < context.scene.frame_start or not frameId.is_integer():
            continue
        # print( "update: setting ", frameId )
        context.scene.frame_set( frameId )
        sphere.location = Vector(state.pos) - Vector(parabola.parent.physacq.groupOrigin)
        bpy.ops.anim.keyframe_insert()
    context.scene.frame_set( frame_current )


def loadParabolas(input, context, inputId):
    scene = context.scene
    # input2names = matchInput(input, scene)
    bpy.ops.object.select_all(action='DESELECT')

    positionsByName = {}
    c = Vector((0.,0.,0.)) # empty centroid
    parabolaName = 'pa.' + str(inputId) + '.' # group name (empty)
    parabolas = [] # blender objects
    for inputName in input.keys():
        # name = input2names[ inputName ]
        # cube = scene.objects[ name ]
        # cube.animation_data_clear()
               
        positions = []
        log = input[inputName]['log']
        for frameId in sorted(log):
            positions.append( log[frameId]["pos"] )
            #linear_velocity = log[sFrameId]["linVel"]

        parabolas.append( MakePolyLine(scene, 'pa.'+ str(inputId) + '.' + inputName+".curve", positions) )
        parabola = parabolas[-1]
        parabola.game.physics_type = 'NO_COLLISION'
        bpy.ops.object.select_pattern( pattern = parabolas[-1].name, extend=False )
        bpy.ops.object.origin_set( type='ORIGIN_GEOMETRY')
        parabolaName += inputName
        c += parabola.location

        for frameId in sorted(log):
            entry = parabola.physacq.log.add()
            entry.name = str(frameId)
            entry.frameId = frameId
            entry.pos = log[frameId]['pos']
            entry.pose = log[frameId]['pose']
            entry.linVel = log[frameId]['linVel']
            entry.angVel = log[frameId]['angVel']
        parabola.physacq.objName = inputName
        parabola.physacq.mass = input[inputName]['mass']
        parabola.physacq.c = input[inputName]['c']
        print( 'c[%s]: ' %inputName, parabola.physacq.c )

        positionsByName[ inputName ] = positions

    # empty = bpy.data.objects.new(parabolaName, None)
    c /= len(parabolas)
    tc = getCollisionTime( parabolas[0], parabolas[1] )
    print('tc: %d', tc )
    bpy.ops.object.empty_add( type='SPHERE', radius=0.3, location = c )
    empty = context.active_object
    empty.name = parabolaName
    # scene.objects.link( empty )
    # empty.location = c
    empty.physacq.groupOrigin = c
    empty.scale = (0.3,0.3,0.3)
    bpy.ops.object.select_pattern( pattern=empty.name, extend = False )
    bpy.ops.object.transform_apply(scale=True)
    empty.lock_rotation[0] = True
    empty.lock_rotation[1] = True
    empty.game.physics_type = 'NO_COLLISION'
    
    for p in parabolas:
        p.parent = empty
        p.location -= empty.location
        p.hide_render = True

    bpy.ops.anim.keying_set_active_set(type='LocRotScale')
    for i, inputName in enumerate(input.keys()):
        parabola = parabolas[i]
        # indicator sphere
        sphereName = parabola.name + ".time";
        bpy.ops.mesh.primitive_uv_sphere_add()
        sphere = context.active_object
        sphere.name = sphereName
        sphere.scale = Vector((0.02, 0.02, 0.02))
        sphere.parent = empty
        sphere.game.physics_type = 'NO_COLLISION'
        sphere.hide_render = True

        materialName = createMaterialName( sphere.name )
        if materialName not in bpy.data.materials:
            material = createMaterial( sphere )
        else:
            material = bpy.data.materials[ materialName ]
        sphere.data.materials.append( material )
        if inputId == 1:
            material.diffuse_color = (0.,1.,0.)
        elif inputId == 2:
            material.diffuse_color = (0.,0.,1.)            
        else:
            material.diffuse_color = (1.,0.,0.)
        material.diffuse_intensity = 0.9
        print(material)

        updateTimeIndicator( context, parabolas[i], sphere )
        #log = input[inputName]['log']
        #for frameId in log.keys():
        #    context.scene.frame_set( frameId )
        #    sphere.location = log[frameId]['pos'] - empty.location
        #    bpy.ops.anim.keyframe_insert()


def rigRecon(input, context, timeOffset, rot, transl):
    for name in input.keys():
        entry = input[name];
        bpy.ops.object.select_pattern(pattern=name+"*", extend=False )
        print( "pattern: %s" % name+'*')
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
            obj.location = state['pos']
            obj.rotation_mode = 'QUATERNION'
            q = Quaternion(state['pose'])
            if prevPose:
                if q.dot(prevPose) < 0:
                    q.negate()
                    #print('newq:',q)
                # else:
                #     print('dotok:', prevPose, q, q.dot(prevPose))
            obj.rotation_quaternion = q
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
            bpy.ops.anim.keying_set_active_set( type='BUILTIN_KSI_LocRot' )
            bpy.ops.anim.keyframe_insert()
            #obj.keyframe_insert(index=-1)
            #obj.keyframe_insert(data_path="location", index=-1, frame=frameId)
            #obj.keyframe_insert(data_path="rotation", index=-1, frame=frameId)
            prevPose = q.copy()
            prevEuler = obj.rotation_euler.copy()