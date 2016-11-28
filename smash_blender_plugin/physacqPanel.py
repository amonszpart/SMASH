import bpy
import json
import math
import os
from mathutils import Vector,Quaternion,Euler, Matrix
from bpy_extras.io_utils import ImportHelper

import imp
import rigScene
import blenderIntrinsics
import readScene
#from coordinatesUtil import jsonVector3ToTuple, vector3ToJson, quatToJson
import coordinatesUtil
import pathList
imp.reload(coordinatesUtil)
imp.reload(readScene)
imp.reload(blenderIntrinsics)
imp.reload(rigScene)
imp.reload(pathList)

bl_info = {
    "name": "PhyAcq",
    "description": "Acquisition of physics properties",
    "author": "Aron Monszpart",
    "version": (0,0,1),
    "blender": (2, 6, 0),
#    "api": 31236,
    "location": "View3D > Properties > Physics",
    "warning": '', # used for warning icon and text in addons panel
    "wiki_url": "http://www.blensor.org",
    "category": "System"}


class PhysacqPathListItem(bpy.types.PropertyGroup):
    """ Group of properties representing an item in the list """

    name = bpy.props.StringProperty(
           name="Name",
           description="Name",
           default="Untitled")

    path = bpy.props.StringProperty(
           name="Path",
           description="",
           default="")
    

def physacq_update_time(self, context):
    # self is object.physacq
    print("update_time", self )
    parabolas = []
    for ob in context.scene.objects:
        if ob.parent == context.active_object:
            if ob.name.find('time') > 0:
                sphere = ob
                parabolaName = sphere.name[ 0 : sphere.name.find('time')-1 ]
                print("name:" , parabolaName)
                parabola = context.scene.objects[parabolaName]
                rigScene.updateTimeIndicator(context, parabola, sphere, self.timeOffset * 3)
                parabolas.append( parabola )
    print( rigScene.getCollisionTime( parabolas[0], parabolas[1] ) )


class PhysacqLogGroup(bpy.types.PropertyGroup):
    frameId = bpy.props.IntProperty(default=0)
    pos     = bpy.props.FloatVectorProperty(default=[0.,0.,0.])
    linVel  = bpy.props.FloatVectorProperty(default=[0.,0.,0.])
    angVel  = bpy.props.FloatVectorProperty(default=[0.,0.,0.])
    pose    = bpy.props.FloatVectorProperty(size=4, default=[0.,0.,0.,0.])
    
    # we're overloading the () operator so that instance() returns self.value
#    def __call__(self):
#        return int(self.value)
bpy.utils.register_class(PhysacqLogGroup)

    
class PhysacqObjectSettings(bpy.types.PropertyGroup):
    
    # Enable flag
    is_actor = bpy.props.BoolProperty(name = "Is Physacq Actor")
    
    # Shape
    shape = bpy.props.EnumProperty(name = "Shape", items= (('BOX', 'Box', 'Cuboid shape'),
                                                           ('SPHERE', 'Sphere', 'Spherical shape'),
                                                           ('ELLIPSOID', 'Ellipsoid', 'Ellipsoid shape'), 
                                                           ('BOX', 'Box', 'Cuboid shape'))
                                                           )
    # Mass                                                                            
    mass = bpy.props.FloatProperty(name = "Mass")
    
    # SMASH
    timeOffset = bpy.props.IntProperty(name='TimeOffset', update=physacq_update_time)
    delay = bpy.props.IntProperty(name='Delay')
    log = bpy.props.CollectionProperty(type=PhysacqLogGroup)
    objName = bpy.props.StringProperty( name="objectName")
    c = bpy.props.FloatProperty( name="CoR")
    groupOrigin = bpy.props.FloatVectorProperty( default=[0.,0.,0.])

bpy.utils.register_class(PhysacqObjectSettings)


class PhysacqSceneSettings( bpy.types.PropertyGroup):
    savePath = bpy.props.StringProperty(name = "Save path")
    loadPath = bpy.props.StringProperty(name = "Load path")
    # Rig start
    rig_start = bpy.props.IntProperty( name = "Rig start frameId" )
    rig_path = bpy.props.StringProperty( name = "Rig path" )
    
bpy.utils.register_class(PhysacqSceneSettings) 


class OBJECT_PT_physacq(bpy.types.Panel):
    bl_label = "SMASH"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "physics"
    
    bpy.types.Object.physacq = bpy.props.PointerProperty(type=PhysacqObjectSettings)
    bpy.types.Scene.physacq  = bpy.props.PointerProperty(type=PhysacqSceneSettings)

    def initScene(self,context):
        pass
        #if not len(context.scene.physacq.savePath):
        #    context.scene.physacq['savePath'] = "//cuboids.json";
        #if not len(context.scene.physacq.loadPath):
        #    context.scene.physacq['loadPath'] = "//cuboids.json";
#        if 'loadPath' not in bpy.context.scene:
        #self.loadPath = "//cuboids.json";
   
    @classmethod
    def poll(cls, context):
        return True #context.active_object is not None
 
    def draw_header(self, context):
        layout = self.layout
        layout.label(text="", icon="PHYSICS")   
 
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        obj = context.object
        physacq = obj.physacq # our own propertygroup
        physacqScene = scene.physacq
        self.initScene( context )
 
        # Rig frame       
        box = layout.box() 
        row = box.row()
        split = row.split(percentage=0.5)
        split.column().prop( physacqScene, "rig_path" )
        split.column().operator( "physacq.set_rig_path", "Set rig path" )
        #column = row.column()
        #column.operator( "physacq.rig_scene", "Rig scene" )

        box.label('Trajectories')
        row = box.row()
        column = row.column()
        column.operator( "physacq.load_parabolas", "Load parabolas" )
        column = row.column()
        column.operator( "physacq.clear_parabolas", "Clear parabolas" )
        if obj.type == 'EMPTY' and obj.name.find('pa') == 0:
            row = box.row()
            row.column().prop(physacq, 'timeOffset')
            row.column().prop(physacq, 'delay')
        row = box.row()
        split = row.split()
        split.prop( physacqScene, "rig_start" )
        split = row.split()
        split.operator("physacq.rig_parabolas", "Rig parabolas" )
            
        if context.active_object.type == 'MESH':
            box = layout.box()
            row = box.row()
            row.prop( physacq, "is_actor" )
            if physacq.is_actor:
                row = box.row()
                split = row.split(percentage=0.5)
                col_left = split.column()
                col_right = split.column()
                col_left.prop( physacq, "shape" )
                col_right.prop( physacq, "mass" )

                # save
                box = layout.box()
                box.label('Keyframes')
                row = box.row()
                row = box.row()
                row.prop( physacqScene, "savePath" )
                row = box.row()
                row.operator("object.saveposes", text="Save poses")
                
                # load
                row = box.row()
                row.prop(physacqScene, "loadPath")
                row = box.row()
                row.operator("physacq.set_keyframes_path", "Browse")
                row.operator("object.loadposes", text="Load poses")

        
        # Intrinsics
        layout.row().separator()

        if context.active_object.type == 'CAMERA':
            box = layout.box()
            box.label('Intrinsics')
            row = box.row()
            row.operator( "object.show_intrinsics", text="Show intrinsics" )
      
       
        #col_left.prop( object.saveposes, "filepath", text="")
        #col_right.prop(self, "filepath")

class OBJECT_OT_showIntrinsics(bpy.types.Operator):
    bl_label = "Show intrinsics operator"
    bl_idname = "object.show_intrinsics"
    bl_description = "Show intrinsics"
    
    def execute(self, context):
        self.report({'INFO'},'Executing showIntrinsics');
        cam = context.active_object
        if not cam.type == 'CAMERA':
            self.report( {'INFO'}, 'Please select a camera' );
            return {'FINISHED'}
        K = blenderIntrinsics.getIntrinsicMatrixFromCamera(cam.data)
        print( K )
        K2 = Matrix(
            ((1186.5,    0,    630.1),
            (    0  ,  1186.6, 358.9),
            (    0  ,    0,      1. )))
        blenderIntrinsics.setIntrinsicMatrix( K2, cam, context )
        return {'FINISHED'}


class OBJECT_OT_loadposes(bpy.types.Operator):
    bl_label = "Load poses Operator"
    bl_idname = "object.loadposes"
    bl_description = "Load poses from json as keyframes for correspondingly named objects"
    
    def execute(self, context):
        self.report({'INFO'},'Executing loadPoses');
        fp = bpy.path.abspath( context.scene.physacq.loadPath )
        input = readScene.readFullScene( fp )
        readScene.addKeyFrames( input, context )
        return {'FINISHED'}


class OBJECT_OT_saveposes(bpy.types.Operator):
    bl_label = "Save poses Operator"
    bl_idname = "object.saveposes"
    bl_description = "Save keyframed poses to json"

#    filepath = bpy.props.StringProperty( subtype="FILE_PATH" )

    # from blender to ours: x, -z, y
    coordChangeM = Matrix(((1,0,0), (0,0,-1), (0,1,0)));
    
    def execute(self, context):
        data = []

        objId = 0        
        for ob in bpy.data.objects:
            if ( ob.physacq.is_actor ):
                print( ob.name )
                o = {}
                o['name'] = ob.name
                o['objId'] = objId
                o['shape'] = ob.physacq.shape
                o['size'] = { "x": ob.dimensions[0], "y": ob.dimensions[1], "z": ob.dimensions[2] }
                o['mass'] = ob.physacq.mass

                log = {}
                
                if not ob.animation_data:
                    continue
                
                action = ob.animation_data.action
                for fcu in action.fcurves:
                    #print( "fcu: ", fcu )
                    #print( "prop: ", fcu.data_path )
                    #print( "propId: ", fcu.array_index )
                    
                    if fcu.data_path == 'location':
                        logKey = 'pos'
                    elif fcu.data_path == 'rotation_euler':
                        logKey = 'pose'
                    elif fcu.data_path == 'rotation_quaternion':
                        logKey = 'pose'
                    elif fcu.data_path == 'scale':
                        continue
                    else:
                        print("Can't interpret keyframe type, attention!!!",fcu.data_path)
                        continue
                        
                    if fcu.array_index == 0:
                        subKey = 'x'
                    elif fcu.array_index == 1:
                        subKey = 'y'
                    elif fcu.array_index == 2:
                        subKey = 'z'
                    elif fcu.array_index == 3:
                        subKey = 'w'
                    else:
                        print("Unprepared!")
                        
                    keyframe_points = fcu.keyframe_points
                    for entry in keyframe_points:
                        #print( entry.co )
                        #print( "key:", str(int(entry.co[0])) )
                        frameId = str(int(entry.co[0]));
                        if frameId not in log:
                            log[ frameId ] = {}
                        if logKey not in log[frameId]:
                            log[frameId][logKey] = {}
                        log[ frameId ][ logKey ][ subKey ] = entry.co[1]
                prevKey = None
                prevQ = None
                for key,frame in log.items():
                    if 'pos' in log[key]:
                        pos = Vector( coordinatesUtil.jsonVector3ToTuple( log[ key ]['pos'] ) )
                        log[key]['pos'] = coordinatesUtil.vector3ToJson( self.coordChangeM * pos )

                    if 'pose' not in log[key]:
                        continue

                    #print( 'ob.rotation_mode:', ob.rotation_mode)
                    if ob.rotation_mode == 'XYZ':
                        v = Vector( coordinatesUtil.jsonVector3ToTuple( log[ key ]['pose'] ) )
                        eul = Euler(v,ob.rotation_mode)
                        q = eul.to_quaternion()
                    elif ob.rotation_mode == 'QUATERNION':
                        entry = log[key]['pose']
                        print(entry)
                        q = Quaternion((entry['x'], entry['y'], entry['z'], entry['w'])) # yes, array_index==3 was parsed to w
                    if prevQ:
                        dotProd = q.dot( prevQ )
                        if dotProd < 0:
                            print("dot: ", dotProd)
                            #q.negate()
                    
                    #print( "q: %s" % (q.__repr__()) )
                    #log[ key ] ['pose'] = { 'x' : q[1], 'y' : -q[3], 'z': q[2], 'w' : q[0] }
                    log[key]['pose'] = coordinatesUtil.quatToJson(coordinatesUtil.quatFromBlender(q))
                    if prevKey:
                        #print( "check: ", log[prevKey]['pose'], "\n",log[key]['pose'] )
                        q2 = Quaternion(q)
                        q2.negate()
                        #print( "q:", q, "negative: ", q2 )
                    
                    prevKey = key
                    prevQ = q
                    
                o['log'] = log
                data.append( o )
                objId += 1

        if not len(data):
            self.report({'WARNING'}, "No keyframes, not saving" )
            return {'FINISHED'}
        
        fp = context.scene.physacq.savePath
        if len(bpy.path.basename(fp)) == 0:
            fp = "//cuboids.json"
            
        if fp.find("//") >= 0:
            fp = bpy.path.abspath(fp)
        self.report({'INFO'}, "Saving poses to %s" % (fp) )

        f = open( fp, 'w' )
        json.dump( data, f )
        f.close()
        
        return {'FINISHED'}


class SCENE_OT_setRigPath(bpy.types.Operator, ImportHelper):
    bl_label = "Set rig input path"
    bl_idname = "physacq.set_rig_path"
    bl_description = "Change rig input path"
    
    # From ImportHelper. Filter filenames.
    filename_ext = ".json"
    filter_glob = bpy.props.StringProperty(default="*.json", options={'HIDDEN'})
    filepath = bpy.props.StringProperty(name="File Path", maxlen=2048, default="//multipletraj.json")
    
    def execute(self, context):
        physacqScene = context.scene.physacq
        physacqScene.rig_path = os.path.relpath(self.filepath)
        print('filepath: %s' % self.filepath )
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}
    
    
class SCENE_OT_setLoadKeyframesPath(bpy.types.Operator, ImportHelper):
    bl_label = "Set load keyframes path"
    bl_idname = "physacq.set_keyframes_path"
    bl_description = "Change keyframes input path"
    
    # From ImportHelper. Filter filenames.
    filename_ext = ".json"
    filter_glob = bpy.props.StringProperty(default="*.json", options={'HIDDEN'})
    filepath = bpy.props.StringProperty(name="File Path", maxlen=2048, default="//cuboids.json")
    
    def execute(self, context):
        physacqScene = context.scene.physacq
        physacqScene.loadPath = os.path.relpath(self.filepath)
        print('FilePath: %s' % self.filepath)
        return {'FINISHED'}
    
    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


class SCENE_OT_rigscene(bpy.types.Operator):
    bl_label = "Rig scene"
    bl_idname = "physacq.rig_scene"
    bl_description = "Rig scene for game engine at a certain frameId"
    
    def execute(self, context):
        self.report({'INFO'},'Executing rigscene 5');
        physacqScene = context.scene.physacq
        fp = physacqScene.rig_path
        fp = bpy.path.abspath( fp )

        input = readScene.readFullScene( fp )
        print( 'have %d input objects' % len(input) )
        rigScene.rigScene( input, physacqScene.rig_start, context.scene )
        return {'FINISHED'}


class SCENE_OT_loadParabolas(bpy.types.Operator):
    bl_label = "Load parabolas"
    bl_idname = "physacq.load_parabolas"
    bl_description = "Load parabolas"
    
    def execute(self, context):
        physacqScene = context.scene.physacq
        for inputId,elem in enumerate(context.scene.physacq_path_list):
#            fp = physacqScene.rig_path
            fp = bpy.path.abspath( elem.path )

            input = readScene.readFullScene( fp )
            print( 'have %d input objects' % len(input) )
            rigScene.loadParabolas( input, context, inputId )
            
        return {'FINISHED'}


class SCENE_OT_clearParabolas(bpy.types.Operator):
    bl_label = "Clear parabolas"
    bl_idname = "physacq.clear_parabolas"
    bl_description = "Clear parabolas"
    
    def execute(self, context):
        bpy.ops.object.select_pattern(pattern='pa.*',extend=False)
        bpy.ops.object.delete()
            
        return {'FINISHED'}


class SCENE_OT_rigParabolas(bpy.types.Operator):
    bl_label = "Rig parabolas"
    bl_idname = "physacq.rig_parabolas"
    bl_description = "Rig parabolas"
    
    def execute(self, context):
        rigScene.rigParabolas(context, context.scene.physacq.rig_start)
        bpy.ops.object.select_all( action='DESELECT')
        return {'FINISHED'}


class PT_PhysacqTrajList(bpy.types.Panel):
    """Demo panel for UI list Tutorial"""
    
    bl_label = "SMASH Trajectory paths"
    bl_idname = "SCENE_PT_PHYSACQ_TRAJECTORY_LIST"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "physics"
 
 
    def draw(self, context):
        layout = self.layout
        scene = context.scene
        
        row = layout.row()
        row.template_list("MY_UL_List", "The_List", scene, "physacq_path_list", scene, "list_index" )
 
        row = layout.row()
        row.operator('physacq_path_list.new_item', text='+')         
        row.operator('physacq_path_list.delete_item', text='-')
        row.operator('physacq_path_list.duplicate_item', text='++')
        #row.operator('physacq_path_list.move_item', text='^').direction = 'UP'         
        #row.operator('physacq_path_list.move_item', text='v').direction = 'DOWN'         
            
        if len(scene.physacq_path_list) > 0 and 0 <= scene.list_index < len(scene.physacq_path_list):
            item = scene.physacq_path_list[scene.list_index]
 
            row = layout.row()
            row.prop(item, "name")
            row.prop(item, "path")


def register():
    bpy.utils.register_module(__name__)
    pathList.register()
    bpy.types.Scene.physacq_path_list = bpy.props.CollectionProperty(type = PhysacqPathListItem )
    bpy.types.Scene.list_index = bpy.props.IntProperty(name = "Index for my_list", default = 0)


def unregister():
    bpy.utils.unregister_module(__name__)
    pathList.unregister()


if __name__ == "__main__":
    register()