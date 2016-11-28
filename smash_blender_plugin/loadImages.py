import bpy
import math
from mathutils import Vector
import os

def clearKeyFrames():
    for object in bpy.data.objects:
        if ( object.type == 'MESH' ):
            object.hide=False
            object.animation_data_clear()

def removeByName( name ):
	bpy.ops.object.select_pattern(pattern=name,extend=False)
	bpy.ops.object.delete(use_global=False)
	if name in bpy.data.meshes:
		mesh = bpy.data.meshes[name]
		print("removing mesh", mesh)
		bpy.data.meshes.remove(mesh)

def find3DViewIndex():
    for i,area in enumerate(bpy.context.screen.areas):
        if area.type == 'VIEW_3D':
            return i
    print("error, no 3D view")
    return -1

def cursorToLocation( loc ):
    areaId = find3DViewIndex()
    view3d = bpy.context.screen.areas[areaId].spaces[0]
    view3d.cursor_location = loc

def clearSelection():
	bpy.ops.object.select_all(action='DESELECT')
	
def removeByName( name ):
	bpy.ops.object.select_pattern(pattern=name)
	bpy.ops.object.delete(use_global=False)
	if name in bpy.data.meshes:
		mesh = bpy.data.meshes[name]
		print("removing mesh", mesh)
		bpy.data.meshes.remove(mesh)
		
def selectChildrenRecursive(obj_parent):
	selection = [];
	for obj in obj_parent.children:
		selectChildrenRecursive(obj)
		if obj.type != 'EMPTY':
			obj.select = True
			selection.append( obj.name );
	return selection;

def addHideKeyFrame( current_frame, doHide ):
	bpy.context.scene.frame_set(current_frame) 
	bpy.context.active_object.hide = doHide
	bpy.context.active_object.hide_render = doHide
	bpy.context.active_object.keyframe_insert( data_path="hide",
	                                           index=-1, 
  	                                         frame=current_frame)
	bpy.context.active_object.keyframe_insert( data_path="hide_render",
 	                                           index=-1, 
  	                                         frame=current_frame)
def setObjectVisibleAtFrame( ob, frameId, noPrevHide = False ):
    clearSelection();
    bpy.context.scene.objects.active=ob;
    if not noPrevHide:
         addHideKeyFrame( frameId-1, True )
    addHideKeyFrame( frameId  , False )
    addHideKeyFrame( frameId+1, True )

clearKeyFrames()
clearSelection()
removeByName("color*")


dir = bpy.path.abspath("//../orig60/")
file_list = os.listdir( dir )
obj_list = sorted( [item for item in file_list if item[-3:] == 'png'] )
files = []
for name in obj_list:
    files.append( {"name":name} )
#files = [{"name":"color_00001.png"}, {"name":"color_00002.png"}, {"name":"color_00003.png"}]
print( files )

cursorToLocation( Vector((0.,10.,0.)) )
#cursorToLocation( Vector((-0.06,9.88,-0.19)) )
bpy.ops.import_image.to_plane(files=files, directory=dir, filter_image=True, filter_movie=True, filter_glob="", align=False, align_offset=0, height=10, use_shadeless=True)
frameId = 0
for im in bpy.data.objects:
    if im.name.find( 'color' ) < 0:
        continue

    im.rotation_euler.x = math.pi/2.

    setObjectVisibleAtFrame( im, frameId )
    frameId += 1
#    setObjectVisibleAtFrame( im, frameId, True )
#    frameId += 1
#    setObjectVisibleAtFrame( im, frameId, True )
#    frameId += 1