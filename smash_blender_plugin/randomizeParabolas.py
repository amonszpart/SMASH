import bpy
import random
import math

# stir parabolas
context = bpy.context

bbox = [[0, 0.5], [2,3], [0,0.2]] # x, y, z

bpy.ops.object.select_pattern(pattern='pa.*', extend=False)
for ob in context.selected_objects:
    if ob.type == 'EMPTY':
        ob.location.x = random.uniform(bbox[0][0], bbox[0][1])
        ob.location.y = random.uniform(bbox[1][0], bbox[1][1])
        ob.location.z = random.uniform(bbox[2][0], bbox[2][1])
        ob.rotation_euler.z = random.uniform(0,math.pi*2)
        ob.physacq.delay = random.uniform(-120,0)
        ob.physacq.timeOffset = -20
#        if ob.name.find('helmet') >= 0:
#            ob.physacq.timeOffset = 10
#        elif ob.name.find('Doddle') >= 0:
#            ob.physacq.timeOffset = -20
#        else:
#            ob.physacq.timeOffset = -5
       
bpy.ops.physacq.rig_parabolas()