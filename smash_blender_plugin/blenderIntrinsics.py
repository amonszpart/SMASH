import bpy
import math
from mathutils import Matrix

def getIntrinsicMatrixFromCamera(camd):
    f_in_mm = camd.lens
    scene = bpy.context.scene
    resolution_x_in_px = scene.render.resolution_x
    resolution_y_in_px = scene.render.resolution_y
    scale = scene.render.resolution_percentage / 100
    sensor_width_in_mm = camd.sensor_width
    sensor_height_in_mm = camd.sensor_height
    pixel_aspect_ratio = scene.render.pixel_aspect_x / scene.render.pixel_aspect_y
    if (camd.sensor_fit == 'VERTICAL'):
        # the sensor height is fixed (sensor fit is horizontal), 
        # the sensor width is effectively changed with the pixel aspect ratio
        s_u = resolution_x_in_px * scale / sensor_width_in_mm / pixel_aspect_ratio 
        s_v = resolution_y_in_px * scale / sensor_height_in_mm
    else: # 'HORIZONTAL' and 'AUTO'
        # the sensor width is fixed (sensor fit is horizontal), 
        # the sensor height is effectively changed with the pixel aspect ratio
        s_u = resolution_x_in_px * scale / sensor_width_in_mm
        s_v = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height_in_mm

    # Parameters of intrinsic calibration matrix K
    alpha_u = f_in_mm * s_u
    alpha_v = f_in_mm * s_v
    maxdim = max(scene.render.resolution_x,scene.render.resolution_y) 
    # shift_x = (K[0][2] - render.resolution_x/2.0)/maxdim
    # shift_x * maxdim + render.resolution_x/2.0 = K[0][2] 
    u_0 = resolution_x_in_px*scale / 2. - camd.shift_x * maxdim
    v_0 = resolution_y_in_px*scale / 2. + camd.shift_y * maxdim
    skew = 0 # only use rectangular pixels

    K = Matrix(
        ((alpha_u, skew,    u_0),
        (    0  ,  alpha_v, v_0),
        (    0  ,    0,      1 )))
    return K

def setIntrinsicMatrix( K, cam, context ):
    camdata = cam.data
    render = context.scene.render

    f_in_mm = camdata.lens
    fx = K[0][0]
    fy = K[1][1]
    if fx > fy:
        render.pixel_aspect_x = fx / fy
        render.pixel_aspect_y = 1.
    else:
        render.pixel_aspect_x = 1.
        render.pixel_aspect_y = fy / fx
    pixel_aspect_ratio = render.pixel_aspect_x / render.pixel_aspect_y
    resolution_x_in_px = render.resolution_x
    resolution_y_in_px = render.resolution_y
    scale = render.resolution_percentage / 100

    if (camdata.sensor_fit == 'VERTICAL'):
        # the sensor height is fixed (sensor fit is horizontal), 
        # the sensor width is effectively changed with the pixel aspect ratio
        
        # fx / f_in_mm = resolution_x_in_px * scale / sensor_width_in_mm / pixel_aspect_ratio
        # sensor_width_in_mm = resolution_x_in_px * scale / fx / pixel_aspect_ratio * f_in_mm
        camdata.sensor_width = resolution_x_in_px * scale / fx / pixel_aspect_ratio * f_in_mm
        # fy / f_in_mm = resolution_y_in_px * scale / sensor_height_in_mm
        # sensor_height_in_mm = resolution_y_in_px * scale / fy * f_in_mm
        camdata.sensor_height = resolution_y_in_px * scale / fy / f_in_mm
        
    else: # 'HORIZONTAL' and 'AUTO'
        # the sensor width is fixed (sensor fit is horizontal), 
        # the sensor height is effectively changed with the pixel aspect ratio
        
        # fx / f_in_mm = resolution_x_in_px * scale / sensor_width_in_mm
        # sensor_width_in_mm = resolution_x_in_px * scale / fx * f_in_mm
        camdata.sensor_width = resolution_x_in_px * scale / fx * f_in_mm

        # fy / f_in_mm = resolution_y_in_px * scale * pixel_aspect_ratio / sensor_height_in_mm
        # sensor_height_in_mm = resolution_y_in_px * scale * pixel_aspect_ratio / fy * f_in_mm
        camdata.sensor_height = resolution_y_in_px * scale * pixel_aspect_ratio / fy * f_in_mm

    maxdim = max(render.resolution_x,render.resolution_y) 
    camdata.shift_x = (render.resolution_x/2.0 - K[0][2])/maxdim
    camdata.shift_y = (K[1][2] - render.resolution_y/2.0)/maxdim
    print( "shift set: %f,%f" % (camdata.shift_x, camdata.shift_y) )

if __name__ == "__main__":
    # Insert your camera name below
    K = getIntrinsicMatrixFromCamera(bpy.data.objects['Camera'].data)
    print(K)
    f = open('intr.txt', 'w')
    f.write( "%f %f %f %f %f %f %f %f %f" % (K[0][0],K[0][1],K[0][2],K[1][0],K[1][1],K[1][2],K[2][0],K[2][1],K[2][2]) )
    f.close()
    
    K = Matrix( ((1186.5,0.,630.1),(0.,1186.6,358.9),(0.,0.,1.)) )
    setIntrinsicMatrix(K,bpy.data.objects['Camera'].data,bpy.context)

#    render = bpy.context.scene.render
#    camdata = bpy.data.cameras[0]
#    K = Matrix( ((1086.,0.,640),(0.,1086,320),(0.,0.,1.)) )
#    camdata.angle_x=2*math.atan(0.5*render.resolution_x/K[0][0]);
#    camdata.angle_y=2*math.atan(0.5*render.resolution_y/K[1][1]);

#    maxdim = max(render.resolution_x,render.resolution_y) 
    # the unit of shiftXY is FOV unit (Lens Shift)
#    camdata.shift_x = (K[0][2] - render.resolution_x/2.0)/maxdim
#    camdata.shift_y = (K[1][2]- render.resolution_y/2.0)/maxdim