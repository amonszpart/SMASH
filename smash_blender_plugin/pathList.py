import bpy
import bpy.props as prop
 
class MY_UL_List(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):

        # We could write some code to decide which icon to use here...
        custom_icon = 'OBJECT_DATAMODE'
 
        # Make sure your code supports all 3 layout types
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.label(item.name, icon = custom_icon)
 
        elif self.layout_type in {'GRID'}:
            layout.alignment = 'CENTER'
            layout.label("", icon = custom_icon)
 
 
class LIST_OT_NewItem(bpy.types.Operator):
    """ Add a new item to the list """
 
    bl_idname = "physacq_path_list.new_item"
    bl_label = "Add a new item"
 
    def execute(self, context):
        context.scene.physacq_path_list.add()
        context.scene.physacq_path_list[-1].name = bpy.path.basename( context.scene.physacq.rig_path )
        context.scene.physacq_path_list[-1].path = context.scene.physacq.rig_path
#        print( context.scene.physacq.rig_path )
 
        return{'FINISHED'}


class LIST_OT_DuplicateItem(bpy.types.Operator):
    """ Add a new item to the list """

    bl_idname = "physacq_path_list.duplicate_item"
    bl_label = "Duplicate current new item"

    @classmethod
    def poll(self, context):
        """ Enable if there's something in the list """
        return context.scene.physacq_path_list and len(context.scene.physacq_path_list) > 0

    def execute(self, context):
        path_list = context.scene.physacq_path_list
        index = context.scene.list_index

        context.scene.physacq_path_list.add()
        context.scene.physacq_path_list[-1].name = path_list[index].name
        context.scene.physacq_path_list[-1].path = path_list[index].path

        return {'FINISHED'}


class LIST_OT_DeleteItem(bpy.types.Operator):
    """ Delete the selected item from the list """
 
    bl_idname = "physacq_path_list.delete_item"
    bl_label = "Deletes an item"
 
    @classmethod
    def poll(self, context):
        """ Enable if there's something in the list """
        return context.scene.physacq_path_list and len(context.scene.physacq_path_list) > 0
 
    def execute(self, context):
        path_list = context.scene.physacq_path_list
        index = context.scene.list_index
 
        path_list.remove(index)
 
        if index > 0:
            index -= 1
 
        return{'FINISHED'}
 

class LIST_OT_MoveItem(bpy.types.Operator):
    """ Move an item in the list """
 
    bl_idname = "physacq_path_list.move_item"
    bl_label = "Move an item in the list"
 
    direction = bpy.props.EnumProperty(
                items=(
                    ('UP', 'Up', ""),
                    ('DOWN', 'Down', ""),))
 
    @classmethod
    def poll(self, context):
        """ Enable if there's something in the list. """
 
        return len(context.scene.physacq_path_list) > 0
 
 
    def move_index(self):
        """ Move index of an item render queue while clamping it. """
 
        index = bpy.context.scene.list_index
        list_length = len(bpy.context.scene.physacq_path_list) - 1 # (index starts at 0)
        new_index = 0
 
        if self.direction == 'UP':
            new_index = index - 1
        elif self.direction == 'DOWN':
            new_index = index + 1
 
        new_index = max(0, min(new_index, list_length))
        index = new_index
 
 
    def execute(self, context):
        list = context.scene.physacq_path_list
        index = context.scene.list_index
 
        if self.direction == 'DOWN':
            neighbor = index + 1
            queue.move(index,neighbor)
            self.move_index()
 
        elif self.direction == 'UP':
            neighbor = index - 1
            queue.move(neighbor, index)
            self.move_index()
        else:
            return{'CANCELLED'}
 
        return {'FINISHED'}


def register():
    bpy.utils.register_class(MY_UL_List)
    bpy.utils.register_class(LIST_OT_NewItem)
    bpy.utils.register_class(LIST_OT_DeleteItem)
    bpy.utils.register_class(LIST_OT_DuplicateItem)
    bpy.utils.register_class(LIST_OT_MoveItem)


def unregister():
    bpy.utils.unregister_class(MY_UL_List)
    bpy.utils.unregister_class(LIST_OT_NewItem)
    bpy.utils.unregister_class(LIST_OT_DeleteItem)
    bpy.utils.unregister_class(LIST_OT_DuplicateItem)
    bpy.utils.unregister_class(LIST_OT_MoveItem)
 
        
if __name__ == "__main__":
    register()