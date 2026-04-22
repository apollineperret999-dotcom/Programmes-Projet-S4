import bpy
import csv
import os
from mathutils import Vector

# on importe les coordonnées du fichier csv 
blend_dir = bpy.path.abspath("//")
csv_file_path = os.path.join(blend_dir,"trajectoire_complete_fusionnee.csv")

if not os.path.exists(csv_file_path):
    raise FileNotFoundError("CSV introuvable")


points = []

# on lit le fichier csv afin de pouvoir les utiliser dans notre script
with open(csv_file_path, 'r') as file:
    reader = csv.reader(file, delimiter=';')
    
    for row in reader:
        if len(row) < 3:
            continue
            
        try:
            x = float(row[1].replace(",", "."))
            y = float(row[2].replace(",", "."))
            z = float(row[3].replace(",", "."))
            points.append(Vector((x,y,z)))
        except:
            continue

# création de la courbe de la trajectoire suivie  et on l'affiche dans Blender

curve_data = bpy.data.curves.new(name="DroneTrajectory", type='CURVE')
curve_data.dimensions = '3D'

curve_data.twist_mode = 'Z_UP'

polyline = curve_data.splines.new('POLY')
polyline.points.add(len(points)-1)

for i, point in enumerate(points):
    polyline.points[i].co = (point.x, point.y, point.z, 1)

curve_object = bpy.data.objects.new("DroneTrajectory", curve_data)
bpy.context.collection.objects.link(curve_object)


drone = bpy.data.objects.get("Base Drone")

if drone is None:
    raise ValueError("Drone introuvable")


# suppression anciennes contraintes liées au drone ( permet de reprendre de zéro si on relance le progamme)
for c in drone.constraints:
    drone.constraints.remove(c)

# ajout de la contrainte Follow Path (pour que le drone suive la courbe)
constraint = drone.constraints.new(type='FOLLOW_PATH')
constraint.target = curve_object

#contrainte pour que le drone tourne et s'incline dans les virages
constraint.use_curve_follow = True
constraint.forward_axis = 'FORWARD_Y'
constraint.up_axis = 'UP_Z'

# durée de l'animation
scene = bpy.context.scene
scene.frame_start = 1
scene.frame_end = 1000

# activer le mode path sur la courbe pour que le drone puisse la suivre 
curve_data.use_path = True
curve_data.path_duration = 1000

# animation de la progression sur la courbe
curve_data.eval_time = 0
curve_object.data.keyframe_insert(data_path="eval_time", frame=1)

curve_data.eval_time = 1000
curve_object.data.keyframe_insert(data_path="eval_time", frame=1000)



