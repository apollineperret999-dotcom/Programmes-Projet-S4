Ces programmes sont ceux que nous avons utilisés pour la simulation de la trajectoire du drone.

Trajectoire_drone_dans_blender.py : programme inséré dans Blender pour tracer la trajectoire et permettre au drone de la suivre durant l'animation

code_matlab_trajectoire_commenté.m : Planification de la trajectoire RRT* et lissage par csaps

visu3D_RRT_vs_Guidance.m : Code commenté de la visualisation 3D dans Matlab de la ville simplifiée sous forme de blocs, et comparaison de la trajectoire issue de l'algorithme RRT* avec la trajectoire issue d'un low fidelity Guidance Model fourni par les ressources Matlab 
Les fichiers city_data_final.csv et trajectoire_complete_fusionnee.xslx ont été utilisé pour importer les coordonnées nécessaires à la modélisation
Voir l'article MathWorks :  
[Guidance Model UAV](https://fr.mathworks.com/help/uav/ug/approximate-high-fidelity-uav-model-with-guidance-model.html)


