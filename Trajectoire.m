clear; clc; close all; % charge les données de la ville supprime toutes les variables du workspace, nettoie la fenêtre de commande,ferme toutes les figures ouvertes
%% 1. CHARGEMENT DES DONNÉES DE LA VILLE
filePath = "C:\Users\phoeb\Documents\Matlab 1\city_data_final_no_wires.csv"; % charge les données de la ville
data = readtable(filePath); % transforme le fichier CSV décrivant la ville en une table MATLAB
vars = data.Properties.VariableNames; % récupère la liste des noms de colonnes du tableau data
if ismember('OriginX', vars), xH='OriginX'; yH='OriginY'; zH='OriginZ'; %rend le code compatible avec plusieurs formats de fichiers CSV. Si le CSV contient OriginX / OriginY / OriginZ, le code les utilise
else xH='CenterX'; yH='CenterY'; zH='CenterZ'; end %sinon il utilise CenterX / CenterY / CenterZ
if ~ismember('RotZ', vars), data.RotZ = zeros(height(data), 1); end
%% 2. CRÉATION DE LA CARTE D'OCCUPATION
resolution = 1; %Définit la taille d'un voxel en mètres. Avec 1, chaque cube de l'espace fait 1m × 1m × 1m
omap = occupancyMap3D(resolution); %Crée la carte d'occupation 3D vide
fprintf('Chargement des bâtiments dans la carte... ');
for i = 1:height(data) %Lance une boucle qui parcourt chaque ligne du tableau CSV — donc chaque bâtiment. height(data) retourne le nombre de lignes du tableau.
  w = data.Width(i); d = data.Depth(i); h = data.Height(i); %Récupère les dimensions du bâtiment numéro i : largeur, profondeur, hauteur en mètres.
  cx = data.(xH)(i); cy = data.(yH)(i); cz = data.(zH)(i); %Récupère la position du centre du bâtiment dans l'espace mondial.
  rz = data.RotZ(i); %récupere les angles de rotation des bâtiments 
  [xGrid, yGrid, zGrid] = meshgrid(-w/2:1/resolution:w/2, -d/2:1/resolution:d/2, -h/2:1/resolution:h/2); %génère une grille 3D de points espacés de 1m (= resolution), qui couvre tout le volume du bâtiment de -w/2 à +w/2 en X, -d/2 à +d/2 en Y, -h/2 à +h/2 en Z. On part du centre (0,0,0)
  points_local = [xGrid(:), yGrid(:), zGrid(:)]; %Transforme les 3 grilles 3D en une seule matrice de N lignes × 3 colonnes. Chaque ligne est un point (x, y, z) dans le repère local du bâtiment. 
  R = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1]; %Construit la matrice de rotation autour de l'axe Z. 
  points_world = (R * points_local')' + [cx, cy, cz]; %Applique la rotation puis la translation pour passer du repère local au repère mondial :
  setOccupancy(omap, points_world, 1); % Marque tous ces points dans la carte comme occupés 
end
fprintf('Terminé.\n');
omap.FreeThreshold = 0.5;
inflate(omap, 1);
%% 3. CONFIGURATION DU PLANIFICATEUR RRT*
ss = stateSpaceSE3([-300 400; -400 400; 0 60 ; -1 1; -1 1; -1 1; -1 1]); %créer un espace de recherche pour l'algorithme
sv = validatorOccupancyMap3D(ss, 'Map', omap, 'ValidationDistance', 0.03); Définit comment le planificateur vérifie si une position est libre ou en collision, lie le validateur à une carte d'occupation 3D existante

planner = plannerRRTStar(ss, sv, … % L'algorithme RRT* cherche à construire un arbre de chemins aléatoires tout en optimisant la distance totale.
        'MaxConnectionDistance', 10, ... % Distance maximale autorisée entre deux nœuds de l'arbre.
        'MaxIterations', 20000, … %Nombre maximal de tentatives pour étendre l'arbre
        'GoalReachedFcn', @(~,s,g)(norm(s(1:3)-g(1:3)) < 3), …% distance à laquelle le point d’arrivée est considéré comme atteint
        'GoalBias', 0.7);
% Points de départ et d'arrivée corrigés start = [-110 , -170, 10, 1, 0, 0, 0];
goal = [50 , 150, 10, 1, 0, 0, 0];
if ~isStateValid(sv, start) || ~isStateValid(sv, goal) %Avant de lancer le calcul, on vérifie si les points de départ et d'arrivée ne sont pas à l'intérieur d'un obstacle
  error('Point de départ ou d''arrivée invalide (collision ou hors limites).');
end
rng(1, "twister"); . %Cela garantit que vous obtiendrez le même chemin à chaque exécution du code
fprintf('Calcul de la trajectoire RRT*... ');
[pthObj, solnInfo] = plan(planner, start, goal); % Contient l'objet trajectoire final c’est à dire la liste des points du chemin .
fprintf('Terminé.\n');
%%4. LISSAGE
if solnInfo.IsPathFound %vérifie si un chemin a bien été trouvé par l’algorithme
   rawStates = pthObj.States(:, 1:3); %extrait les coordonnées de position (x, y, z)
   dist_diff = sqrt(sum(diff(rawStates).^2, 2)); %calcule la distance euclidienne entre chaque point consécutif
   rawStates([false; dist_diff < 0.01], :) = []; %supprime les points trop proches pour nettoyer le bruit dans la trajectoire
  
   % Paramétrage de la Spline
   numPointsSpline = 5000; %nombre de points souhaités dans la trajectoire lissée
   t_coords = 1:size(rawStates, 1); %paramètre du temps discret associé aux points initiaux
   tt = linspace(1, size(rawStates, 1), numPointsSpline); %paramétrage plus fin  
   % Calcul de la spline sur les points nettoyés
   splinePath = csaps(t_coords, rawStates', 0.0065, tt); %application de la fonction csaps

   splinePath(:, 1) = start(1:3)'; %correction du point initial :  force le premier point de la spline à être exactement le point de départ
else
   warning('Aucune trajectoire trouvée.'); %message d'avertissement si aucune trajectoire n'a été trouvée
end

%% 5. VISUALISATION FINALE
figure('Color', [0.12 0.12 0.14], 'Name', 'RRT* Path Smoothing');
hold on; axis equal;
% Rendu des bâtiments
for i = 1:height(data)
  w = data.Width(i); d = data.Depth(i); h = data.Height(i);
  cx = data.(xH)(i); cy = data.(yH)(i); cz = data.(zH)(i);
  rz = data.RotZ(i);
  v_local = [-w/2 -d/2 -h/2; w/2 -d/2 -h/2; w/2 d/2 -h/2; -w/2 d/2 -h/2;
             -w/2 -d/2 h/2;  w/2 -d/2 h/2;  w/2 d/2 h/2;  -w/2 d/2 h/2];
  R = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
  v_final = (R * v_local')' + [cx, cy, cz];
  f = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
  patch('Vertices', v_final, 'Faces', f, 'FaceColor', [0.35 0.45 0.6], ...
        'EdgeColor', [0.1 0.1 0.1], 'FaceAlpha', 0.5, 'LineWidth', 0.5);
end
if solnInfo.IsPathFound
   % Affichage du chemin lissé (Spline)
  plot3(splinePath(1, :), splinePath(2, :), splinePath(3, :), ...
        'm-', 'LineWidth', 2, 'DisplayName', 'Trajectoire Lissée');
   scatter3(start(1), start(2), start(3), 100, 'g', 'filled');
  scatter3(goal(1), goal(2), goal(3), 100, 'r', 'filled');
  legend('Bâtiments','','','','','','Chemin RRT*','Chemin Spline','Départ','Arrivée','TextColor','w');
end
view(45, 30); camlight headlight; lighting gouraud;
set(gca, 'Color', [0.12 0.12 0.14], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
% 6. EXPORT POUR BLENDER 
if solnInfo.IsPathFound
   pointsToExport = splinePath';
   numPoints = size(pointsToExport, 1);
   vitesse_moyenne = 5; % m/s
  
   temps = zeros(numPoints, 1);
   yaw = zeros(numPoints, 1); % Angle d'orientation
  
   for i = 1:numPoints
       if i > 1
           %Calcul du temps cumulé
           dist = norm(pointsToExport(i,:) - pointsToExport(i-1,:));
           temps(i) = temps(i-1) + (dist / vitesse_moyenne);
          
           %Calcul de l'orientation (Direction du vecteur de mouvement)
           % On calcule l'angle entre le point actuel et le précédent
           dx = pointsToExport(i,1) - pointsToExport(i-1,1);
           dy = pointsToExport(i,2) - pointsToExport(i-1,2);
           yaw(i) = atan2(dy, dx);
       end
   end
  
   % Le premier point prend la même orientation que le second pour ne pas être à 0
   yaw(1) = yaw(2);
  
   %Créer la table avec Temps, Position et Rotation
   T = table(temps, pointsToExport(:,1), pointsToExport(:,2), pointsToExport(:,3), yaw, ...
             'VariableNames', {'Time', 'X', 'Y', 'Z', 'Yaw'});
  
   filename = 'ltrajectoire_blender_complete1.csv';
   writetable(T, filename);
  
   fprintf('Exportation réussie : %s (Positions + Rotation)\n', filename);
else
   error('Impossible d''exporter : aucune trajectoire trouvée.');
end

