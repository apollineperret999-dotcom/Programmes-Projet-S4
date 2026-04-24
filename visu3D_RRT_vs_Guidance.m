clear; clc; close all;

%% 1. CHARGEMENT DE L'ENVIRONNEMENT
% Lecture du fichier contenant les coordonnées de l'environnement
filename = 'city_data_final.csv';
data = readtable(filename);

% Suppression des lignes contenant des valeurs manquantes
data(any(ismissing(data), 2), :) = [];

% Vérification que le fichier n'est pas vide
if isempty(data)
    error('Le tableau est vide : vérifier le fichier CSV.');
end

figure;
hold on;
grid on;


%% 2. AFFICHAGE DES BÂTIMENTS

for i = 1:height(data)
    
    % Récupération des paramètres de chaque bloc représentant des obstacles
    % gestion de la rotation et du référentiel de chaque bloc pour les
    % représenter alignés comme dns l'environnement réel
    cx = data.OriginX(i);   % position centre X
    cy = data.OriginY(i);   % position centre Y
    cz = data.OriginZ(i);   % position centre Z
    
    w = data.Width(i);      % largeur
    d = data.Depth(i);      % profondeur
    h = data.Height(i);     % hauteur
    
    theta = data.RotZ(i);   % rotation autour de Z

    X = [-w/2  w/2  w/2 -w/2 -w/2  w/2  w/2 -w/2];
    Y = [-d/2 -d/2  d/2  d/2 -d/2 -d/2  d/2  d/2];
    Z = [-h/2 -h/2 -h/2 -h/2  h/2  h/2  h/2  h/2];

    % Matrice de rotation autour de l'axe Z -> alignement des bâtiments 
    R = [cos(theta) -sin(theta) 0;
         sin(theta)  cos(theta) 0;
         0           0          1];

    % Application de la rotation
    verts_rot = R * [X; Y; Z];

    % Translation pour positionner le bâtiment dans la scène
    Xr = verts_rot(1,:) + cx;
    Yr = verts_rot(2,:) + cy;
    Zr = verts_rot(3,:) + cz;


    faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

    % Affichage de chaque bâtiment
    patch('Vertices',[Xr' Yr' Zr'], 'Faces', faces, ...
          'FaceColor', [0.6 0.8 1], 'EdgeColor', 'k', 'FaceAlpha', 0.7);
end


%% 3. IMPORT DE LA TRAJECTOIRE
% Chargement de la trajectoire obtenue par RRT* 
trajData = readtable(['trajectoire_complete_fusionnee.xlsx']);




% Extraction des coordonnées du fichier excel
% on gère automatiquement le nom des colonnes
if ismember('X', trajData.Properties.VariableNames)
    x = trajData.X;
    y = trajData.Y;
    z = trajData.Z;
else
    % Cas où les colonnes ne sont pas nommées 
    x = trajData{:,1};
    y = trajData{:,2};
    z = trajData{:,3};
end

% Si l'altitude z n'est pas précisée, on ajoute une altitude nulle
if isempty(z)
    z = zeros(size(x));
end


%% 3.5 GUIDANCE MODEL
% Cette partie reconstruit une dynamique réaliste du drone
% en prenant en compte l'inertie réelle du drone
%comme vu dans un exemple Matlab

% Gestion du temps
if ismember('Time', trajData.Properties.VariableNames)
    t = trajData.Time;
else
    dt = 0.1; % hypothèse d'un pas de temps constant 
    t = (0:length(x)-1)' * dt;
end

% Calcul des vitesses par dérivation numérique
vx = gradient(x, t);
vy = gradient(y, t);
vz = gradient(z, t);

% Norme de la vitesse
speed = sqrt(vx.^2 + vy.^2 + vz.^2);

% Calcul de l'orientation (yaw)
if ismember('Yaw', trajData.Properties.VariableNames)
    yaw = trajData.Yaw;
else
    yaw = atan2(vy, vx); 
end


%% PARAMÈTRES DU MODÈLE
% Constantes de temps représentant l'inertie et la réactivité du drone
tau_pos = 1.5;   % dynamique de position
tau_yaw = 0.8;   % dynamique d'orientation

% Initialisation des variables dynamiques
x_dyn = zeros(size(x));
y_dyn = zeros(size(y));
z_dyn = zeros(size(z));
yaw_dyn = zeros(size(yaw));

% Conditions initiales
x_dyn(1) = x(1);
y_dyn(1) = y(1);
z_dyn(1) = z(1);
yaw_dyn(1) = yaw(1);


%% SIMULATION DYNAMIQUE
% Modèle simplifié de suivi de trajectoire au 1er ordre 
% on tuilise un modèle low fidelity moins précis qu'un modèle de drone réel
for i = 2:length(t)
    dt = t(i) - t(i-1);

    % Suivi de position avec retard : utilisation d'un filtre
    x_dyn(i) = x_dyn(i-1) + dt * (x(i-1) - x_dyn(i-1)) / tau_pos;
    y_dyn(i) = y_dyn(i-1) + dt * (y(i-1) - y_dyn(i-1)) / tau_pos;
    z_dyn(i) = z_dyn(i-1) + dt * (z(i-1) - z_dyn(i-1)) / tau_pos;

    % Suivi de l'orientation (yaw)
    yaw_error = wrapToPi(yaw(i-1) - yaw_dyn(i-1));
    yaw_dyn(i) = yaw_dyn(i-1) + dt * yaw_error / tau_yaw;
end


%% 4. TRACÉ DES TRAJECTOIRES
% Trajectoire issue du RRT*
plot3(x, y, z, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Trajectoire planifiée');

% Trajectoire réaliste d'après le low fidelity guidance model
plot3(x_dyn, y_dyn, z_dyn, 'b-', 'LineWidth', 2, 'DisplayName', 'Trajectoire réaliste');

% Points de départ et d'arrivée définis lors de la création de la ville
scatter3(x(1), y(1), z(1), 80, 'g', 'filled', 'DisplayName', 'Départ');
scatter3(x(end), y(end), z(end), 80, 'r', 'filled', 'DisplayName', 'Arrivée');


%% 5. MISE EN FORME
view(3);              % vue 3D
axis equal;           % échelle identique sur tous les axes
camlight;             % éclairage
lighting gouraud;     % rendu plus lisse

xlabel('X');
ylabel('Y');
zlabel('Z');

legend;
title('Trajectoire du drone dans l''environnement 3D');

hold off;