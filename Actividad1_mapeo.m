%Limpieza de pantalla
clear all
close all
clc

%Declaración de variables simbolicas
syms x(t) y(t) th(t) t %Grados de Libertad del robot móvil

%Creamos el vector de posición
xi_inercial= [x; y; th];
disp('Coordenadas generalizadas');
pretty(xi_inercial);

%Creamos el vector de velocidades
xip_inercial= diff(xi_inercial, t); 
disp('Velocidades generalizadas');
pretty(xip_inercial);

%Defino mi vector de posición y matriz de rotación
P(:,:,1)= [x;y;th]; %Viene siendo "xi_inercial"
%Matriz de rotación alrededor del eje z.....
R(:,:,1)= [cos(th)  -sin(th)  0;
           sin(th)   cos(th)  0;
           0         0        1];

%Realizo mi transformación del marco de referencia global al local...
xi_local=R(:,:,1)*P(:,:,1)

%Defino el arreglo de coordenadas inerciales para un tiempo 1
Valorx1 =[-5,-3,5,0,-6,10,9,5,-1,6,5,7,11,20,10,-9,1,3,15,-10]; %Posición inicial eje x

Valory1 = [9,8,-2,0,3,-2,1,2,-1,4,7,7,-4,5,9,-8,1,1,2,0];%Posición inicial eje y

Valorth1 = [-2,63,90,180,-55,45,88,33,21,-40,72,30,360,270,345,8,60,30,199,300];%Orientación inicial del robot

% Iterar sobre los valores iniciales y calcular las transformaciones
for i = 1:length(Valorx1)
    
    x1 = Valorx1(i);
    y1 = Valory1(i);
    th1 = Valorth1(i);

    % Defino mi vector de posición y matriz de rotación para un tiempo 1
    Pos_1 = [x1; y1; th1];
    Rot_1 = [cos(th1), -sin(th1), 0;
             sin(th1), cos(th1), 0;
             0, 0, 1];

    % Realizo mi transformación del marco de referencia inercial al local...
    xi_local_1=Rot_1*Pos_1;

    % Obtengo la magnitud del vector resultante
    magnitud= sqrt(xi_local_1(1)^2 + xi_local_1(2)^2);

    % Compruebo que me devuelva el vector inercial
    inv_Rot_1 = inv(Rot_1);
    xi_inercial_1 = inv_Rot_1*xi_local_1;

    disp(['Coordenadas ', num2str(i), ':']);
    disp(['Valores de x1, y1, th1: ', num2str(x1), ', ', num2str(y1), ', ', num2str(th1)]);
    disp(['Vector del marco de referencia inercial local: ', num2str(xi_local_1')]);
    disp(['Magnitud del vector resultante: ', num2str(magnitud)]);
    disp(['Vector inercial: ', num2str(xi_inercial_1')]);
    fprintf('\n');
end
