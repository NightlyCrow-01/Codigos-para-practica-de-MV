clear; clc; close all;
syms V

%Definimos los parámetros para el rizo:
R=450;
phi=deg2rad(90);
T_avion=15800;

W   = 32000;   
S   = 20.0;  
rho = 1.225;
g   = 9.81;     

CD0 = 0.025;
k   = 0.050;

q      = 0.5 * rho * V^2; 
CL_req = (W/(q*S)) * ((V^2/(g*R)) + cos(phi));
CD_req = CD0 + k * CL_req^2;
D_req  = q * S * CD_req;
T_req_max  = D_req + W * sin(phi);

%Buscamos minimizar el empuje requerido máximo, es decir, para phi = 90º
dT_req_max=diff(T_req_max,V);
V_opt_vec=double(solve(dT_req_max==0));
V_opt=V_opt_vec(2);

%Observamos que el V_opt=0, por lo que el empuje requerido quedaría
%únicamente en función del peso del avión y del ángulo de la trayectoria,
%siendo el máximo el peso del avión cuando el ángulo es de 90º

T_req_min_max=double(subs(T_req_max,V,V_opt));

%Comparamos los empujes requeridos, tanto el original como con la velocidad
%que minimiza el empuje requerido máximo

N   = 1000;
phi_opt = linspace(0,pi,N); 

%A velocidad nula queda en función del peso y ángulo:
T_opt  = W * sin(phi_opt);

phi_opt_deg = rad2deg(phi_opt); 

V1=120;

q1      = 0.5 * rho * V1^2; 
CL1 = (W/(q1*S)) * ((V1^2/(g*R)) + cos(phi_opt));
CD1 = CD0 + k * CL1.^2;
D1  = q1.* S.* CD1;
T1  = D1 + W * sin(phi_opt);

figure()
plot(phi_opt_deg,T_opt./1000)
hold on
plot(phi_opt_deg,T1./1000)
fplot(T_avion./1000,[0,180],'--')
hold off

xlabel('\phi [°]')
ylabel('T [kN]')
legend('Treq,máx,mín','Treq', 'T avión')
title('Rizo')
