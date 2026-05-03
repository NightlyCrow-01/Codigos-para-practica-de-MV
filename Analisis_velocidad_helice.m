clear; clc; close all;

%% CÁLCULO DE LA VELOCIDAD ACELERADA PARA EL CASO DEL RIZO
%Definimos los parámetros del rizo y del avión:
R=450;
T_avion=15800;

W   = 32000;   
S   = 20.0;  
rho = 1.225;
g   = 9.81;     

CD0 = 0.025;
k   = 0.050;

%El cálculo se realizará mediante una EDO con la función "ode45", ya que
%tanto el drag como el ángulo dependen de la velocidad, que pasa a no ser
%constante a causa del déficit y exceso de empuje.

%Definimos las derivadas de la velocidad lineal y angular en función del
%tiempo, despejando en la ecuación del eje tangencial acelerado, siendo:
% y(1) = V
% y(2) = phi

odefun=@(t,y) [(T_avion - (0.5 * rho * y(1)^2* S*(CD0 + k * ...
    ((W/(0.5 * rho * y(1)^2*S)) * ((y(1)^2/(g*R)) + cos(y(2))))^2)) - ...
    W * sin(y(2)))*(g/W); y(1)/R];

%Definimos un espacio de tiempo lo suficientemente largo como para abarcar
%la trayectoria de 0 a pi:
% tspan=linspace(0,20,1000);

%Vemos para qué posición del vector de tiempo se da el final de la
%trayectoria, es decir, phi = 180. En este caso es para 
%t = 15.4354354354354 s, obtenido de otra iteración, así que cambiamos el 
% rango temporal:
tspan=linspace(0,15.4354354354354,1000);


%De condiciones iniciales del problema partimos de la velocidad V=120 m/s
%y el ángulo inicial de phi = 0:
y0 = [120; 0];

%Se calcula la velocidad y posición angular:
[t, y]=ode45(odefun,tspan,y0);
V_sol = y(:,1);
phi_sol = y(:,2);


phi_deg=rad2deg(phi_sol);

figure()
plot(phi_deg, V_sol)
xlabel('\phi [°]')
ylabel('V [m/s]')
title('Velocidad no uniforme')
grid on

%% VARIACIÓN DEL EMPUJE CON LA VELOCIDAD
%Si asumimos que el motor del avión tiene una potencia fija y empezó volando 
%a su velocidad máxima (V=120 m/s):
Potencia=120*T_avion;

%Si ahora tomamos que es un turbohélice de rendimiento = 1, su empuje 
%cambiará inversamente en función de la velocidad, tanto cuando frena, 
%como cuando acelera:
T_helice=Potencia./V_sol;

%El empuje requerido:
V1=120;

q1      = 0.5 * rho * V1^2; 
CL1 = (W/(q1*S)) * ((V1^2/(g*R)) + cos(phi_sol));
CD1 = CD0 + k * CL1.^2;
D1  = q1.* S.* CD1;
T1  = D1 + W * sin(phi_sol);

%Graficamos los empujes requerido, en función de la velocidad, y el del
%turbofán (empuje constante):
figure()
plot(phi_deg, T_helice./1000)
hold on
plot(phi_deg, T1./1000)
fplot(T_avion./1000,[0,180],'--')
hold off
xlabel('\phi [°]')
ylabel('T [kN]')
legend('T hélice','T requerido','T turbofán')
title('Empuje')
grid on