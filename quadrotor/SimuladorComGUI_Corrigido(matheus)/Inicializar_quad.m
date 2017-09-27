function Inicializar_quad()

% global quad
% clear global
global quad
% 
% posicao = [0;0;0];      %x, y, z
% orientacao = [0;0;0]; %roll, pitch, yaw
% velocidade_linear = [0;0;0]; %vx, vy, vz
% velocidade_angular_frame_inercial = [0;0;0]; %wx, wy, wz
% velocidade_angular_frame_quad= [0;0;0]; %p, q, r
% aceleracao_linear = [0;0;0]; %ax, ay, az
% aceleracao_angular = [0;0;0]; %wxdot, wydot, wzdot
% 
% roll = orientacao(1);
% pitch = orientacao(2);
% yaw = orientacao(3);
% 
% 
% 
% R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
%     cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
%     -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
% 
% T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
%         0, 1, sin(roll);...
%         sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
%     
%     
% CSI = []; n sei pra que existe 


% m = 0.468;
% g = 9.81;
% dt = 0.02;
% %momento de inercia
% Ixx = 4.856*10^-3;
% Iyy = Ixx;
% Izz = 8.801*10^-3;
% I = [Ixx 0 0;0 Iyy 0; 0 0 Izz];
% %tamanho do braço
% l = 0.225;
% %parametros
% b = 1.140*10^-7;
% k = 2.980*10^-6;

% estados = [posicao;orientacao;velocidade_linear;velocidade_angular_frame_quad];
% estados_medidos = estados;

motor = [0;0;0;0];


K = [6.1405    9.1331    6.1405    9.1331    6.1405    9.1331    1.5782    1.5782    1.5782    0.8000    0.8000    0.8000];
Krollp = K(1);
Krolld = K(2);
Kpitchp = K(3);
Kpitchd = K(4);
Kyawp = K(5);
Kyawd = K(6);
Kcp = K(7:9)';
Kcd = K(10:12)';

Kr = 9656.655720288503;
Kw = 1832.403541405957;


% Para plots
b1 = [-l;0;0];
b2 = [0;-l;0];
b3 = [l;0;0];
b4 = [0;l;0];
b5 = [0;0;l/4];

x_plot = [posicao(1)];
y_plot = [posicao(2)];
z_plot = [posicao(3)];
roll_plot = orientacao(1);
pitch_plot = orientacao(2);
yaw_plot = orientacao(3);

x_des_plot = [];
y_des_plot = [];
z_des_plot = [];
roll_des_plot = [];
pitch_des_plot = [];
yaw_des_plot = [];

R_plot= [];
%

PodeComecar = 0;



%


% PARA CONTROLADOR

iteracao=1;

rdes = [];
rdv = [];
rda = [];
rdj = [];
rds = [];

pqrc = [0;0;0];  % p q r (controlado)

rc = zeros(6,1);   % x y z roll pitch yaw (controlados)
rc_anterior = zeros(6,1);
pqrc_anterior = zeros(3,1);

% T_medido = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
%     roll, 1, sin(roll);...
%     sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
% 
% 



% PARA TRAJETORIA / gambiarra da trajetoria TODO achar melhor metodo

% if(isfield(quad,'ApagarWaypoints'))
%     if(ApagarWaypoints)
%         caminho.X = posicao(1);
%         caminho.Y = posicao(2);
%         caminho.Z = posicao(3);
%         caminho.YAW = orientacao(3);
%         caminho.T = 0;
%     end
% else
%     caminho.X = posicao(1);
%     caminho.Y = posicao(2);
%     caminho.Z = posicao(3);
%     caminho.YAW = orientacao(3);
%     caminho.T = 0;
% end

% caminho.X=[0 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.15 3 3.5 4 4.5 5];
% caminho.Y=[0 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0];
% caminho.Z=[0 1.5 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1];
% caminho.YAW=[0 0   0.7854    1.5708    2.3562    3.1416    3.9270    4.7124    5.4978    6.2832    7.0686    7.8540    8.6394    9.4248   10.2102   10.9956   11.7810   12.5664   13.3518   14.1372   14.9226   15.7080];
% caminho.T=[0 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22];

rdes = [];
rdv  = [];
rda = [];
rdj = [];
rds = [];



% erro_qd_medio = [];


xaxis.min = -5;
xaxis.max = 5;
yaxis.min = -5;
yaxis.max = 5;
zaxis.min = 0;
zaxis.max = 3;


end