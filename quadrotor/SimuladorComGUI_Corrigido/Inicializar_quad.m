function Inicializar_quad()

% global quad
% clear global
global quad

quad.posicao = [0;0;0];      %x, y, z
quad.orientacao = [0;0;0]; %roll, pitch, yaw
quad.velocidade_linear = [0;0;0]; %vx, vy, vz
quad.velocidade_angular_frame_inercial = [0;0;0]; %wx, wy, wz
quad.velocidade_angular_frame_quad= [0;0;0]; %p, q, r
quad.aceleracao_linear = [0;0;0]; %ax, ay, az
quad.aceleracao_angular = [0;0;0]; %wxdot, wydot, wzdot

roll = quad.orientacao(1);
pitch = quad.orientacao(2);
yaw = quad.orientacao(3);



quad.R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
    cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
    -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];

quad.T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
        0, 1, sin(roll);...
        sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
    
quad.CSI = [];


quad.m = 0.468;
quad.g = 9.81;
quad.dt = 0.02;
%momento de inercia
quad.Ixx = 4.856*10^-3;
quad.Iyy = quad.Ixx;
quad.Izz = 8.801*10^-3;
quad.I = [quad.Ixx 0 0;0 quad.Iyy 0; 0 0 quad.Izz];
%tamanho do braço
quad.l = 0.225;
%parametros
quad.b = 1.140*10^-7;
quad.k = 2.980*10^-6;

quad.estados = [quad.posicao;quad.orientacao;quad.velocidade_linear;quad.velocidade_angular_frame_quad];
quad.estados_medidos = quad.estados;

quad.motor = [0;0;0;0];


quad.K = [6.1405    9.1331    6.1405    9.1331    6.1405    9.1331    1.5782    1.5782    1.5782    0.8000    0.8000    0.8000];
quad.Krollp = quad.K(1);
quad.Krolld = quad.K(2);
quad.Kpitchp = quad.K(3);
quad.Kpitchd = quad.K(4);
quad.Kyawp = quad.K(5);
quad.Kyawd = quad.K(6);
quad.Kcp = quad.K(7:9)';
quad.Kcd = quad.K(10:12)';

quad.Kr = 9656.655720288503;
quad.Kw = 1832.403541405957;


% Para plots
quad.b1 = [-quad.l;0;0];
quad.b2 = [0;-quad.l;0];
quad.b3 = [quad.l;0;0];
quad.b4 = [0;quad.l;0];
quad.b5 = [0;0;quad.l/4];

quad.x_plot = [quad.posicao(1)];
quad.y_plot = [quad.posicao(2)];
quad.z_plot = [quad.posicao(3)];
quad.roll_plot = quad.orientacao(1);
quad.pitch_plot = quad.orientacao(2);
quad.yaw_plot = quad.orientacao(3);

quad.x_des_plot = [];
quad.y_des_plot = [];
quad.z_des_plot = [];
quad.roll_des_plot = [];
quad.pitch_des_plot = [];
quad.yaw_des_plot = [];

quad.R_plot= [];
%

quad.PodeComecar = 0;



%


% PARA CONTROLADOR

quad.iteracao=1;

quad.rdes = [];
quad.rdv = [];
quad.rda = [];
quad.rdj = [];
quad.rds = [];

quad.pqrc = [0;0;0];  % p q r (controlado)

quad.rc = zeros(6,1);   % x y z roll pitch yaw (controlados)
quad.rc_anterior = zeros(6,1);
quad.pqrc_anterior = zeros(3,1);

quad.T_medido = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
    roll, 1, sin(roll);...
    sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)





% PARA TRAJETORIA

if(isfield(quad,'ApagarWaypoints'))
    if(quad.ApagarWaypoints)
        quad.caminho.X = quad.posicao(1);
        quad.caminho.Y = quad.posicao(2);
        quad.caminho.Z = quad.posicao(3);
        quad.caminho.YAW = quad.orientacao(3);
        quad.caminho.T = 0;
    end
else
    quad.caminho.X = quad.posicao(1);
    quad.caminho.Y = quad.posicao(2);
    quad.caminho.Z = quad.posicao(3);
    quad.caminho.YAW = quad.orientacao(3);
    quad.caminho.T = 0;
end

% quad.caminho.X=[0 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.15 3 3.5 4 4.5 5];
% quad.caminho.Y=[0 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0];
% quad.caminho.Z=[0 1.5 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1];
% quad.caminho.YAW=[0 0   0.7854    1.5708    2.3562    3.1416    3.9270    4.7124    5.4978    6.2832    7.0686    7.8540    8.6394    9.4248   10.2102   10.9956   11.7810   12.5664   13.3518   14.1372   14.9226   15.7080];
% quad.caminho.T=[0 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22];

quad.rdes = [];
quad.rdv  = [];
quad.rda = [];
quad.rdj = [];
quad.rds = [];



% quad.erro_qd_medio = [];


quad.xaxis.min = -5;
quad.xaxis.max = 5;
quad.yaxis.min = -5;
quad.yaxis.max = 5;
quad.zaxis.min = 0;
quad.zaxis.max = 3;


end