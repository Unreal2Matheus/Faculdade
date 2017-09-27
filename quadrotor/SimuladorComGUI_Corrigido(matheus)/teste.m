posicao = quad;
posicao.Value = [0;0;0];  %x, y, z
orientacao = quad;
orientacao.Value = [0;0;0]; %roll, pitch, yaw
velocidade_linear = quad;
velocidade_linear.Value = [0;0;0]; %vx, vy, vz
velocidade_angular_frame_inercial = quad;
velocidade_angular_frame_inercial.Value = [0;0;0]; %wx, wy, wz
velocidade_angular_frame_quad = quad;
velocidade_angular_frame_quad.Value = [0;0;0]; %p, q, r
aceleracao_linear = quad;
aceleracao_linear.Value = [0;0;0]; %ax, ay, az
aceleracao_angular = quad;
aceleracao_angular.Value = [0;0;0]; %wxdot, wydot, wzdot

roll =  orientacao.Value(1);
pitch = orientacao.Value(2);
yaw = orientacao.Value(3);

R = quad;


R.Value = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
    cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
    -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];

T = quad;
T.Value = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
        0, 1, sin(roll);...
        sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
CSI = quad ;   
CSI.Value = [];

m = quad;
m.Value = 0.468;
g = quad;
g.Value = 9.81;
dt = quad;
dt.Value = 0.02;
%momento de inercia
Ixx = quad;
Iyy = quad;
Izz = quad;
Ixx.Value = 4.856*10^-3;
Iyy.Value = 4.856*10^-3;
Izz.Value = 8.801*10^-3;
I = quad;
I.Value = [Ixx.Value 0 0;0 Iyy.Value 0; 0 0 Izz.Value];
%tamanho do braço
l = quad;
l.Value = 0.225;
%parametros
b = quad;
b.Value = 1.140*10^-7;
k = quad;
k.Value = 2.980*10^-6;
estados = quad;
estados.Value = [posicao.Value;orientacao.Value;velocidade_linear.Value;velocidade_angular_frame_quad.Value];
estados_medidos = quad;
estados_medidos.Value = estados.Value;
motor = quad;
motor.Value = [0;0;0;0];
K = quad;
K.Value = [6.1405    9.1331    6.1405    9.1331    6.1405    9.1331    1.5782    1.5782    1.5782    0.8000    0.8000    0.8000];
Krollp = quad;
Krollp.Value = K.Value(1);
Krolld = quad;
Krolld.Value = K.Value(2);
Kpitchp = quad;
Kpitchp.Value = K.Value(3);
Kpitchd = quad;
Kpitchd.Value = K.Value(4);
Kyawp = quad;
Kyawp.Value = K.Value(5);
Kyawd = quad;
Kyawd.Value = K.Value(7);
Kcp = quad;
Kcp.Value = K.Value(7:9)';
Kcd = quad;
Kcd.Value = K.Value(10:12)';
Kr = quad;
Kr.Value = 9656.655720288503;
Kw = quad;
Kw.Value = 1832.403541405957;


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

quad.T_medido = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
    0, 1, sin(roll);...
    sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)





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
