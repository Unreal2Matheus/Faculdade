function CalcularTrajetoria(varargin)

global quad;

% disp('s');
if(length(quad.caminho.T) ==  1)
    errordlg('ERRO - NÃO EXISTEM WAYPOINTS MARCADOS');
    return
end
quad.Tmax = quad.caminho.T(end);
quad.plot.t = 0:quad.dt:quad.Tmax;


% [rdes,rdv,rda,rdj,rds]=plan_traj(estados(1:6),[10;10;10;0;0;pi/2],dt,Tmax);

[quad.rdes(:,1),quad.rdv(:,1),quad.rda(:,1),quad.rdj(:,1),quad.rds(:,1)]=planejamento_de_trajetoria_n_pontos(quad.caminho.X,quad.dt,quad.caminho.T);
[quad.rdes(:,2),quad.rdv(:,2),quad.rda(:,2),quad.rdj(:,2),quad.rds(:,2)]=planejamento_de_trajetoria_n_pontos(quad.caminho.Y,quad.dt,quad.caminho.T);
[quad.rdes(:,3),quad.rdv(:,3),quad.rda(:,3),quad.rdj(:,3),quad.rds(:,3)]=planejamento_de_trajetoria_n_pontos(quad.caminho.Z,quad.dt,quad.caminho.T);
[quad.rdes(:,6),quad.rdv(:,6),quad.rda(:,6),quad.rdj(:,6),quad.rds(:,6)]=planejamento_de_trajetoria_n_pontos(quad.caminho.YAW,quad.dt,quad.caminho.T);
quad.rdes = quad.rdes';
quad.rdv = quad.rdv';
quad.rda = quad.rda';
quad.rdj = quad.rdj';
quad.rds = quad.rds';

quad.PodeComecar = 1;


set(quad.BotaoCalcTraj,'Visible','Off');
set(quad.ListaControlador,'Visible','Off');
end