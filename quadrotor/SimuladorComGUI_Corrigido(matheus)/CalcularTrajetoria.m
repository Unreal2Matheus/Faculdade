function CalcularTrajetoria()

% disp('s');
if(length(obj.caminho.T) ==  1)
    errordlg('ERRO - NÃO EXISTEM WAYPOINTS MARCADOS');
    return
end
obj.Tmax = obj.caminho.T(end);
obj.plot.t = 0:obj.dt:obj.Tmax;


% [rdes,rdv,rda,rdj,rds]=plan_traj(estados(1:6),[10;10;10;0;0;pi/2],dt,Tmax);

[obj.rdes(:,1),obj.rdv(:,1),obj.rda(:,1),obj.rdj(:,1),obj.rds(:,1)]=planejamento_de_trajetoria_n_pontos(obj.caminho.X,obj.dt,obj.caminho.T);
[obj.rdes(:,2),obj.rdv(:,2),obj.rda(:,2),obj.rdj(:,2),obj.rds(:,2)]=planejamento_de_trajetoria_n_pontos(obj.caminho.Y,obj.dt,obj.caminho.T);
[obj.rdes(:,3),obj.rdv(:,3),obj.rda(:,3),obj.rdj(:,3),obj.rds(:,3)]=planejamento_de_trajetoria_n_pontos(obj.caminho.Z,obj.dt,obj.caminho.T);
[obj.rdes(:,6),obj.rdv(:,6),obj.rda(:,6),obj.rdj(:,6),obj.rds(:,6)]=planejamento_de_trajetoria_n_pontos(obj.caminho.YAW,obj.dt,obj.caminho.T);
obj.rdes = obj.rdes';
obj.rdv = obj.rdv';
obj.rda = obj.rda';
obj.rdj = obj.rdj';
obj.rds = obj.rds';

obj.PodeComecar = 1;


set(obj.BotaoCalcTraj,'Visible','Off');
set(obj.ListaControlador,'Visible','Off');
end