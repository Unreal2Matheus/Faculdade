function codigo_principal

% 
clear all
% clc

global quad
Inicializar_quad();

quad.Janela_principal = figure('units','normalized','position',[.05 .08 .90 .82],'name','Simulador de Quadrotor - UFJF','color','w','numbertitle','off');

quad.Figura_principal = axes('units','normalized','position',[.17 .08 .65 .8]);


quad.Figura_x = axes('units','normalized','position',[.87 .85 .115 .1]);
quad.Figura_y = axes('units','normalized','position',[.87 .70 .115 .1]);
quad.Figura_z = axes('units','normalized','position',[.87 .55 .115 .1]);
quad.Figura_roll = axes('units','normalized','position',[.87 .4 .115 .1]);
quad.Figura_pitch = axes('units','normalized','position',[.87 .25 .115 .1]);
quad.Figura_yaw = axes('units','normalized','position',[.87 .10 .115 .1]);

axis(quad.Figura_principal,[-5 5 -5 5 0 3])
axis(quad.Figura_x,[0 5 -5 5])
axis(quad.Figura_y,[0 5 -5 5])
axis(quad.Figura_z,[0 5 -5 5])
axis(quad.Figura_roll,[0 5 -90 90 ])
axis(quad.Figura_pitch,[0 5 -90 90 ])
axis(quad.Figura_yaw,[ 0 5 -180 180])


xlabel(quad.Figura_principal,'X');
ylabel(quad.Figura_principal,'Y');
zlabel(quad.Figura_principal,'Z');

xlabel(quad.Figura_x,'Tempo');
ylabel(quad.Figura_x,'X');

xlabel(quad.Figura_y,'Tempo');
ylabel(quad.Figura_y,'Y');

xlabel(quad.Figura_z,'Tempo');
ylabel(quad.Figura_z,'Z');

xlabel(quad.Figura_roll,'Tempo');
ylabel(quad.Figura_roll,'Roll');

xlabel(quad.Figura_pitch,'Tempo');
ylabel(quad.Figura_pitch,'Pitch');

xlabel(quad.Figura_yaw,'Tempo');
ylabel(quad.Figura_yaw,'Yaw');


quad.handler.X = uicontrol('units','normalized','position',[.06 .85 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.Y = uicontrol('units','normalized','position',[.06 .75 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.Z = uicontrol('units','normalized','position',[.06 .65 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.YAW = uicontrol('units','normalized','position',[.06 .55 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');
quad.handler.T = uicontrol('units','normalized','position',[.06 .45 .06 .04],'style','edit','fontsize',10,'string',0,'backgroundcolor','w');

uicontrol('units','normalized','position',[.02 .85 .03 .04],'style','text','fontsize',10,'string','X','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .75 .03 .04],'style','text','fontsize',10,'string','Y','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .65 .03 .04],'style','text','fontsize',10,'string','Z','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .55 .03 .04],'style','text','fontsize',10,'string','Yaw','backgroundcolor','w');
uicontrol('units','normalized','position',[.02 .45 .03 .04],'style','text','fontsize',10,'string','Tempo','backgroundcolor','w');



% uicontrol('units','normalized','position',[.9 .95 .03 .03],'style','text','fontsize',10,'string','X','backgroundcolor','w');

uicontrol('units','normalized','position',[.05 .95 .07 .02],'style','pushbutton','fontsize',10,'string','Salvar Simulação','callback',@SalvarSimulacao);

uicontrol('units','normalized','position',[.05 .40 .07 .02],'style','pushbutton','fontsize',10,'string','Marcar Waypoint','callback',@MarcarWaypoint);

quad.BotaoCalcTraj = uicontrol('units','normalized','position',[.025 .33 .1 .03],'style','pushbutton','fontsize',10,'string','Seguir Trajetória','callback',@CalcularTrajetoria);

uicontrol('units','normalized','position',[.025 .23 .1 .03],'style','pushbutton','fontsize',10,'string','Configurar Parâmetros','callback',@Configurar_Parametros);

uicontrol('units','normalized','position',[.025 .13 .1 .03],'style','pushbutton','fontsize',10,'string','Limpar Simulação','callback',@Limpar_Simulacao);

quad.ListaControlador = uicontrol('units','normalized','position',[0.025 .05 .1 .03],'style','popupmenu','fontsize',10,'string',{'Controle Linear';'Controle Não Linear 1';'Controle Não Linear 2'},'value',1);

% rotate3d(quad.Figura_principal)


PlotQuad();

drawnow
while(~quad.PodeComecar)
    Loop_Principal();
end

end



function Loop_Principal()
global quad;

Inicializar_quad();
while(~quad.PodeComecar)
    drawnow
    PlotQuad();
end
while(quad.PodeComecar == 1)
   tic
   Medicoes();
   %Filtro();
   try
   if(get(quad.ListaControlador,'Value')==1)
       Controlador();
   elseif(get(quad.ListaControlador,'Value')==2)
       ControladorNaoLinear();
   elseif(get(quad.ListaControlador,'Value')==3)
       ControladorNaoLinearArtigo();
   end
   catch
       return
   end
   Modelo();
   try
       if(~mod(quad.iteracao,10))
        PlotQuad();
       end
   catch
       return
   end
   drawnow
    
   while(toc<quad.dt)
   end
   
   
   quad.iteracao=quad.iteracao+1;
end
if(quad.PodeComecar == 3)
    quad.PodeComecar = 0;
end
end
