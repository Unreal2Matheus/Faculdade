function SalvarSimulacao(varargin)
global quad;
salvar = quad;
arquivo = inputdlg('Salvar em qual arquivo?','Salvar',1,{'.mat'});
if(~isempty(arquivo))
    arquivo = arquivo{1}; %% porque a funcao inputdlg dá resultado do tipo cell, e não string. Essa linha transforma de cell para string
    salvar.x_plot(1) = [];
    salvar.y_plot(1) = [];
    salvar.z_plot(1) = [];
    salvar.roll_plot(1) = [];
    salvar.pitch_plot(1) = [];
    salvar.yaw_plot(1) = [];
    salvar.tempo_plot = 0:salvar.dt:salvar.dt*(salvar.iteracao-1);
    salvar = rmfield(salvar,{'Figura_x','Figura_y','Figura_z','Figura_roll','Figura_pitch','Figura_yaw','rc_anterior','pqrc_anterior','pqrc','rc','rdes','rdv','rda','rdj','rds','BotaoCalcTraj','handler','ListaControlador','Janela_principal','Figura_principal','Tmax','PodeComecar','iteracao','b1','b2','b3','b4','b5','motor','estados','estados_medidos','Ixx','Iyy','Izz','R','T','posicao','orientacao','velocidade_linear','velocidade_angular_frame_inercial','velocidade_angular_frame_quad','aceleracao_linear','aceleracao_angular','plot','T_medido'});
    if(isfield(salvar,'ApagarWaypoints'))
        salvar = rmfield(salvar,'ApagarWaypoints');
    end
    save(arquivo,'salvar');
end
% disp('a');
end