function Limpar_Simulacao(varargin)
global quad;

choice = questdlg('Deseja apagar waypoints?', ...
	'Limpar Simula��o', ...
	'Sim','N�o','Cancelar','N�o');
% Handle response
switch choice
    case 'Sim'
        quad.ApagarWaypoints = 1;
    case 'N�o'
        quad.ApagarWaypoints = 0;
    case 'Cancelar'
        return
    case ''
        return
end

quad.PodeComecar =3;
set(quad.BotaoCalcTraj,'Visible','on');
set(quad.ListaControlador,'Visible','on');
end