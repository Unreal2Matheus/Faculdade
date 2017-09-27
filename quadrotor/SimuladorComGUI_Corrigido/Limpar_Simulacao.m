function Limpar_Simulacao(varargin)
global quad;

choice = questdlg('Deseja apagar waypoints?', ...
	'Limpar Simulação', ...
	'Sim','Não','Cancelar','Não');
% Handle response
switch choice
    case 'Sim'
        quad.ApagarWaypoints = 1;
    case 'Não'
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