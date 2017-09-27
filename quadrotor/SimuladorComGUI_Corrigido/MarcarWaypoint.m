function MarcarWaypoint(varargin)
    
    global quad;
    
    x=str2double(get(quad.handler.X,'String'));
    y=str2double(get(quad.handler.Y,'String'));
    z=str2double(get(quad.handler.Z,'String'));
    yaw=str2double(get(quad.handler.YAW,'String'));
    t=str2double(get(quad.handler.T,'String'));
    if(isempty(x)||isempty(y)||isempty(z)||isempty(yaw)||isempty(t))
        errordlg('ERRO - PONTOS DIGITADOS NÃO NUMÉRICOS');
        return
    end
    
    if(t > quad.caminho.T(end))
        quad.caminho.X = [quad.caminho.X x];
        quad.caminho.Y = [quad.caminho.Y y];
        quad.caminho.Z = [quad.caminho.Z z];
        quad.caminho.YAW = [quad.caminho.YAW yaw];
        quad.caminho.T = [quad.caminho.T t];
    else
        errordlg('ERRO - TEMPO DIGITADO MENOR OU IGUAL AO TEMPO ANTERIOR');
        return 
    end

end