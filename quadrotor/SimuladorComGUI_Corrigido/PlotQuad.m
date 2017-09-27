function PlotQuad()

global quad;

try


b1 = quad.R*quad.b1 + quad.posicao;
b2 = quad.R*quad.b2 + quad.posicao;
b3 = quad.R*quad.b3 + quad.posicao;
b4 = quad.R*quad.b4 + quad.posicao;
b5 = quad.R*quad.b5 + quad.posicao;


plot3(quad.Figura_principal,quad.x_plot,quad.y_plot,quad.z_plot,'k')
hold(quad.Figura_principal,'on');
plot3(quad.Figura_principal,quad.x_des_plot,quad.y_des_plot,quad.z_des_plot,'k--')
plot3(quad.Figura_principal,[b1(1) b3(1)],[b1(2) b3(2)],[b1(3) b3(3)],'k')
plot3(quad.Figura_principal,[b2(1) b4(1)],[b2(2) b4(2)],[b2(3) b4(3)],'k')
plot3(quad.Figura_principal,[quad.posicao(1) b5(1)],[quad.posicao(2) b5(2)],[quad.posicao(3) b5(3)],'r')
desenhar_motor(quad.l/5,quad.R,b1)
desenhar_motor(quad.l/5,quad.R,b2)
desenhar_motor(quad.l/5,quad.R,b3,1)
desenhar_motor(quad.l/5,quad.R,b4)


plot3(quad.Figura_principal,quad.caminho.X,quad.caminho.Y,quad.caminho.Z,'x')

xlabel(quad.Figura_principal,'x');ylabel(quad.Figura_principal,'y');zlabel(quad.Figura_principal,'z');
% %     axis([x-1 x+1 y-1 y+1 z-1 z+1])
% axis([min(rdes(1,:)) max(rdes(1,:)) min(rdes(1,:)) max(rdes(2,:)) min(rdes(3,:)) max(rdes(3,:))])
axis(quad.Figura_principal,[quad.xaxis.min quad.xaxis.max quad.yaxis.min quad.yaxis.max quad.zaxis.min quad.zaxis.max])
grid(quad.Figura_principal,'on');
hold(quad.Figura_principal,'off');

if(quad.iteracao>1)
    plot(quad.Figura_x,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(1,1:quad.iteracao),'r--')
    hold(quad.Figura_x,'on');
    plot(quad.Figura_x,0:quad.dt:quad.dt*quad.iteracao,quad.x_plot)
    grid(quad.Figura_x,'on');
    axis(quad.Figura_x,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(1,:))-1 max(quad.rdes(1,:))+1])
    xlabel(quad.Figura_x,'Tempo');
    ylabel(quad.Figura_x,'X');
    hold(quad.Figura_x,'off');
    
    plot(quad.Figura_y,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(2,1:quad.iteracao),'r--')
    hold(quad.Figura_y,'on');
    plot(quad.Figura_y,0:quad.dt:quad.dt*quad.iteracao,quad.y_plot)
    grid(quad.Figura_y,'on');
    axis(quad.Figura_y,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(2,:))-1 max(quad.rdes(2,:))+1])
    xlabel(quad.Figura_y,'Tempo');
    ylabel(quad.Figura_y,'Y');
    hold(quad.Figura_y,'off');
    
    plot(quad.Figura_z,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(3,1:quad.iteracao),'r--')
    hold(quad.Figura_z,'on');
    plot(quad.Figura_z,0:quad.dt:quad.dt*quad.iteracao,quad.z_plot)
    grid(quad.Figura_z,'on');
    axis(quad.Figura_z,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(3,:))-1 max(quad.rdes(3,:))+1])
    xlabel(quad.Figura_z,'Tempo');
    ylabel(quad.Figura_z,'Z');
    hold(quad.Figura_z,'off');
    
    plot(quad.Figura_roll,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(4,1:quad.iteracao),'r--')
    hold(quad.Figura_roll,'on');
    plot(quad.Figura_roll,0:quad.dt:quad.dt*quad.iteracao,quad.roll_plot)
    grid(quad.Figura_roll,'on');
    axis(quad.Figura_roll,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(4,:))-0.1 max(quad.rdes(4,:))+0.1])
    xlabel(quad.Figura_roll,'Tempo');
    ylabel(quad.Figura_roll,'Roll');
    hold(quad.Figura_roll,'off');
    
    plot(quad.Figura_pitch,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(5,1:quad.iteracao),'r--')
    hold(quad.Figura_pitch,'on');
    plot(quad.Figura_pitch,0:quad.dt:quad.dt*quad.iteracao,quad.pitch_plot)
    grid(quad.Figura_pitch,'on');
    axis(quad.Figura_pitch,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(5,:))-0.1 max(quad.rdes(5,:))+0.1])
    xlabel(quad.Figura_pitch,'Tempo');
    ylabel(quad.Figura_pitch,'Pitch');
    hold(quad.Figura_pitch,'off');
    
    plot(quad.Figura_yaw,0:quad.dt:quad.dt*(quad.iteracao-1),quad.rdes(6,1:quad.iteracao),'r--')
    hold(quad.Figura_yaw,'on');
    plot(quad.Figura_yaw,0:quad.dt:quad.dt*quad.iteracao,quad.yaw_plot)
    grid(quad.Figura_yaw,'on');
    axis(quad.Figura_yaw,[0 quad.dt*(quad.iteracao-1) min(quad.rdes(6,:))-0.1 max(quad.rdes(6,:))+0.1])
    xlabel(quad.Figura_yaw,'Tempo');
    ylabel(quad.Figura_yaw,'Yaw');
    hold(quad.Figura_yaw,'off');
else
    plot(quad.Figura_x,[0],[0])
    axis(quad.Figura_x,[0 1 -1 1])
    plot(quad.Figura_y,[0],[0])
    axis(quad.Figura_y,[0 1 -1 1])
    plot(quad.Figura_z,[0],[0])
    axis(quad.Figura_z,[0 1 -1 1])
    plot(quad.Figura_roll,[0],[0])
    axis(quad.Figura_roll,[0 1 -1 1])
    plot(quad.Figura_pitch,[0],[0])
    axis(quad.Figura_pitch,[0 1 -1 1])
    plot(quad.Figura_yaw,[0],[0])
    axis(quad.Figura_yaw,[0 1 -1 1])
    drawnow
end
catch
    quad.PodeComecar = 2;
%     return
end

end