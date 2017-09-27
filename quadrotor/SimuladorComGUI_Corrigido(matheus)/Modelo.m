function Modelo()
    global quad;
    
    roll = orientacao(1);
    pitch = orientacao(2);
    yaw = orientacao(3);
    
    
    R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
        cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
        -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
        
    aceleracao_linear = -g * [0;0;1] + k*(motor(1)^2+motor(2)^2+motor(3)^2+motor(4)^2)/m * R(:,3);
    
    
    velocidade_linear = velocidade_linear + aceleracao_linear*dt;
    posicao = posicao + velocidade_linear*dt;
    
    p = velocidade_angular_frame_quad(1);
    q = velocidade_angular_frame_quad(2);
    r = velocidade_angular_frame_quad(3);
    
    aceleracao_angular = [(Iyy-Izz) * q*r/Ixx;(Izz-Ixx) * p*r/Iyy;(Ixx-Iyy) * p*q/Izz] + [l*k*(motor(2)^2-motor(4)^2)/Ixx;l*k*(-motor(1)^2+motor(3)^2)/Iyy;b*(motor(1)^2-motor(2)^2+motor(3)^2-motor(4)^2)/Izz];
    
    velocidade_angular_frame_quad = velocidade_angular_frame_quad + aceleracao_angular*dt;
    
    
    T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
        0, 1, sin(roll);...
        sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
    velocidade_angular_frame_inercial = T^-1 * velocidade_angular_frame_quad;
    orientacao = orientacao + velocidade_angular_frame_inercial*dt;
    
    if(orientacao(1) > pi) orientacao(1) = mod(orientacao(1),-pi);end
    if(orientacao(1) < -pi) orientacao(1) = mod(orientacao(1),pi);end
    
    if(orientacao(2) > pi) orientacao(2) = mod(orientacao(2),-pi);end
    if(orientacao(2) < -pi) orientacao(2) = mod(orientacao(2),pi);end
    
    estados = [posicao;orientacao;velocidade_linear;velocidade_angular_frame_quad];
%     erro_qd_medio = erro_qd_medio + [(rdes(1:3,a)-estados(1:3)).^2;0;0;0] + [0;0;0;(cos(rdes(4:6,a))-cos(estados(4:6))).^2];
    
    
    x_plot = [x_plot posicao(1)];
    y_plot = [y_plot posicao(2)];
    z_plot = [z_plot posicao(3)];
    roll_plot = [roll_plot orientacao(1)];
    pitch_plot = [pitch_plot orientacao(2)];
    yaw_plot = [yaw_plot orientacao(3)];
    R_plot(:,:,iteracao) = R;
    
    
    
end