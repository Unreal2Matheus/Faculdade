function Modelo()
    global quad;
    
    roll = quad.orientacao(1);
    pitch = quad.orientacao(2);
    yaw = quad.orientacao(3);
    
    
    quad.R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
        cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
        -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
        
    quad.aceleracao_linear = -quad.g * [0;0;1] + quad.k*(quad.motor(1)^2+quad.motor(2)^2+quad.motor(3)^2+quad.motor(4)^2)/quad.m * quad.R(:,3);
    
    
    quad.velocidade_linear = quad.velocidade_linear + quad.aceleracao_linear*quad.dt;
    quad.posicao = quad.posicao + quad.velocidade_linear*quad.dt;
    
    p = quad.velocidade_angular_frame_quad(1);
    q = quad.velocidade_angular_frame_quad(2);
    r = quad.velocidade_angular_frame_quad(3);
    
    quad.aceleracao_angular = [(quad.Iyy-quad.Izz) * q*r/quad.Ixx;(quad.Izz-quad.Ixx) * p*r/quad.Iyy;(quad.Ixx-quad.Iyy) * p*q/quad.Izz] + [quad.l*quad.k*(quad.motor(2)^2-quad.motor(4)^2)/quad.Ixx;quad.l*quad.k*(-quad.motor(1)^2+quad.motor(3)^2)/quad.Iyy;quad.b*(quad.motor(1)^2-quad.motor(2)^2+quad.motor(3)^2-quad.motor(4)^2)/quad.Izz];
    
    quad.velocidade_angular_frame_quad = quad.velocidade_angular_frame_quad + quad.aceleracao_angular*quad.dt;
    
    
    quad.T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
        0, 1, sin(roll);...
        sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
    quad.velocidade_angular_frame_inercial = quad.T^-1 * quad.velocidade_angular_frame_quad;
    quad.orientacao = quad.orientacao + quad.velocidade_angular_frame_inercial*quad.dt;
    
    if(quad.orientacao(1) > pi) quad.orientacao(1) = mod(quad.orientacao(1),-pi);end
    if(quad.orientacao(1) < -pi) quad.orientacao(1) = mod(quad.orientacao(1),pi);end
    
    if(quad.orientacao(2) > pi) quad.orientacao(2) = mod(quad.orientacao(2),-pi);end
    if(quad.orientacao(2) < -pi) quad.orientacao(2) = mod(quad.orientacao(2),pi);end
    
    quad.estados = [quad.posicao;quad.orientacao;quad.velocidade_linear;quad.velocidade_angular_frame_quad];
%     quad.erro_qd_medio = quad.erro_qd_medio + [(rdes(1:3,a)-estados(1:3)).^2;0;0;0] + [0;0;0;(cos(rdes(4:6,a))-cos(estados(4:6))).^2];
    
    
    quad.x_plot = [quad.x_plot quad.posicao(1)];
    quad.y_plot = [quad.y_plot quad.posicao(2)];
    quad.z_plot = [quad.z_plot quad.posicao(3)];
    quad.roll_plot = [quad.roll_plot quad.orientacao(1)];
    quad.pitch_plot = [quad.pitch_plot quad.orientacao(2)];
    quad.yaw_plot = [quad.yaw_plot quad.orientacao(3)];
    quad.R_plot(:,:,quad.iteracao) = quad.R;
    
    
    
end