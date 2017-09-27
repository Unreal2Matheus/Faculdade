function ControladorNaoLinearArtigo

    global quad;
    
    quad.Kcp = 18.1403918162131;
    quad.Kcd = 12.6051464768001;
    quad.Kr = 10;
    quad.Kw = 7;
    
    if(quad.iteracao > length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
    
    quad.rc = quad.rda(1:3,quad.iteracao);
    quad.rc = quad.rc + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - [quad.estados_medidos(7:9)]) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.estados_medidos(1:3));
    
    quad.rc(6) = quad.rdes(6,quad.iteracao);
    
    
    vetor_t = quad.m*quad.rc(1:3) + quad.m*quad.g*[0;0;1];
    
    R_medido = [cos(quad.estados_medidos(6))*cos(quad.estados_medidos(5))-sin(quad.estados_medidos(4))*sin(quad.estados_medidos(6))*sin(quad.estados_medidos(5)), -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(6)), cos(quad.estados_medidos(6))*sin(quad.estados_medidos(5))+cos(quad.estados_medidos(5))*sin(quad.estados_medidos(4))*sin(quad.estados_medidos(6));...
    cos(quad.estados_medidos(5))*sin(quad.estados_medidos(6))+cos(quad.estados_medidos(6))*sin(quad.estados_medidos(4))*sin(quad.estados_medidos(5)), cos(quad.estados_medidos(4))*cos(quad.estados_medidos(6)), sin(quad.estados_medidos(6))*sin(quad.estados_medidos(5))-cos(quad.estados_medidos(5))*sin(quad.estados_medidos(4))*cos(quad.estados_medidos(6));...
    -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)), sin(quad.estados_medidos(4)), cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];
    
    
    vetor_t_normalizado = vetor_t/norm(vetor_t);
    
    

    b1d = [cos(quad.rc(6));sin(quad.rc(6));0];
    b2d = cross(vetor_t_normalizado, b1d)/norm(cross(vetor_t_normalizado, b1d));
    
    R_des = [cross(b2d,vetor_t_normalizado), b2d, vetor_t_normalizado];
    
    
    quad.rc(4) = atan2(R_des(3,2),R_des(2,2)/cos(quad.rc(6)));
    quad.rc(5) = atan2(R_des(1,3)*cos(quad.rc(6))+sin(quad.rc(6))*R_des(2,3), R_des(3,3)/cos(quad.rc(4)));

    quad.rdes(4:6,quad.iteracao) = quad.rc(4:6);
    
    
    erro_nao_linear_skew_symmetric = 1/2 * (R_des'*R_medido - R_medido'*R_des);
    erro_nao_linear = [0;0;0];
    erro_nao_linear(1) = erro_nao_linear_skew_symmetric(3,2);
    erro_nao_linear(2) = erro_nao_linear_skew_symmetric(1,3);
    erro_nao_linear(3) = erro_nao_linear_skew_symmetric(2,1);
    
    
    TRANSFORMACAO = [cos(quad.estados_medidos(5)), 0, -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)); ...
        0, 1, sin(quad.estados_medidos(4));...
        sin(quad.estados_medidos(5)), 0, cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];  
    
    quad.pqrc = TRANSFORMACAO* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];
    
    erro_nao_linear_w = quad.estados_medidos(10:12) - R_medido' * R_des * quad.pqrc;
    
    
    erro_ac_angular = (quad.pqrc - quad.pqrc_anterior)/quad.dt;
    
    u1 = vetor_t' * (R_medido*[0;0;1]); 
    
%     u2 = cross(estados_medidos(10:12),I*estados_medidos(10:12)) + (-Kr*erro_nao_linear - Kw * erro_nao_linear_w);
    u2 = cross(quad.estados_medidos(10:12),quad.I*quad.estados_medidos(10:12)) + (-quad.Kr*erro_nao_linear - quad.Kw * erro_nao_linear_w) - quad.I * (cross(quad.estados_medidos(10:12),R_medido'*R_des*quad.pqrc) - R_medido'*R_des*erro_ac_angular);

    
    
    
    quad.rc_anterior = quad.rc;
    quad.pqrc_anterior = quad.pqrc;
    
    
    quad.motor(1) = sqrt(u1/(4*quad.k) - quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(2) = sqrt(u1/(4*quad.k) + quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    quad.motor(3) = sqrt(u1/(4*quad.k) + quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(4) = sqrt(u1/(4*quad.k) - quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    
    
    
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.roll_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.pitch_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.yaw_des_plot quad.rdes(6,quad.iteracao)];
    
    
end