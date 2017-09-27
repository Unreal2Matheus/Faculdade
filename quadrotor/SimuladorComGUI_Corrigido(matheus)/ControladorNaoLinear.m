function ControladorNaoLinear

    global quad;
    
    quad.Kcp = quad.K(7:9)';
    quad.Kcd = quad.K(10:12)';

    quad.Kr = 9656.655720288503;
    quad.Kw = 1832.403541405957;
    
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
    
    
    quad.rc(4) = atan((vetor_t_normalizado(1) * sin(quad.rc(6)) - vetor_t_normalizado(2) * cos(quad.rc(6)))/vetor_t_normalizado(3)); %      <- quase certo
    

    quad.rc(5) = atan2(vetor_t_normalizado(1) * cos(quad.rc(6)) + vetor_t_normalizado(2) * sin(quad.rc(6)), vetor_t_normalizado(3)/cos(quad.rc(4)));
    
    R_des = [cos(quad.rc(6))*cos(quad.rc(5))-sin(quad.rc(4))*sin(quad.rc(6))*sin(quad.rc(5)), -cos(quad.rc(4))*sin(quad.rc(6)), cos(quad.rc(6))*sin(quad.rc(5))+cos(quad.rc(5))*sin(quad.rc(4))*sin(quad.rc(6));...
    cos(quad.rc(5))*sin(quad.rc(6))+cos(quad.rc(6))*sin(quad.rc(4))*sin(quad.rc(5)), cos(quad.rc(4))*cos(quad.rc(6)), sin(quad.rc(6))*sin(quad.rc(5))-cos(quad.rc(5))*sin(quad.rc(4))*cos(quad.rc(6));...
    -cos(quad.rc(4))*sin(quad.rc(5)), sin(quad.rc(4)), cos(quad.rc(4))*cos(quad.rc(5))];
    

    quad.rdes(4:6,quad.iteracao) = quad.rc(4:6);
    
    

     delta_R = R_des'*R_medido;
    
    theta = acos(( delta_R(1,1) + delta_R(2,2) + delta_R(3,3) - 1)/2);
    vetorK(1,1) = (delta_R(3,2) - delta_R(2,3))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
    vetorK(2,1) = (delta_R(1,3) - delta_R(3,1))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
    vetorK(3,1) = (delta_R(2,1) - delta_R(1,2))/sqrt((delta_R(3,2) - delta_R(2,3))^2+(delta_R(1,3) - delta_R(3,1))^2+(delta_R(2,1) - delta_R(1,2))^2);
    if(isnan(vetorK))
        vetorK = [1;0;0];
    end
%     waitforbuttonpress
%      produto_vetorial = cross(R_medido*[0;0;1],R_des*[0;0;1])/(norm(R_des*[0;0;1])*norm(R_medido*[0;0;1])); %para só sobrar o sin(theta) multiplicando
    
%     theta2 = asin(norm(produto_vetorial))
    
    %erro_nao_linear = produto_vetorial/norm(produto_vetorial) * theta;
    
    erro_nao_linear = vetorK * theta;
    
    
    
    TRANSFORMACAO = [cos(quad.estados_medidos(5)), 0, -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)); ...
        0, 1, sin(quad.estados_medidos(4));...
        sin(quad.estados_medidos(5)), 0, cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];  
    
    pqrc = TRANSFORMACAO* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];
    
    
    
    u1 = vetor_t' * (R_medido*[0;0;1]); 
    
    u2 = cross(quad.estados_medidos(10:12),quad.I*quad.estados_medidos(10:12)) + quad.I*(-quad.Kr*erro_nao_linear + quad.Kw*(pqrc - quad.estados_medidos(10:12)));
    
    
    
    
    
    quad.rc_anterior = quad.rc;
    
    
    quad.motor(1) = sqrt(u1/(4*quad.k) - quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(2) = sqrt(u1/(4*quad.k) + quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    quad.motor(3) = sqrt(u1/(4*quad.k) + quad.Iyy*u2(2)/(2*quad.k*quad.l) + quad.Izz*u2(3)/(4*quad.b));
    quad.motor(4) = sqrt(u1/(4*quad.k) - quad.Ixx*u2(1)/(2*quad.k*quad.l) - quad.Izz*u2(3)/(4*quad.b));
    
    
    
    quad.x_des_plot = [quad.x_des_plot quad.rdes(1,quad.iteracao)];
    quad.y_des_plot = [quad.y_des_plot quad.rdes(2,quad.iteracao)];
    quad.z_des_plot = [quad.z_des_plot quad.rdes(3,quad.iteracao)];
    quad.roll_des_plot = [quad.x_des_plot quad.rdes(4,quad.iteracao)];
    quad.pitch_des_plot = [quad.y_des_plot quad.rdes(5,quad.iteracao)];
    quad.yaw_des_plot = [quad.z_des_plot quad.rdes(6,quad.iteracao)];
    
    
end