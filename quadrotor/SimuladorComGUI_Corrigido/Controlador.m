function Controlador

    global quad;
    
    if(quad.iteracao > length(quad.rdes(1,:)))
        Controlador_Position_Hold();
    end
    
    quad.rc = quad.rda(1:3,quad.iteracao);
    quad.rc = quad.rc + quad.Kcd.*(quad.rdv(1:3,quad.iteracao) - [quad.estados_medidos(7:9)]) + quad.Kcp.* (quad.rdes(1:3,quad.iteracao) - quad.estados_medidos(1:3));
    
    quad.rc(4) = 1/quad.g*(quad.rc(1) * sin(quad.rdes(6,quad.iteracao)) - quad.rc(2) *cos(quad.rdes(6,quad.iteracao)));
    quad.rc(5) = 1/quad.g*(quad.rc(1) * cos(quad.rdes(6,quad.iteracao)) + quad.rc(2)*sin(quad.rdes(6,quad.iteracao)));
    quad.rc(6) = quad.rdes(6,quad.iteracao);
    
    %%%%%%%%%%apenas para criar o plot no final
%     rc2 = rda(1:3,iteracao);
%     rc2 = rc2 + Kcd.*(rdv(1:3,iteracao) - [estados(7:9)]) + Kcp.* (rdes(1:3,iteracao) - estados(1:3));
%     
%     rc2(4) = 1/g*(rc2(1) * sin(rdes(6,iteracao)) - rc2(2) *cos(rdes(6,iteracao)));
%     rc2(5) = 1/g*(rc2(1) * cos(rdes(6,iteracao)) + rc2(2)*sin(rdes(6,iteracao)));
%     rc2(6) = rdes(6,iteracao);
    
%     quad.rdes(4:6,quad.iteracao) = quad.rc2(4:6);
    quad.rdes(4:6,quad.iteracao) = quad.rc(4:6);
    
%     erro_qd_medio = erro_qd_medio + (rdes(1:6,iteracao)-estados(1:6)).^2;
%     if(~isfinite(erro_qd_medio))
%         erro = 50000;
%         return
%     end
    %%%%%%%%%%
    
    quad.T_medido = [cos(quad.estados_medidos(5)), 0, -cos(quad.estados_medidos(4))*sin(quad.estados_medidos(5)); ...
        0, 1, sin(quad.estados_medidos(4));...
        sin(quad.estados_medidos(5)), 0, cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5))];  
    
    quad.pqrc = quad.T_medido* [(quad.rc(4)-quad.rc_anterior(4))/quad.dt;(quad.rc(5)-quad.rc_anterior(5))/quad.dt;(quad.rc(6)-quad.rc_anterior(6))/quad.dt];


%     T = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
%         roll, 1, sin(roll);...
%         sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
    
%     pqrc = rdv(4:6,iteracao);
%     prqc(1) = (rc(4)-rc_anterior(4))/dt;
%     prqc(2) = (rc(5)-rc_anterior(5))/dt;
    
    
    
    u1 = quad.m*(quad.g+quad.rc(3));%/(cos(quad.estados_medidos(4))*cos(quad.estados_medidos(5)));
    u2 = [quad.Krollp * (quad.rc(4) - quad.estados_medidos(4)) + quad.Krolld * (quad.pqrc(1) - quad.estados_medidos(10));...
        quad.Kpitchp * (quad.rc(5) - quad.estados_medidos(5)) + quad.Kpitchd * (quad.pqrc(2) - quad.estados_medidos(11));...
        quad.Kyawp * (quad.rc(6) - quad.estados_medidos(6)) + quad.Kyawd * (quad.pqrc(3) - quad.estados_medidos(12));];
    
    
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