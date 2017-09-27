function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters
erro = z-z_des;
erro_ponto = diff(z)-diff(z_des);
Kp = 10;
Kv = 100;
z_dois_pontos = diff(z_des,2);
u = ans.mass(z_dois_pontos+Kp*erro+Kv*erro_ponto+ans.gravity);


% FILL IN YOUR CODE HERE


end

