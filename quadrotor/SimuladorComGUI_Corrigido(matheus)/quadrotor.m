classdef quadrotor
    properties
        
        posicao;      %x, y, z
        orientacao ; %roll, pitch, yaw
        velocidade_linear = [0;0;0]; %vx, vy, vz
        velocidade_angular_frame_inercial = [0;0;0]; %wx, wy, wz
        velocidade_angular_frame_quad= [0;0;0]; %p, q, r
        aceleracao_linear = [0;0;0]; %ax, ay, az
        aceleracao_angular = [0;0;0]; %wxdot, wydot, wzdot
        CSI = [];
        m = 0.468; % massa generica do quadrotor TODO
        g = 9.81;
        dt = 0.02;
        %momento de inercia
        Ixx = 4.856*10^-3;
        Iyy = 4.856*10^-3;
        Izz = 8.801*10^-3;
        I = [4.856*10^-3 0 0;0 4.856*10^-3 0; 0 0 8.801*10^-3];
        %tamanho do braço
        l = 0.225;
        %parametros
        b = 1.140*10^-7;
        k = 2.980*10^-6;
        caminho = struct('X',0 , 'Y', 0, 'Z',0 , 'YAW',0, 'T',0);
        estados;
        estados_medidos;
        motor = [0;0;0;0];
        
        
        K = [6.1405    9.1331    6.1405    9.1331    6.1405    9.1331    1.5782    1.5782    1.5782    0.8000    0.8000    0.8000];
        %         Krollp = K(1);
        %         Krolld = K(2);
        %         Kpitchp = K(3);
        %         Kpitchd = K(4);
        %         Kyawp = K(5);
        %         Kyawd = K(6);
        %         Kcp = K(7:9)';
        %         Kcd = K(10:12)';
        
        Kr = 9656.655720288503;
        Kw = 1832.403541405957;
        
        
        % Para plots
        %         b1 = [-l;0;0];
        %         b2 = [0;-l;0];
        %         b3 = [l;0;0];
        %         b4 = [0;l;0];
        %         b5 = [0;0;l/4];
        
        %         x_plot = [posicao(1)];
        %         y_plot = [posicao(2)];
        %         z_plot = [posicao(3)];
        %         roll_plot = orientacao(1);
        %         pitch_plot = orientacao(2);
        %         yaw_plot = orientacao(3);
        
        x_des_plot = [];
        y_des_plot = [];
        z_des_plot = [];
        roll_des_plot = [];
        pitch_des_plot = [];
        yaw_des_plot = [];
        
        R_plot= [];
        %
        
        PodeComecar = 0;
        
        
        
        %
        
        
        % PARA CONTROLADOR
        
        iteracao=1;
        
        rdes = [];
        rdv = [];
        rda = [];
        rdj = [];
        rds = [];
        
        pqrc = [0;0;0];  % p q r (controlado)
        
        rc = zeros(6,1);   % x y z roll pitch yaw (controlados)
        rc_anterior = zeros(6,1);
        pqrc_anterior = zeros(3,1);
        
        %         T_medido = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
        %             roll, 1, sin(roll);...
        %             sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
        
        
        
        
        
        
%         caminho.X=[0 -5 -4.5 -4 -3.5 -3 -2.5 -2 -1.5 -1 -0.5 0 0.5 1 1.5 2 2.15 3 3.5 4 4.5 5];
%         caminho.Y=[0 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0 0.35 0.5 0.35 0 -0.35 -0.5 -0.35 0];
%         caminho.Z=[0 1.5 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1 1.15 1.5 1.85 2 1.85 1.5 1.15 1];
%         caminho.YAW=[0 0   0.7854    1.5708    2.3562    3.1416    3.9270    4.7124    5.4978    6.2832    7.0686    7.8540    8.6394    9.4248   10.2102   10.9956   11.7810   12.5664   13.3518   14.1372   14.9226   15.7080];
%         caminho.T=[0 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22];
        
        
        
        % erro_qd_medio = [];
        
        
        %         xaxis.min = -5;
        %         xaxis.max = 5;
        %         yaxis.min = -5;
        %         yaxis.max = 5;
        %         zaxis.min = 0;
        %         zaxis.max = 3;
        %
        
    end
    methods
        function obj= CalcularTrajetoria(obj)
            
            % disp('s');
            if(length(obj.caminho.T) ==  1)
                errordlg('ERRO - NÃO EXISTEM WAYPOINTS MARCADOS');
                return
            end
            
%            Tmax = obj.caminho.T(end);
%             obj.plot.t = 0:obj.dt:obj.Tmax;
            
            
            % [rdes,rdv,rda,rdj,rds]=plan_traj(estados(1:6),[10;10;10;0;0;pi/2],dt,Tmax);
            
            [obj.rdes(:,1),obj.rdv(:,1),obj.rda(:,1),obj.rdj(:,1),obj.rds(:,1)]=planejamento_de_trajetoria_n_pontos(obj.caminho.X,obj.dt,obj.caminho.T);
            [obj.rdes(:,2),obj.rdv(:,2),obj.rda(:,2),obj.rdj(:,2),obj.rds(:,2)]=planejamento_de_trajetoria_n_pontos(obj.caminho.Y,obj.dt,obj.caminho.T);
            [obj.rdes(:,3),obj.rdv(:,3),obj.rda(:,3),obj.rdj(:,3),obj.rds(:,3)]=planejamento_de_trajetoria_n_pontos(obj.caminho.Z,obj.dt,obj.caminho.T);
            [obj.rdes(:,6),obj.rdv(:,6),obj.rda(:,6),obj.rdj(:,6),obj.rds(:,6)]=planejamento_de_trajetoria_n_pontos(obj.caminho.YAW,obj.dt,obj.caminho.T);
            obj.rdes = obj.rdes';
            obj.rdv = obj.rdv';
            obj.rda = obj.rda';
            obj.rdj = obj.rdj';
            obj.rds = obj.rds';
            
            obj.PodeComecar = 1;
            
            
%             set(obj.BotaoCalcTraj,'Visible','Off');
%             set(obj.ListaControlador,'Visible','Off');
        end
        function  obj = desenhar_motor(r,R,posicao,frente,pc)
            if(nargin < 4)
                frente = 0;
                pc = 0;
            end
            if(nargin < 5)
                pc = 0;
            end
            obj.posicao = [x;y;z;];
            
            th = 0:pi/50:2*pi;
            
            % xunit = r * cos(th) + x;
            %
            % yunit = r * sin(th) + y;
            
            xunit = r * cos(th);
            
            yunit = r * sin(th);
            
            asd = [xunit;yunit;zeros(size(xunit))];
            
            for(i=1:length(xunit))
                asd(:,i) = R*asd(:,i) + posicao;
            end
            
            % h = plot3(xunit, yunit,z*ones(size(xunit)));
            if(~frente)
                if(~pc)
                    plot3(quad.Figura_principal,asd(1,:),asd(2,:),asd(3,:),'k')
                else
                    plot3(asd(1,:),asd(2,:),asd(3,:),'k')
                end
            else
                if(~pc)
                    % fill3(quad.Figura_principal,asd(1,:),asd(2,:),asd(3,:),'k')
                    plot3(quad.Figura_principal,asd(1,:),asd(2,:),asd(3,:),'r')
                else
                    fill3(asd(1,:),asd(2,:),asd(3,:),'k')
                end
                
                
            end
            
        end
        function obj = quadrotor(x,y,z,roll,pitch,yaw)
            if nargin == 0
                obj.posicao = [0;0;0];
                obj.orientacao = [0;0;0];
                obj.estados = [obj.posicao;obj.orientacao;obj.velocidade_linear;obj.velocidade_angular_frame_quad];
                obj.estados_medidos = obj.estados;
                
            else
                obj.posicao = [x;y;z];
                obj.orientacao = [roll;pitch;yaw];
                obj.estados = [obj.posicao;obj.orientacao;obj.velocidade_linear;obj.velocidade_angular_frame_quad];
                obj.estados_medidos = obj.estados;
            end
            
        end
        function obj=Modelo(obj)
            
            R = obj.Matriz_rotacao(obj.orientacao(1),obj.orientacao(2),obj.orientacao(3));
            
            
            obj.aceleracao_linear = -obj.g * [0;0;1] + obj.k*(obj.motor(1)^2+obj.motor(2)^2+obj.motor(3)^2+obj.motor(4)^2)/obj.m * R(:,3);
            
            
            obj.velocidade_linear = obj.velocidade_linear + obj.aceleracao_linear*obj.dt;
            obj.posicao = obj.posicao + obj.velocidade_linear*obj.dt;
            
            p = obj.velocidade_angular_frame_quad(1);
            q = obj.velocidade_angular_frame_quad(2);
            r = obj.velocidade_angular_frame_quad(3);
            
            obj.aceleracao_angular = [(obj.Iyy-obj.Izz) * q*r/obj.Ixx;(obj.Izz-obj.Ixx) * p*r/obj.Iyy;(obj.Ixx-obj.Iyy) * p*q/obj.Izz] + [obj.l*obj.k*(obj.motor(2)^2-obj.motor(4)^2)/obj.Ixx;obj.l*obj.k*(-obj.motor(1)^2+obj.motor(3)^2)/obj.Iyy;obj.b*(obj.motor(1)^2-obj.motor(2)^2+obj.motor(3)^2-obj.motor(4)^2)/obj.Izz];
            
            obj.velocidade_angular_frame_quad = obj.velocidade_angular_frame_quad + obj.aceleracao_angular*obj.dt;
            
            T = obj.Matriz_transformacao(obj.orientacao(1),obj.orientacao(2));
            
            obj.velocidade_angular_frame_inercial = T^-1 * obj.velocidade_angular_frame_quad;
            obj.orientacao = obj.orientacao + obj.velocidade_angular_frame_inercial*obj.dt;
            
            if(obj.orientacao(1) > pi) obj.orientacao(1) = mod(obj.orientacao(1),-pi);end
            if(obj.orientacao(1) < -pi) obj.orientacao(1) = mod(obj.orientacao(1),pi);end
            
            if(obj.orientacao(2) > pi) obj.orientacao(2) = mod(obj.orientacao(2),-pi);end
            if(obj.orientacao(2) < -pi) obj.orientacao(2) = mod(obj.orientacao(2),pi);end
            
            obj.estados = [obj.posicao;obj.orientacao;obj.velocidade_linear;obj.velocidade_angular_frame_quad];
        end
        function obj = Controlador_Position_Hold(obj)
            
            
            obj.rdes(:,end+1) = obj.rdes(:,end);
            obj.rdv(:,end+1) = 0;
            obj.rda(:,end+1) = 0;
            obj.rdj(:,end+1) = 0;
            obj.rds(:,end+1) = 0;
        end 
            function obj = Controlador(obj)
                
                
                if(obj.iteracao > length(obj.rdes(1,:)))
                    obj = obj.Controlador_Position_Hold();
                end
                obj.rc = obj.rda(1:3,obj.iteracao);
                obj.rc = obj.rc + obj.K(10:12)'.*(obj.rdv(1:3,obj.iteracao) - [obj.estados_medidos(7:9)]) + obj.Kcp.* (obj.rdes(1:3,obj.iteracao) - obj.estados_medidos(1:3));
                
                obj.rc(4) = 1/obj.g*(obj.rc(1) * sin(obj.rdes(6,obj.iteracao)) - obj.rc(2) *cos(obj.rdes(6,obj.iteracao)));
                obj.rc(5) = 1/obj.g*(obj.rc(1) * cos(obj.rdes(6,obj.iteracao)) + obj.rc(2)*sin(obj.rdes(6,obj.iteracao)));
                obj.rc(6) = obj.rdes(6,obj.iteracao);
                
                %%%%%%%%%%apenas para criar o plot no final
                %     rc2 = rda(1:3,iteracao);
                %     rc2 = rc2 + Kcd.*(rdv(1:3,iteracao) - [estados(7:9)]) + Kcp.* (rdes(1:3,iteracao) - estados(1:3));
                %
                %     rc2(4) = 1/g*(rc2(1) * sin(rdes(6,iteracao)) - rc2(2) *cos(rdes(6,iteracao)));
                %     rc2(5) = 1/g*(rc2(1) * cos(rdes(6,iteracao)) + rc2(2)*sin(rdes(6,iteracao)));
                %     rc2(6) = rdes(6,iteracao);
                
                %     obj.rdes(4:6,obj.iteracao) = obj.rc2(4:6);
                obj.rdes(4:6,obj.iteracao) = obj.rc(4:6);
                
                %     erro_qd_medio = erro_qd_medio + (rdes(1:6,iteracao)-estados(1:6)).^2;
                %     if(~isfinite(erro_qd_medio))
                %         erro = 50000;
                %         return
                %     end
                %%%%%%%%%%
                
                T = obj.Matriz_transformacao(obj.orientacao(1),obj.orientacao(2));
                
                obj.pqrc = T* [(obj.rc(4)-obj.rc_anterior(4))/obj.dt;(obj.rc(5)-obj.rc_anterior(5))/obj.dt;(obj.rc(6)-obj.rc_anterior(6))/obj.dt];
                
                
                %     T = [cos(pitch), roll, -cos(roll)*sin(pitch); ...
                %         roll, 1, sin(roll);...
                %         sin(pitch), roll, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
                
                %     pqrc = rdv(4:6,iteracao);
                %     prqc(1) = (rc(4)-rc_anterior(4))/dt;
                %     prqc(2) = (rc(5)-rc_anterior(5))/dt;
                
                
                
                u1 = obj.m*(obj.g+obj.rc(3));%/(cos(obj.estados_medidos(4))*cos(obj.estados_medidos(5)));
                u2 = [obj.Krollp * (obj.rc(4) - obj.estados_medidos(4)) + obj.Krolld * (obj.pqrc(1) - obj.estados_medidos(10));...
                    obj.Kpitchp * (obj.rc(5) - obj.estados_medidos(5)) + obj.Kpitchd * (obj.pqrc(2) - obj.estados_medidos(11));...
                    obj.Kyawp * (obj.rc(6) - obj.estados_medidos(6)) + obj.Kyawd * (obj.pqrc(3) - obj.estados_medidos(12));];
                
                
                obj.rc_anterior = obj.rc;
                
                
                obj.motor(1) = sqrt(u1/(4*obj.k) - obj.Iyy*u2(2)/(2*obj.k*obj.l) + obj.Izz*u2(3)/(4*obj.b));
                obj.motor(2) = sqrt(u1/(4*obj.k) + obj.Ixx*u2(1)/(2*obj.k*obj.l) - obj.Izz*u2(3)/(4*obj.b));
                obj.motor(3) = sqrt(u1/(4*obj.k) + obj.Iyy*u2(2)/(2*obj.k*obj.l) + obj.Izz*u2(3)/(4*obj.b));
                obj.motor(4) = sqrt(u1/(4*obj.k) - obj.Ixx*u2(1)/(2*obj.k*obj.l) - obj.Izz*u2(3)/(4*obj.b));
                
                
                
                obj.x_des_plot = [obj.x_des_plot obj.rdes(1,obj.iteracao)];
                obj.y_des_plot = [obj.y_des_plot obj.rdes(2,obj.iteracao)];
                obj.z_des_plot = [obj.z_des_plot obj.rdes(3,obj.iteracao)];
                obj.roll_des_plot = [obj.x_des_plot obj.rdes(4,obj.iteracao)];
                obj.pitch_des_plot = [obj.y_des_plot obj.rdes(5,obj.iteracao)];
                obj.yaw_des_plot = [obj.z_des_plot obj.rdes(6,obj.iteracao)];
                
                
            end
        end
        %metodos estaticos n dependem do obj da classe
        methods (Static)
            function R = Matriz_rotacao(roll,pitch,yaw)
                R = [cos(yaw)*cos(pitch)-sin(roll)*sin(yaw)*sin(pitch), -cos(roll)*sin(yaw), cos(yaw)*sin(pitch)+cos(pitch)*sin(roll)*sin(yaw);...
                    cos(pitch)*sin(yaw)+cos(yaw)*sin(roll)*sin(pitch), cos(roll)*cos(yaw), sin(yaw)*sin(pitch)-cos(pitch)*sin(roll)*cos(yaw);...
                    -cos(roll)*sin(pitch), sin(roll), cos(roll)*cos(pitch)];
            end
            function T = Matriz_transformacao(roll,pitch)
                T = [cos(pitch), 0, -cos(roll)*sin(pitch); ...
                    0, 1, sin(roll);...
                    sin(pitch), 0, cos(roll)*cos(pitch)];     %[p;q;r] = T*(euler rates)
            end
            
        end
        
end
    
