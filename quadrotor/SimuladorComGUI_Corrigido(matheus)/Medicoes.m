function Medicoes()
    
    global quad;
%     quad.estados_medidos(1:3,1) = quad.estados(1:3) + 1/10*randn(3,1);
%     quad.estados_medidos(4:6,1) = quad.estados(4:6) + 1/50*randn(3,1);
%     quad.estados_medidos(7:9,1) = quad.estados(7:9) + 1/10*randn(3,1);
%     quad.estados_medidos(10:12,1) = quad.estados(10:12) + 1/50 *randn(3,1);

%     quad.estados_medidos(1:3,1) = quad.estados(1:3) + 1/100*randn(3,1);
%     quad.estados_medidos(4:6,1) = quad.estados(4:6) + 1/100*randn(3,1);
%     quad.estados_medidos(7:9,1) = quad.estados(7:9) + 1/100*randn(3,1);
%     quad.estados_medidos(10:12,1) = quad.estados(10:12) + 1/100 *randn(3,1);
    
    quad.estados_medidos = quad.estados;
end