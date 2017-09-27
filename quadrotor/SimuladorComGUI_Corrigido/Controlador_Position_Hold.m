function Controlador_Position_Hold()

global quad;

quad.rdes(:,end+1) = quad.rdes(:,end);
quad.rdv(:,end+1) = 0;
quad.rda(:,end+1) = 0;
quad.rdj(:,end+1) = 0;
quad.rds(:,end+1) = 0;



end