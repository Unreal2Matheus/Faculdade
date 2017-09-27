% sem a realimentacao 
syms s K Kh ;
den = K ;
num = s^2 + s + K;
UP = 0.2;
zeta = (-log(UP))/((pi^2+(log((UP))^2))^(1/2));
wn = 1/(2*zeta);
K = wn^2;
wd = wn*sqrt((1-zeta^2));
tp = pi/wd ; 
