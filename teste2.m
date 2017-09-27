%% questao 1
sys1 = tf([0 1],[1 1]);
sys2 = tf([0 1 0],[1 0 2]);
sys3 = tf([0 4 2],[1 2 1]);
sys4 = tf([0 1 0 2],[1 0 0 14]);
sys5 = tf([0 0 1],[1 0 0]);
sys6 = 50;
sys7 = 4;
sysfeed1 = feedback(series(sys1,sys2),sys3);
sysfeed2 = feedback(sys5,sys6,+1);
sysfeed3 = feedback(series(sysfeed1,sysfeed2),sys4);
FT = series(sys7,sysfeed3);
pzmap(FT);
figure
polos = pole(FT);
zeros = zero(FT);
plot(real(polos),imag(polos), 'x');
hold on
plot(real(zeros),imag(zeros), 'o');
grid on
%% Questao 2
 i = 1;
for k  = 0.01:0.01:10   
  
  %para T(s)=0 e R(s)=1/s
    sys1(i) = tf([0 0 k],[1 20 20]);
    sysfeed1(i) = feedback(sys1(i),1);
     %para T(s)=1/s e R(s)=0
    sys2 = tf([0 0 1],[1 20 20]);
    sysfeed2(i) = feedback(sys2,-k,+1);
    %resposta em regime permanene 
    DC1(i) = dcgain(sysfeed1(i));
    DC2(i) = dcgain(sysfeed2(i));
    if DC1(i)==DC2(i)
        display(k)
    end 
    i = i+1;
 
   
end
 k  = 0.01:0.01:10;
%plot dos regimes permanentes de acordo com k
plot(k,DC1);
figure
plot(k,DC1);
hold on 
plot(k,DC2);
%% questao 3
syms s C2 R2 C1 R1
z2 = ((1/(s*C2))^-1+(R2)^-1)^-1;
z1 = R1+(1)/(s*C1);    
FT(C2,R2,C1,R1) = -z2/z1;
FT2 = simplify(FT(0.1e-3,10e3,0.5e-3,1e3));
FT= simplify(FT);
[num,den] = numden(FT2);
[A,B,C,D] = tf2ss([0 -10 0],[1 3 2]);
t = 0:0.01:10; 
u=heaviside(t);
lsim(A,B,C,D,u,t);
legend('funcao degrau');
figure
lsim(A,B,C,D,t,t);
legend('funcao rampa');
%% questao 4