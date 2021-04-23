%% Aquisi��o e identifica��o multivar:
% Tutorial para a aquisi��o de datalog e identifica��o de modelo em Espa�o
% de estados
% ME.ENG.Bruno Gomes Dutra
clear all; close all; clc;
%Adciona pastas das fun��es utilizadas
addpath('functions_daqduino') % fun��es de comunica��o com o arduino
addpath('ident_functions') % fun��o com algoritmos de identifica��o 
addpath('prbs_signal') % fun��o para gerar o sinal prbs
%% Etapa 1: Sinal de entrada do modelo
% Vetor de entrada do modelo, pode ser um sinal quadrado, uma entrada ao
% degrau ou at� mesmo um impulso. Obs: � interessante tamb�m adicionar um
% sinal PRBS. 

% Esse sinal � utilizado para acionar os atuadores com o objetivo de medir
% posteriormente o sinal de sa�da 

u1(1:25)=0; u1(26:200)=2; u1(201:410)=4;  u1(411:620)=0;
% Para o caso de um sistema Multivar, com v�rias entradas e sa�das 
% Podemos atribuir mais entradas, ex: u2, u3, u4 ... uN 
nit = length(u1); % numero de intera��es com base no tamanho de u.
u_prbs=create_prbs(0,0.2,0 ,9, 1, nit,1);
u1=u1+u_prbs;
% Para um bra�o rob�tico com 6 servos tem-se : 
u2=u1; u3=u1; u4=u1; u5=u1; u6=u1;


% Tempo de amostragem utilizado para aquisi��o e pro controle
  Ts = 0.05; % 0.05 segundos � recomend�vel para uma comunica��o est�vel entre arduino e o matlab

%% Etapa 2): Conex�o entre MATLAB, arduino e a planta 
delete(instrfindall); % fun��o para limpar comunica��es serias existentes
daqduino_start('COM13',9600); % Inicia a comunica��o pela porta COM(N).
%Obs: excolher a porta certa de acordo com o arduino
for i=1:3
daqduino_write_Mimo(0,0,0,0,0,Ts);
daqduino_read;
daqduino_write_Mimo(0,0,0,0,0,Ts);
end

for k=1:nit,
  Y(k,:)=daqduino_read; % Leitura dos sinais de sa�da dos servos[ de 1 a 6]
  %%plota o sinal de sa�da em tempo real(Descomentar caso queira acompanhar o sinal)
   %plot(1:k,Y(:,1),'r' ,1:k,Y(:,2),'b',1:k,Y(:,3),'m',1:k,Y(:,4),'c',1:k,Y(:,4),'y',1:k,Y(:,6),'k');
   %drawnow
  daqduino_write_Mimo(u1(k),u2(k),0,0,0,Ts); %Manda o sinal de entrada para o arduino atuar nos atuadores
end
% Y(1:10,:)=0;
daqduino_end; % End the connection to the DaqDuino device.

%% Visualisa��o do Datalog
y1=  Y(:,1); % sinal de sa�da 1 (primeiro servo)
y2=  Y(:,2); % Leitura do sinal de sa�da 2 (segundo servo)
y3=  Y(:,3); % Leitura do sinal de sa�da 3 (terceiro servo)
y4=  Y(:,4); % Leitura do sinal de sa�da 4 ...
y5=  Y(:,5); % Leitura do sinal de sa�da 5 ...
y6=  Y(:,6); % Leitura do sinal de sa�da 6 ...
 
t=0:Ts:nit*Ts-Ts; % Vetor de tempo com base em Ts
figure('units','normalized','outerposition',[.4 .05 0.559 0.925])
subplot(211)
    plot(t,y1,'r','linewidth',2.2); hold
    plot(t,y2,'b','linewidth',2.2);
    plot(t,y3,'m','linewidth',2.2);
    plot(t,y4,'c','linewidth',2.2);
    plot(t,y5,'y','linewidth',2.2);
    plot(t,y6,'k','linewidth',2.2);
    legend({'Posi��o angular 1','Posi��o angular 2','Posi��o angular 3','Posi��o angular 4','Posi��o angular 6'});

    xlabel('Tempo(s)')
    ylabel('Tens�o(V)')
    % legend('Ref_{for�a}','For�a');
    title('Datalog dos servos')
    set(gca,'fontsize',10);
    set(gca,'linewidth',1);

subplot(212)
    plot(t,u1,':r','linewidth',2.5); hold on;
    plot(t,u2,' :b','linewidth',2.4);
    plot(t,u3,' :m','linewidth',2.3);
    plot(t,u4,' :c','linewidth',2.2);
    plot(t,u5,' :y','linewidth',2.1);
    plot(t,u6,' :k','linewidth',2.0);
    legend({'Servo 1','Servo 2','Servo 3','Servo 4','Servo 5','Servo 6'});

    title('Atuadores')
    ylabel('Tens�o(V)');
    xlabel('Tempo(s)');
    set(gca,'fontsize',10);
    set(gca,'linewidth',1);
    save("datalog.mat")
%% Etapa 3) Ajuste do datalog 
U=[u1; u2; u3; u4; u5; u6]'; % vetor com todas as entradas

datalog = iddata(Y,U,Ts); % vari�vel com os dados de entada e sa�da

%% Etapa 4) Identifica��o do modelo 
ordem=2; % escolha do projetista. O tamanho das matrizes(Ordem) vai interfirir na efic�cia do modelo
numero_estados=6*ordem;
inputs=6
output=6
%% Etapa 4.2) Modelo Arx 
%identifica um modelo arx para cada reala��o de entrada e sa�da
% e suas respectivas rela��es de acoplamento
% ap�s isso encontra-se o seu equivalente em Espado de Estados (SS) 
d=size(datalog);
output=1;
inputs=1;
for i= 1:d(2)
    sys(:,:,i) =armax(datalog(:,i,i),[ordem*ones(output,inputs), ordem*ones(output,inputs), zeros(output,inputs), zeros(inputs)]);
end
for i=1:d(2)
        Gz(i,i)=tf(tf(sys(:,:,i)));
end
Gzmin=ss(Gz,'min');% Para criar o modelo no espa�o de estados
[Ap,Bp,Cp,Dp,Ts]=ssdata(Gzmin);% Para obter as matrizes A B C
SS_model1=ss(Ap,Bp,Cp,Dp,Ts)
[Y1,X1]=lsim(SS_model1,U,t,zeros(size(Ap,1),1));
%% Etapa 4.2) Minimos quadrados SS
% Utiliza-se a t�cnica de minimos quadrados recursivo (Recursive Least Square) para encontrar as matrizes A B e C do modelo 
[A,B,C,GAMA,W,V,AIC,fit,R2_,emq_]=ident_MQR_SS_master(Y,U,Ts,ordem)
SS_model2=ss(A,B,C,0,Ts); % Modelo discreto
% [Y2,X2]=dlsim(A,B,C,0,U,[ones(numero_estados,1)*Y(1,1)]);
[Y2,X2]=lsim(SS_model2,U,t,zeros(numero_estados,1));
%% Etapa 4.3) Subespa�o n4sid
% T�cnica n4sid de identifica��o multivar

[SYS3, X0]=n4sid(datalog,numero_estados); % Identifica��o do modelo em Espa�o de estados
SS_model3=ss(SYS3.A,SYS3.B,SYS3.C,SYS3.D,Ts); % Modelo discreto
% [Y3,X3]=lsim(SYS3.A,SYS3.B,SYS3.C,SYS3.D,U,t,zeros(numero_estados,1));
[Y3,X3]=lsim(SS_model3,U,t,zeros(numero_estados,1));

%% Etapa 5) Avalia��o das respostas do modelo_1 e modelo_2 identificados
%% Etapa 5.2 valida��o do modelo 1
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Valida��o do modelo2")
subplot(211),
    plot(t,y1,'r','linewidth',2); hold
    plot(t,Y1(:,1),'--r','linewidth',2.2);

    plot(t,y2,'b','linewidth',2);
    plot(t,Y1(:,2),'--b','linewidth',2.2);
    
    plot(t,y3,'m','linewidth',2);
    plot(t,Y1(:,3),'--m','linewidth',2.2);
    
    plot(t,y4,'c','linewidth',2);
    plot(t,Y1(:,4),'--c','linewidth',2.2);
    
    plot(t,y5,'y','linewidth',2);
    plot(t,Y1(:,5),'--y','linewidth',2.2);
    
    plot(t,y6,'k','linewidth',2);
    plot(t,Y1(:,6),'--k','linewidth',2.2);
    
    legend({'Y(1)_{real}','Ys(1)_{simulado}','Y(2)_{real}','Ys(2)_{simulado}','Y(3)_{real}','Ys(3)_{simulado}'...
        ,'Y(4)_{real}','Ys(4)_{simulado}','Y(5)_{real}','Ys(5)_{simulado}','Y(6)_{real}','Ys(6)_{simulado}' });

    ylabel('sa�da real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
%% Etapa 5.1 valida��o do modelo 2
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Valida��o do modelo1")
subplot(211),
    plot(t,y1,'r','linewidth',2); hold
    plot(t,Y2(:,1),'--r','linewidth',2.2);

    plot(t,y2,'b','linewidth',2);
    plot(t,Y2(:,2),'--b','linewidth',2.2);
    
    plot(t,y3,'m','linewidth',2);
    plot(t,Y2(:,3),'--m','linewidth',2.2);
    
    plot(t,y4,'c','linewidth',2);
    plot(t,Y2(:,4),'--c','linewidth',2.2);
    
    plot(t,y5,'y','linewidth',2);
    plot(t,Y2(:,5),'--y','linewidth',2.2);
    
    plot(t,y6,'k','linewidth',2);
    plot(t,Y2(:,6),'--k','linewidth',2.2);
    
    legend({'Y(1)_{real}','Ys(1)_{simulado}','Y(2)_{real}','Ys(2)_{simulado}','Y(3)_{real}','Ys(3)_{simulado}'...
        ,'Y(4)_{real}','Ys(4)_{simulado}','Y(5)_{real}','Ys(5)_{simulado}','Y(6)_{real}','Ys(6)_{simulado}' });

    ylabel('sa�da real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
%% Etapa 5.2 valida��o do modelo 3
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Valida��o do modelo2")
subplot(211),
    plot(t,y1,'r','linewidth',2); hold
    plot(t,Y3(:,1),'--r','linewidth',2.2);

    plot(t,y2,'b','linewidth',2);
    plot(t,Y3(:,2),'--b','linewidth',2.2);
    
    plot(t,y3,'m','linewidth',2);
    plot(t,Y3(:,3),'--m','linewidth',2.2);
    
    plot(t,y4,'c','linewidth',2);
    plot(t,Y3(:,4),'--c','linewidth',2.2);
    
    plot(t,y5,'y','linewidth',2);
    plot(t,Y3(:,5),'--y','linewidth',2.2);
    
    plot(t,y6,'k','linewidth',2);
    plot(t,Y3(:,6),'--k','linewidth',2.2);
    
    legend({'Y(1)_{real}','Ys(1)_{simulado}','Y(2)_{real}','Ys(2)_{simulado}','Y(3)_{real}','Ys(3)_{simulado}'...
        ,'Y(4)_{real}','Ys(4)_{simulado}','Y(5)_{real}','Ys(5)_{simulado}','Y(6)_{real}','Ys(6)_{simulado}' });

    ylabel('sa�da real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
    
    
