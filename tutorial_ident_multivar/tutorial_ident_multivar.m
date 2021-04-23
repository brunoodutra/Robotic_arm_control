%% Aquisição e identificação multivar:
% Tutorial para a aquisição de datalog e identificação de modelo em Espaço
% de estados
% ME.ENG.Bruno Gomes Dutra
clear all; close all; clc;
%Adciona pastas das funções utilizadas
addpath('functions_daqduino') % funções de comunicação com o arduino
addpath('ident_functions') % função com algoritmos de identificação 
addpath('prbs_signal') % função para gerar o sinal prbs
%% Etapa 1: Sinal de entrada do modelo
% Vetor de entrada do modelo, pode ser um sinal quadrado, uma entrada ao
% degrau ou até mesmo um impulso. Obs: è interessante também adicionar um
% sinal PRBS. 

% Esse sinal é utilizado para acionar os atuadores com o objetivo de medir
% posteriormente o sinal de saída 

u1(1:25)=0; u1(26:200)=2; u1(201:410)=4;  u1(411:620)=0;
% Para o caso de um sistema Multivar, com várias entradas e saídas 
% Podemos atribuir mais entradas, ex: u2, u3, u4 ... uN 
nit = length(u1); % numero de interações com base no tamanho de u.
u_prbs=create_prbs(0,0.2,0 ,9, 1, nit,1);
u1=u1+u_prbs;
% Para um braço robótico com 6 servos tem-se : 
u2=u1; u3=u1; u4=u1; u5=u1; u6=u1;


% Tempo de amostragem utilizado para aquisição e pro controle
  Ts = 0.05; % 0.05 segundos é recomendável para uma comunicação estável entre arduino e o matlab

%% Etapa 2): Conexão entre MATLAB, arduino e a planta 
delete(instrfindall); % função para limpar comunicações serias existentes
daqduino_start('COM13',9600); % Inicia a comunicação pela porta COM(N).
%Obs: excolher a porta certa de acordo com o arduino
for i=1:3
daqduino_write_Mimo(0,0,0,0,0,Ts);
daqduino_read;
daqduino_write_Mimo(0,0,0,0,0,Ts);
end

for k=1:nit,
  Y(k,:)=daqduino_read; % Leitura dos sinais de saída dos servos[ de 1 a 6]
  %%plota o sinal de saída em tempo real(Descomentar caso queira acompanhar o sinal)
   %plot(1:k,Y(:,1),'r' ,1:k,Y(:,2),'b',1:k,Y(:,3),'m',1:k,Y(:,4),'c',1:k,Y(:,4),'y',1:k,Y(:,6),'k');
   %drawnow
  daqduino_write_Mimo(u1(k),u2(k),0,0,0,Ts); %Manda o sinal de entrada para o arduino atuar nos atuadores
end
% Y(1:10,:)=0;
daqduino_end; % End the connection to the DaqDuino device.

%% Visualisação do Datalog
y1=  Y(:,1); % sinal de saída 1 (primeiro servo)
y2=  Y(:,2); % Leitura do sinal de saída 2 (segundo servo)
y3=  Y(:,3); % Leitura do sinal de saída 3 (terceiro servo)
y4=  Y(:,4); % Leitura do sinal de saída 4 ...
y5=  Y(:,5); % Leitura do sinal de saída 5 ...
y6=  Y(:,6); % Leitura do sinal de saída 6 ...
 
t=0:Ts:nit*Ts-Ts; % Vetor de tempo com base em Ts
figure('units','normalized','outerposition',[.4 .05 0.559 0.925])
subplot(211)
    plot(t,y1,'r','linewidth',2.2); hold
    plot(t,y2,'b','linewidth',2.2);
    plot(t,y3,'m','linewidth',2.2);
    plot(t,y4,'c','linewidth',2.2);
    plot(t,y5,'y','linewidth',2.2);
    plot(t,y6,'k','linewidth',2.2);
    legend({'Posição angular 1','Posição angular 2','Posição angular 3','Posição angular 4','Posição angular 6'});

    xlabel('Tempo(s)')
    ylabel('Tensão(V)')
    % legend('Ref_{força}','Força');
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
    ylabel('Tensão(V)');
    xlabel('Tempo(s)');
    set(gca,'fontsize',10);
    set(gca,'linewidth',1);
    save("datalog.mat")
%% Etapa 3) Ajuste do datalog 
U=[u1; u2; u3; u4; u5; u6]'; % vetor com todas as entradas

datalog = iddata(Y,U,Ts); % variável com os dados de entada e saída

%% Etapa 4) Identificação do modelo 
ordem=2; % escolha do projetista. O tamanho das matrizes(Ordem) vai interfirir na eficácia do modelo
numero_estados=6*ordem;
inputs=6
output=6
%% Etapa 4.2) Modelo Arx 
%identifica um modelo arx para cada realação de entrada e saída
% e suas respectivas relações de acoplamento
% após isso encontra-se o seu equivalente em Espado de Estados (SS) 
d=size(datalog);
output=1;
inputs=1;
for i= 1:d(2)
    sys(:,:,i) =armax(datalog(:,i,i),[ordem*ones(output,inputs), ordem*ones(output,inputs), zeros(output,inputs), zeros(inputs)]);
end
for i=1:d(2)
        Gz(i,i)=tf(tf(sys(:,:,i)));
end
Gzmin=ss(Gz,'min');% Para criar o modelo no espaço de estados
[Ap,Bp,Cp,Dp,Ts]=ssdata(Gzmin);% Para obter as matrizes A B C
SS_model1=ss(Ap,Bp,Cp,Dp,Ts)
[Y1,X1]=lsim(SS_model1,U,t,zeros(size(Ap,1),1));
%% Etapa 4.2) Minimos quadrados SS
% Utiliza-se a técnica de minimos quadrados recursivo (Recursive Least Square) para encontrar as matrizes A B e C do modelo 
[A,B,C,GAMA,W,V,AIC,fit,R2_,emq_]=ident_MQR_SS_master(Y,U,Ts,ordem)
SS_model2=ss(A,B,C,0,Ts); % Modelo discreto
% [Y2,X2]=dlsim(A,B,C,0,U,[ones(numero_estados,1)*Y(1,1)]);
[Y2,X2]=lsim(SS_model2,U,t,zeros(numero_estados,1));
%% Etapa 4.3) Subespaço n4sid
% Técnica n4sid de identificação multivar

[SYS3, X0]=n4sid(datalog,numero_estados); % Identificação do modelo em Espaço de estados
SS_model3=ss(SYS3.A,SYS3.B,SYS3.C,SYS3.D,Ts); % Modelo discreto
% [Y3,X3]=lsim(SYS3.A,SYS3.B,SYS3.C,SYS3.D,U,t,zeros(numero_estados,1));
[Y3,X3]=lsim(SS_model3,U,t,zeros(numero_estados,1));

%% Etapa 5) Avaliação das respostas do modelo_1 e modelo_2 identificados
%% Etapa 5.2 validação do modelo 1
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Validação do modelo2")
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

    ylabel('saída real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
%% Etapa 5.1 validação do modelo 2
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Validação do modelo1")
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

    ylabel('saída real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
%% Etapa 5.2 validação do modelo 3
figure('units','normalized','outerposition',[0 0 0.8 1])
title("Validação do modelo2")
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

    ylabel('saída real y(t)');
subplot(212),
    plot(t,U,'linewidth',2);
    ylabel('entradas U(t)'); xlabel('Tempo (s)');
    
    
