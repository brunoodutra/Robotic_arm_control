function [A,B,C,GAMA,W,V,AIC,fit,R2_,emq_]=ident_MQR_SS_master(y,u,Ts_,n)
% 

Ni = length(y); % numero de amostras / numero de iteracoes
N_o= size(y,2); % n�mero de sa�das   
N_i=size(u,2); % numero de entradas
N_em=n/N_o;% numero de estados mediso
N=n*N_o;  % Numero de estados 
% Proposicao de estados medidos/observados
% Estados do sistema para uma ordem n qualquer   
X=zeros([length(y),N]);% cria as vari�veis de estados 
jj=1;
for i=1:N_o
X(:,jj)=y(:,i);% primeiro estado atribuido � leitura do sensor(saida)

    for k=2: Ni
     for j=jj+1:N
             X(k,j)=(X(k,j-1)-X(k-1,j-1))/Ts_;
         end
    end
    
jj=jj+n;
end
%%
t_ =0:Ts_:Ni*Ts_-Ts_;
C=(inv(X'*X)*X'*y)';
%% MQR 
e=zeros([5,N_o]);
theta_hat=0.1*ones((2*N +N_i),N);  % valor inicial do vetor de par�metros. 
P=1*eye((2*N +N_i),(2*N +N_i));       % valor iNicial da matriz de covari�ncia. 
% nit=size(u,1);               % numero de amostras 
                                    % entrada-> vetor "u" do SBPA 
                                    % sa�da-> vetor "y" obtido com SBPA 
Yhat=zeros(size(y));    % vetor de sa�das estimadas 
Xhat=zeros(length(y),N);    % vetor de sa�das estimadas 
PHI=zeros((2*N +N_i),1);           % vetor de dados 
ek=0;                            % erro de previs�o 
W=Xhat;
TRACOP=zeros(size(u,1),1);
lambda=1; % fator de esquecimento
I=eye((2*N +N_i)); % Inicializar a matriz identidade
for t=5:Ni;     
  PHI = [X(t-1,:) u(t-1,:) W(t-1,:)]';  
 % (3) : Calcula a sa�da estimada pelo modelo no instante k, yhat;    
  xhat= (PHI' * theta_hat); 
  yhat=C*xhat';
  %guarda as sa�das obtidas no vetor Yhat 
  Xhat(t,:)=xhat; 
  Yhat(t,:)=yhat; 
 % (4) : Calcula o valor do erro de estima��o no instante k; 
  ex=X(t,1:N)-xhat;  
%   eX(t,:)=ex;
  ek=y(t,:)-yhat';  e(t,:)=ek;
 % (5) :  Calcula o vetor de ganho do estimador:  
  
  denominador= lambda + PHI' * P * PHI; 
  K=(1/denominador)*P*PHI; 
   
  % (6): atualiza estimativa do vetor de par�metros theta_hat; 
  
  theta_hat= theta_hat + K*ex;  
   
 % ru�do brando, processo estoc�stico 
  W(t,:)=X(t,1:N)-(PHI' * theta_hat);
  % passo 7: atualiza a matriz de covari�ncia; 
   
  P = (I-K*PHI')*P/lambda;
  TRACOP(t)= trace(P); 
   
end; %final do la�o for principal.  
% plot(THETA)     %par�metros estimados 
%% Modelo em espaco de estado obtido
  AB = theta_hat';
  A = AB(1:N,1:N);
  B = AB(1:N,N+1:N+N_i);
  GAMA = AB(:,(N+N_i+1):end);
  C=(inv(Xhat'*Xhat)*Xhat'*Yhat)';  
%% Teste do modelo identificado
xest=zeros(N,1);
yest=zeros(10,N_o);
for k=10:length(y)
%%Sa�da da planta estimada
    xest= A*xest +B*u(k-1,:)';
%    + GAMA*ex;
    yest(k,:)=C*xest;
end
  
  % Indices   
e1=y-yest;
emq = sum(e1.^2)/length(e1); % EMQ 


% Erro Quadr�tico 
e_AR = y-yest; % Erro da sa�da real e a sa�da estimada
e2_AR = e_AR.^2; % Erro quadr�tico
SEQ_AR = sum(e2_AR); % Somat�rio dos erros quadr�ticos
% Coeficiente de Correla��o M�ltipla RMS
% Indices   
e1=e_AR;
N=length(e1);
emq_ = sum(sum(e1.^2))/N;% EMQ 
AIC= (1+ n/N)*emq_;% Cr�t�rio de akaike
for j=1:N_o
m1_AR = sum((y(:,1)-mean(yest(:,1))).^2); % Soma a diferen�a entre cada amostra e a m�dia das amostras elevadas ao quadrado armazenadas
end
R2_ = (1-(SEQ_AR/m1_AR)) *100; % Coeficiente de correla��o m�ltipla
fit = goodnessOfFit(yest,y,'NRMSE')*100;
% end
% 
%   figure();
%   subplot(211)
%    set(gca,'fontsize',12);
%    set(gca,'linewidth',1);
%   stairs(t_,y,'k','linewidth',2.0); hold;
%   stairs(t_,yest,'b','linewidth',2.0);
% 
%      title(['NRMSE: ' num2str(fit) ' % ','  R^2: ',num2str(R2_)]);
%   subplot(212) 
%   stairs(t_,u,'linewidth',2.0);
%%   Ruido medido v(k)
  v = y-yhat'; % Erro de estimacao
  disp(['Ruido de medida v(k): media=' num2str(mean(v)) '; variancia=' num2str(var(v))]);
  W=[var(W)];
  V=[var(v)];
end