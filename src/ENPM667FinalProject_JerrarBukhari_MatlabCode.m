%syms('g','m1','m2','M','l1','l2') %Symbolic Parameters

%Parameters
M=1000;
m1=100;
m2=100;
l1=20;
l2=10;
g=9.8;

% Linearized A Matrix
A=[0 1 0 0 0 0
    0 0 (-g*m1)/M 0 (-g*m2)/M 0
    0 0 0 1 0 0
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0
    0 0 0 0 0 1
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0 ]

% Linearized B Matrix
B=[ 0
    1/M
    0
    1/(M*l1)
    0
    1/(M*l2)]

% Linearized C Matrix
%C = eye(6);
C = [ 1 0 0 0 0 0 %x Q1 Q2
      0 0 1 0 0 0
      0 0 0 0 1 0];

% Zero D Matrix
D=zeros(size(C,1),size(B,2));

%step(A,B,C,D)

% Controllability Matrix
controlM=[B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B]

% Controllability Determinat Check
det(controlM)

% Controllability Rank Check
rank(controlM)

% LQR State Cost Matrix
Q=eye(6,6);
 Q(1,1)=5;
 Q(2,2)=5;
 Q(3,3)=1000;
 Q(4,4)=1000;
 Q(5,5)=1000;
 Q(6,6)=1000;

% LQR Actuator Cost Matrix 
R=ones(1,1)*0.001;

% LQR Formulation
[K,S,E]=lqr(A,B,Q,R)

% LQR System Response
%step(A-B*K,B,C,D)

%LQR System Poles
poles=eig(A-BK)

% New Poles for Luenberger Observer
P=[-2 -3 -4 -5 -6 -7]
L=place(A',C',P)'
poles=eig(A-L*C)

% Closed Loop System with Luenberger Observer
Ace = [(A-B*K) (B*K);
       zeros(size(A)) (A-L*C)];
Bce = [B;
       zeros(size(B))];
Cce = [Cc zeros(size(C))];
Dce = [zeros(size(C,1),1)];

% Step Response of CLosed Loop System with Luenberger Observer
 %step(Ace,Bce,Cce,Dce)

% LQG Design
Vd= 0.1*eye(6); %Disturbance Co-Variance
Vn=0.1; %Noise Co-Variance

%Construct Kalman Filter
[Kfilter,P,E] = lqe(A,Vd,C,Vd,Vn);

%Construct kalman Filter State Space Model
sysKF=ss(A-Kfilter*C,[B Kfilter],eye(6),0*[B Kfilter]);