
%% Starting control with ADRC dynamic controller
clear
clc
N = 200;                 % Number of Iterations
T = 0.1;                 % Sampling time (sec.)

Vr = 0.25;               % Reference Linear velocity (m/s)
Wr = 0.5;                % Reference Angular velocity (rad/s)

% Initialize reference trajectory
xr = zeros(1, N);
yr = zeros(1, N);
theta_r = zeros(1, N);

% Initial conditions for reference
xr(1) = 0;
yr(1) = 0;
theta_r(1) = 0;

% Initialize robot measured states
xm = zeros(1, N); 
ym = zeros(1, N); 
theta_m = zeros(1, N);  

xm(1) = 0;
ym(1) = 0.5;   % Start along Y-axis (center of circle)
theta_m(1) = 0;

% Initialize dynamic model states
vm = zeros(1,N);    % actual linear velocity
wm = zeros(1,N);    % actual angular velocity

% Initialize vm_dot, wm_dot
vm_dot = zeros(1,N);
wm_dot = zeros(1,N);

% Control gains for Kinematic Controller & fuzzy TSK
kx = 0.9;
ky = 0.9;
kth = 0.7;

kev=2;
kcev=0.1;
kuv=7;

kew=1.9;
kcew=0.1;
kuw=6;

LLv=0.9;
LCv=0.1; 
LSv=0.1; 

LLw=0.01;
LCw=0.01; 
LSw=0.1;

% Save desired velocities
vd = zeros(1,N);
wd = zeros(1,N);

% ESO parameters
w0 =6;
bo = 12;

% Initialize ESO states (for both linear and angular)
x1_v = 0;    % ESO state 1 for linear velocity
x2_v = 0;    % ESO state 2 for linear velocity (disturbance)

x1_w = 0;    % ESO state 1 for angular velocity
x2_w = 0;    % ESO state 2 for angular velocity (disturbance)

vdd=0;
wdd=0;
dv =0;
dw=0;

%error derivative initialzie 
e_v_prev = 0;
e_w_prev = 0;
cevo=0;
cewo=0;

%% Critic Initialize
% parameters for linear

 % Number of nodes in each layer
 Ip=4;           % number of inputs
 RH=20;          % number of hidden layers
 No=1;           % number of output nodes
 etaO_v=0.3;       % Learning rate for output weights
 etaD_v=0.1;       % Learning rate for Dignoal weights
 etaI_v=0.2;       % Learning rate for Input weights
 % Intial values for the weights
 for rh=1:RH
     for p=1:Ip
         WEI_v(2,rh,p)=0.1;
     end
     WED_v(2,rh)=0.1;
     SH_v(1,rh)=0;
 end
 for no=1:No
     for rh=1:RH
         WEO_v(2,no,rh)=0.1;
     end
 end

% parameters for angular
 % Number of nodes in each layer
 Ip=4;           % number of inputs
 RH=20;          % number of hidden layers
 No=1;           % number of output nodes
 etaO_w=0.3;       % Learning rate for output weights
 etaD_w=0.1;       % Learning rate for Dignoal weights
 etaI_w=0.2;       % Learning rate for Input weights
 % Intial values for the weights
 for rh=1:RH
     for p=1:Ip
         WEI_w(2,rh,p)=0.1;
     end
     WED_w(2,rh)=0.1;
     SH_w(1,rh)=0;
 end
 for no=1:No
     for rh=1:RH
         WEO_w(2,no,rh)=0.1;
     end
 end
gam_v=0.3;                         % this value between 0 and 1
gam_w=0.5;                         % this value between 0 and 1
%% Adaptive IT2-TSK
u0_v=zeros(1,N);
u0_w=zeros(1,N);

% paramters of MF for fuzzy logic controller FOR LINEAR
% Rule 1: NE with NCE
c11v(2)=-0.5; s11uv(2)=0.25; s11lv(2)=0.15;
c21v(2)=-0.5; s21uv(2)=0.25; s21lv(2)=0.15;
% Rule 2: NE with ZCE
c12v(2)=-0.5; s12uv(2)=0.25; s12lv(2)=0.15;
c22v(2)=0; s22uv(2)=0.25; s22lv(2)=0.15;
% Rule 3: NE with PCE
c13v(2)=-0.5; s13uv(2)=0.25; s13lv(2)=0.15;
c23v(2)=0.5; s23uv(2)=0.25; s23lv(2)=0.15;
% Rule 1: ZE with NCE
c14v(2)=0; s14uv(2)=0.25; s14lv(2)=0.15;
c24v(2)=-0.5; s24uv(2)=0.25; s24lv(2)=0.15;
% Rule 5: ZE with ZCE
c15v(2)=0; s15uv(2)=0.25; s15lv(2)=0.15;
c25v(2)=0; s25uv(2)=0.25; s25lv(2)=0.15;
% Rule 6: ZE with PCE
c16v(2)=0; s16uv(2)=0.25; s16lv(2)=0.15;
c26v(2)=0.5; s26uv(2)=0.25; s26lv(2)=0.15;
% Rule 7: PE with NCE
c17v(2)=0.5; s17uv(2)=0.25; s17lv(2)=0.15;
c27v(2)=-0.5; s27uv(2)=0.25; s27lv(2)=0.15;
% Rule 8: PE with ZCE
c18v(2)=0.5; s18uv(2)=0.25; s18lv(2)=0.15;
c28v(2)=0; s28uv(2)=0.25; s28lv(2)=0.15;
% Rule 9: PE with PCE
c19v(2)=0.5; s19uv(2)=0.25; s19lv(2)=0.15;
c29v(2)=0.5; s29uv(2)=0.25; s29lv(2)=0.15;
%==============================
a11v(2)=0.1; a21v(2)=0.9;
a12v(2)=1.5; a22v(2)=0.01; % changed a15 from 1 to 1.5  for step dis output
a13v(2)=0.5; a23v(2)=0.5;
a14v(2)=0.5; a24v(2)=1;
a15v(2)=0.9; a25v(2)=0.9; % changed a15 from 0.3 to 0.9  for step dis output
a16v(2)=0.2; a26v(2)=1;
a17v(2)=0.9; a27v(2)=0.9;
a18v(2)=1; a28v(2)=0;
a19v(2)=0.8; a29v(2)=0.2;
%=============================

% paramters of MF for fuzzy logic controller FOR ANGULAR
% Rule 1: NE with NCE
c11w(2)=-0.5; s11uw(2)=0.25; s11lw(2)=0.15;
c21w(2)=-0.5; s21uw(2)=0.25; s21lw(2)=0.15;
% Rule 2: NE with ZCE
c12w(2)=-0.5; s12uw(2)=0.25; s12lw(2)=0.15;
c22w(2)=0; s22uw(2)=0.25; s22lw(2)=0.15;
% Rule 3: NE with PCE
c13w(2)=-0.5; s13uw(2)=0.25; s13lw(2)=0.15;
c23w(2)=0.5; s23uw(2)=0.25; s23lw(2)=0.15;
% Rule 1: ZE with NCE
c14w(2)=0; s14uw(2)=0.25; s14lw(2)=0.15;
c24w(2)=-0.5; s24uw(2)=0.25; s24lw(2)=0.15;
% Rule 5: ZE with ZCE
c15w(2)=0; s15uw(2)=0.25; s15lw(2)=0.15;
c25w(2)=0; s25uw(2)=0.25; s25lw(2)=0.15;
% Rule 6: ZE with PCE
c16w(2)=0; s16uw(2)=0.25; s16lw(2)=0.15;
c26w(2)=0.5; s26uw(2)=0.25; s26lw(2)=0.15;
% Rule 7: PE with NCE
c17w(2)=0.5; s17uw(2)=0.25; s17lw(2)=0.15;
c27w(2)=-0.5; s27uw(2)=0.25; s27lw(2)=0.15;
% Rule 8: PE with ZCE
c18w(2)=0.5; s18uw(2)=0.25; s18lw(2)=0.15;
c28w(2)=0; s28uw(2)=0.25; s28lw(2)=0.15;
% Rule 9: PE with PCE
c19w(2)=0.5; s19uw(2)=0.25; s19lw(2)=0.15;
c29w(2)=0.5; s29uw(2)=0.25; s29lw(2)=0.15;
%==============================
a11w(2)=0.1; a21w(2)=0.9;
a12w(2)=1.5; a22w(2)=0.01;
a13w(2)=0.5; a23w(2)=0.5;
a14w(2)=0.5; a24w(2)=1;
a15w(2)=0.9; a25w(2)=0.9;
a16w(2)=0.2; a26w(2)=1;
a17w(2)=0.9; a27w(2)=0.9;
a18w(2)=1; a28w(2)=0;
a19w(2)=0.8; a29w(2)=0.2;
%=============================

%% Parameters for dynamic model
k1 = 0.24089;
k2 = 0.2424;
k3 = -0.00093603;
k4 = 0.99629;
k5 = -0.0057256;
k6 = 1;

%% MAIN LOOP
for k = 2:N
    % Circle Change From 
    % if k==160
    % Wr=0.3;
    % end
    %% -------- Generate Reference Trajectory --------
    theta = theta_r(k-1);      % current reference orientation
    
    dx = cos(theta) * Vr * T;    % delta x
    dy = sin(theta) * Vr * T;    % delta y
    d_theta = Wr * T;            % delta theta
    
    xr(k) = xr(k-1) + dx;        
    yr(k) = yr(k-1) + dy;        
    theta_r(k) = theta_r(k-1) + d_theta;   
    theta_r(k) = atan2(sin(theta_r(k)), cos(theta_r(k)));
    %% -------- Kinematic Controller to compute vd, wd --------
    % Tracking error in robot frame
    ex = ( (xr(k) - xm(k-1)) * cos(theta_m(k-1)) ) + ( (yr(k) - ym(k-1)) * sin(theta_m(k-1)) );
    ey = -( (xr(k) - xm(k-1)) * sin(theta_m(k-1)) ) + ( (yr(k) - ym(k-1)) * cos(theta_m(k-1)) );
    etheta = theta_r(k) - theta_m(k-1);
    EX(k)=xr(k)- xm(k-1);
    EY(k)=yr(k) - ym(k-1);
    ETH(k)=theta_r(k) - theta_m(k-1);

    EX2(k) = ex^2;
    EY2(k) = ey^2;
    % Normalize etheta
    etheta = atan2(sin(etheta), cos(etheta));
    
    % Compute desired linear and angular velocities
    vd(k) = (kx * ex) + (Vr * cos(etheta));
    wd(k) = (ky * Vr * ey) + Wr + (kth * sin(etheta));
 
 %% =============== Proposed Controller for Linear ==================================
 % ================= Part 1: Actor using IT2-TSK-FPD for linear =================
    ev=vd(k)-vm(k-1);
    cev= ev-e_v_prev;

    e_v= kev*ev;
    ce_v= kcev*cev;

% Rules
r1uv=(exp(-0.5*((e_v-c11v(k))/s11uv(k))^2))*(exp(-0.5*((ce_v-c21v(k))/s21uv(k))^2)); % Fi find , gaussian formula
r2uv=(exp(-0.5*((e_v-c12v(k))/s12uv(k))^2))*(exp(-0.5*((ce_v-c22v(k))/s22uv(k))^2));
r3uv=(exp(-0.5*((e_v-c13v(k))/s13uv(k))^2))*(exp(-0.5*((ce_v-c23v(k))/s23uv(k))^2));
r4uv=(exp(-0.5*((e_v-c14v(k))/s14uv(k))^2))*(exp(-0.5*((ce_v-c24v(k))/s24uv(k))^2));
r5uv=(exp(-0.5*((e_v-c15v(k))/s15uv(k))^2))*(exp(-0.5*((ce_v-c25v(k))/s25uv(k))^2));
r6uv=(exp(-0.5*((e_v-c16v(k))/s16uv(k))^2))*(exp(-0.5*((ce_v-c26v(k))/s26uv(k))^2));
r7uv=(exp(-0.5*((e_v-c17v(k))/s17uv(k))^2))*(exp(-0.5*((ce_v-c27v(k))/s27uv(k))^2));
r8uv=(exp(-0.5*((e_v-c18v(k))/s18uv(k))^2))*(exp(-0.5*((ce_v-c28v(k))/s28uv(k))^2));
r9uv=(exp(-0.5*((e_v-c19v(k))/s19uv(k))^2))*(exp(-0.5*((ce_v-c29v(k))/s29uv(k))^2));

r1lv=(exp(-0.5*((e_v-c11v(k))/s11lv(k))^2))*(exp(-0.5*((ce_v-c21v(k))/s21lv(k))^2));
r2lv=(exp(-0.5*((e_v-c12v(k))/s12lv(k))^2))*(exp(-0.5*((ce_v-c22v(k))/s22lv(k))^2));
r3lv=(exp(-0.5*((e_v-c13v(k))/s13lv(k))^2))*(exp(-0.5*((ce_v-c23v(k))/s23lv(k))^2));
r4lv=(exp(-0.5*((e_v-c14v(k))/s14lv(k))^2))*(exp(-0.5*((ce_v-c24v(k))/s24lv(k))^2));
r5lv=(exp(-0.5*((e_v-c15v(k))/s15lv(k))^2))*(exp(-0.5*((ce_v-c25v(k))/s25lv(k))^2));
r6lv=(exp(-0.5*((e_v-c16v(k))/s16lv(k))^2))*(exp(-0.5*((ce_v-c26v(k))/s26lv(k))^2));
r7lv=(exp(-0.5*((e_v-c17v(k))/s17lv(k))^2))*(exp(-0.5*((ce_v-c27v(k))/s27lv(k))^2));
r8lv=(exp(-0.5*((e_v-c18v(k))/s18lv(k))^2))*(exp(-0.5*((ce_v-c28v(k))/s28lv(k))^2));
r9lv=(exp(-0.5*((e_v-c19v(k))/s19lv(k))^2))*(exp(-0.5*((ce_v-c29v(k))/s29lv(k))^2));
%================================
% Defuzzification
du1v=a11v(k)*e_v+a21v(k)*ce_v;
du2v=a12v(k)*e_v+a22v(k)*ce_v;
du3v=a13v(k)*e_v+a23v(k)*ce_v;
du4v=a14v(k)*e_v+a24v(k)*ce_v;
du5v=a15v(k)*e_v+a25v(k)*ce_v;
du6v=a16v(k)*e_v+a26v(k)*ce_v;
du7v=a17v(k)*e_v+a27v(k)*ce_v;
du8v=a18v(k)*e_v+a28v(k)*ce_v;
du9v=a19v(k)*e_v+a29v(k)*ce_v;
denUv=r1uv+r2uv+r3uv+r4uv+r5uv+r6uv+r7uv+r8uv+r9uv; %Fi sum
denLv=r1lv+r2lv+r3lv+r4lv+r5lv+r6lv+r7lv+r8lv+r9lv;
numUv=du1v*r1uv+du2v*r2uv+du3v*r3uv+du4v*r4uv+du5v*r5uv+du6v*r6uv+du7v*r7uv+du8v*r8uv+du9v*r9uv; %Fi * delta Ui
numLv=du1v*r1lv+du2v*r2lv+du3v*r3lv+du4v*r4lv+du5v*r5lv+du6v*r6lv+du7v*r7lv+du8v*r8lv+du9v*r9lv;

if denUv == 0
    dUV=0;
else
    dUV=numUv/denUv;  
end
if denLv == 0
    dLV=0;
else
    dLV=numLv/denLv;  
end
%===============
% Denormalization and control signal
u0_v(k+1)=kuv*(dUV+dLV)*0.5;

if u0_v(k+1)>24
     u0_v(k+1)=24;
 else
     u0_v(k+1)=u0_v(k+1);
end
%======================== End of Actor ========================

%% -------- ADRC for Linear Velocity Control --------
    
    % ESO Update (Linear velocity)
    e_eso_v = x1_v - vm(k-1);
    dx1_v = x2_v + (bo * vdd) - (2*w0)*e_eso_v;    % u=0 temporary
    dx2_v = -(w0^2) * e_eso_v;
    
    x1_v = x1_v + dx1_v * T;
    x2_v = x2_v + dx2_v * T;
    
    % Final control input for dynamic model
    vdd = 0.083 * (u0_v(k+1) - x2_v);

    
%=============================================

     
 %% =============== Proposed Controller for Angular ==================================
 % ================= Part 1: Actor using IT2-TSK-FPD for Angular =================
    ew=wd(k)-wm(k-1);
    cew=ew-e_w_prev;
    
    e_w= kew*ew;
    ce_w= kcew*cew;
    
%========================================================
% Rules
r1uw=(exp(-0.5*((e_w-c11w(k))/s11uw(k))^2))*(exp(-0.5*((ce_w-c21w(k))/s21uw(k))^2));
r2uw=(exp(-0.5*((e_w-c12w(k))/s12uw(k))^2))*(exp(-0.5*((ce_w-c22w(k))/s22uw(k))^2));
r3uw=(exp(-0.5*((e_w-c13w(k))/s13uw(k))^2))*(exp(-0.5*((ce_w-c23w(k))/s23uw(k))^2));
r4uw=(exp(-0.5*((e_w-c14w(k))/s14uw(k))^2))*(exp(-0.5*((ce_w-c24w(k))/s24uw(k))^2));
r5uw=(exp(-0.5*((e_w-c15w(k))/s15uw(k))^2))*(exp(-0.5*((ce_w-c25w(k))/s25uw(k))^2));
r6uw=(exp(-0.5*((e_w-c16w(k))/s16uw(k))^2))*(exp(-0.5*((ce_w-c26w(k))/s26uw(k))^2));
r7uw=(exp(-0.5*((e_w-c17w(k))/s17uw(k))^2))*(exp(-0.5*((ce_w-c27w(k))/s27uw(k))^2));
r8uw=(exp(-0.5*((e_w-c18w(k))/s18uw(k))^2))*(exp(-0.5*((ce_w-c28w(k))/s28uw(k))^2));
r9uw=(exp(-0.5*((e_w-c19w(k))/s19uw(k))^2))*(exp(-0.5*((ce_w-c29w(k))/s29uw(k))^2));

r1lw=(exp(-0.5*((e_w-c11w(k))/s11lw(k))^2))*(exp(-0.5*((ce_w-c21w(k))/s21lw(k))^2));
r2lw=(exp(-0.5*((e_w-c12w(k))/s12lw(k))^2))*(exp(-0.5*((ce_w-c22w(k))/s22lw(k))^2));
r3lw=(exp(-0.5*((e_w-c13w(k))/s13lw(k))^2))*(exp(-0.5*((ce_w-c23w(k))/s23lw(k))^2));
r4lw=(exp(-0.5*((e_w-c14w(k))/s14lw(k))^2))*(exp(-0.5*((ce_w-c24w(k))/s24lw(k))^2));
r5lw=(exp(-0.5*((e_w-c15w(k))/s15lw(k))^2))*(exp(-0.5*((ce_w-c25w(k))/s25lw(k))^2));
r6lw=(exp(-0.5*((e_w-c16w(k))/s16lw(k))^2))*(exp(-0.5*((ce_w-c26w(k))/s26lw(k))^2));
r7lw=(exp(-0.5*((e_w-c17w(k))/s17lw(k))^2))*(exp(-0.5*((ce_w-c27w(k))/s27lw(k))^2));
r8lw=(exp(-0.5*((e_w-c18w(k))/s18lw(k))^2))*(exp(-0.5*((ce_w-c28w(k))/s28lw(k))^2));
r9lw=(exp(-0.5*((e_w-c19w(k))/s19lw(k))^2))*(exp(-0.5*((ce_w-c29w(k))/s29lw(k))^2));
%================================
% Defuzzification
du1w=a11w(k)*e_w+a21w(k)*ce_w;
du2w=a12w(k)*e_w+a22w(k)*ce_w;
du3w=a13w(k)*e_w+a23w(k)*ce_w;
du4w=a14w(k)*e_w+a24w(k)*ce_w;
du5w=a15w(k)*e_w+a25w(k)*ce_w;
du6w=a16w(k)*e_w+a26w(k)*ce_w;
du7w=a17w(k)*e_w+a27w(k)*ce_w;
du8w=a18w(k)*e_w+a28w(k)*ce_w;
du9w=a19w(k)*e_w+a29w(k)*ce_w;
denUw=r1uw+r2uw+r3uw+r4uw+r5uw+r6uw+r7uw+r8uw+r9uw;
denLw=r1lw+r2lw+r3lw+r4lw+r5lw+r6lw+r7lw+r8lw+r9lw;
numUw=du1w*r1uw+du2w*r2uw+du3w*r3uw+du4w*r4uw+du5w*r5uw+du6w*r6uw+du7w*r7uw+du8w*r8uw+du9w*r9uw;
numLw=du1w*r1lw+du2w*r2lw+du3w*r3lw+du4w*r4lw+du5w*r5lw+du6w*r6lw+du7w*r7lw+du8w*r8lw+du9w*r9lw;

if denUw == 0
    dUW=0;
else
    dUW=numUw/denUw;  
end
if denLw == 0
    dLW=0;
else
    dLW=numLw/denLw;  
end
%===============
% Denormalization and control signal
u0_w(k+1)=kuw*(dUW+dLW)*0.5;

if u0_w(k+1)>24
     u0_w(k+1)=24;
 else
     u0_w(k+1)=u0_w(k+1);
end

  %% -------- ADRC for Angular Velocity Control --------
    
    % ESO Update (Angular velocity)
    e_eso_w = x1_w - wm(k-1);
    dx1_w = x2_w + (bo * wdd) - (2*w0)*e_eso_w;
    dx2_w = -(w0^2) * e_eso_w;
    
    x1_w = x1_w + dx1_w * T;
    x2_w = x2_w + dx2_w * T;
    
    % Final control input for dynamic model
    wdd = 0.083 * (u0_w(k+1) - x2_w);
    
%=============================================



    %% -------- Dynamic model --------
    vc= vdd;
    wc=wdd;
     % if k>=80
     %     vc = vc + 0.99;   
     %     wc = wc + 0.99;
     % end
      if k>=80
          dv = 3.9;
          dw = 3.9;
     end
     % if k>=110 && k<=190
     %      vc = vc + 0.6 * sin(0.3*pi*k*T) + 0.6 * cos(0.3*pi*k*T);   
     %      wc = wc + 0.6 * sin(0.3*pi*k*T) + 0.6 * sin(0.3*pi*k*T);
     %  end

     % if k>=110 && k<=190
     %     dv = 2.5 * sin(0.3*pi*k*T) + 2.5 * cos(0.3*pi*k*T);
     %     dw = 2.5 * sin(0.3*pi*k*T) + 2.5 * sin(0.3*pi*k*T);
     % end

    
    vm_dot(k) = ( (k3/k1) * (wm(k-1)^2) ) - ( (k4/k1) * vm(k-1) ) + (vc/k1) + dv;
    wm_dot(k) = - ( (k5/k2) * (vm(k-1)*wm(k-1)) ) - ( (k6/k2) * wm(k-1) ) + (wc/k2) + dw;
    
    % Integrate to update vm and wm
    vm(k) = vm(k-1) + vm_dot(k) * T;
    wm(k) = wm(k-1) + wm_dot(k) * T;
    
 %% -------- Update robot position --------
    % Robot motion
    dxm = vm(k) * cos(theta_m(k-1)) * T;
    dym = vm(k) * sin(theta_m(k-1)) * T;
    
    xm(k) = xm(k-1) + dxm;
    ym(k) = ym(k-1) + dym;
    theta_m(k) = theta_m(k-1) + wm(k) * T;
    
    % Normalize theta_m
    theta_m(k) = atan2(sin(theta_m(k)), cos(theta_m(k)));
    MAEALL(k)=sum(abs(EX)+abs(EY))/k;

%=======================================
%Part 2: Update the actor
% Updating consequent Parameters
H1=-1*(XncV(k));
%H2=(ym3(k+3)-ym3(k+2))/(du3(k+1)-du3(k)+0.00001);
H2=1;
H11=0.5*(r1lv/(denLv+0.00001))+0.5*(r1uv/(denUv+0.00001));
H12=0.5*(r2lv/(denLv+0.00001))+0.5*(r2uv/(denUv+0.00001));
H13=0.5*(r3lv/(denLv+0.00001))+0.5*(r3uv/(denUv+0.00001));
H14=0.5*(r4lv/(denLv+0.00001))+0.5*(r4uv/(denUv+0.00001));
H15=0.5*(r5lv/(denLv+0.00001))+0.5*(r5uv/(denUv+0.00001));
H16=0.5*(r6lv/(denLv+0.00001))+0.5*(r6uv/(denUv+0.00001));
H17=0.5*(r7lv/(denLv+0.00001))+0.5*(r7uv/(denUv+0.00001));
H18=0.5*(r8lv/(denLv+0.00001))+0.5*(r8uv/(denUv+0.00001));
H19=0.5*(r9lv/(denLv+0.00001))+0.5*(r9uv/(denUv+0.00001));


%LLv=0.0;
a11v(k+1)=a11v(k)-LLv*H1*H2*H11*e_v;
a12v(k+1)=a12v(k)-LLv*H1*H2*H12*e_v;
a13v(k+1)=a13v(k)-LLv*H1*H2*H13*e_v;
a14v(k+1)=a14v(k)-LLv*H1*H2*H14*e_v;
a15v(k+1)=a15v(k)-LLv*H1*H2*H15*e_v;
a16v(k+1)=a16v(k)-LLv*H1*H2*H16*e_v;
a17v(k+1)=a17v(k)-LLv*H1*H2*H17*e_v;
a18v(k+1)=a18v(k)-LLv*H1*H2*H18*e_v;
a19v(k+1)=a19v(k)-LLv*H1*H2*H19*e_v;

a21v(k+1)=a21v(k)-LLv*H1*H2*H11*ce_v;
a22v(k+1)=a22v(k)-LLv*H1*H2*H12*ce_v;
a23v(k+1)=a23v(k)-LLv*H1*H2*H13*ce_v;
a24v(k+1)=a24v(k)-LLv*H1*H2*H14*ce_v;
a25v(k+1)=a25v(k)-LLv*H1*H2*H15*ce_v;
a26v(k+1)=a26v(k)-LLv*H1*H2*H16*ce_v;
a27v(k+1)=a27v(k)-LLv*H1*H2*H17*ce_v;
a28v(k+1)=a28v(k)-LLv*H1*H2*H18*ce_v;
a29v(k+1)=a29v(k)-LLv*H1*H2*H19*ce_v;
%=========================================================================
% Updating antecedent Parameters
Hc11=0.5*((du1v-dLV)/(denLv+0.00001))+0.5*((du1v-dUV)/(denUv+0.00001));
Hc12=0.5*((du2v-dLV)/(denLv+0.00001))+0.5*((du2v-dUV)/(denUv+0.00001));
Hc13=0.5*((du3v-dLV)/(denLv+0.00001))+0.5*((du3v-dUV)/(denUv+0.00001));
Hc14=0.5*((du4v-dLV)/(denLv+0.00001))+0.5*((du4v-dUV)/(denUv+0.00001));
Hc15=0.5*((du5v-dLV)/(denLv+0.00001))+0.5*((du5v-dUV)/(denUv+0.00001));
Hc16=0.5*((du6v-dLV)/(denLv+0.00001))+0.5*((du6v-dUV)/(denUv+0.00001));
Hc17=0.5*((du7v-dLV)/(denLv+0.00001))+0.5*((du7v-dUV)/(denUv+0.00001));
Hc18=0.5*((du8v-dLV)/(denLv+0.00001))+0.5*((du8v-dUV)/(denUv+0.00001));
Hc19=0.5*((du9v-dLV)/(denLv+0.00001))+0.5*((du9v-dUV)/(denUv+0.00001));

HFc11=r1uv*((e_v-c11v(k))/s11uv(k)^2)+r1lv*((e_v-c11v(k))/s11lv(k)^2);
HFc12=r2uv*((e_v-c12v(k))/s12uv(k)^2)+r2lv*((e_v-c12v(k))/s12lv(k)^2);
HFc13=r3uv*((e_v-c13v(k))/s13uv(k)^2)+r3lv*((e_v-c13v(k))/s13lv(k)^2);
HFc14=r4uv*((e_v-c14v(k))/s14uv(k)^2)+r4lv*((e_v-c14v(k))/s14lv(k)^2);
HFc15=r5uv*((e_v-c15v(k))/s15uv(k)^2)+r5lv*((e_v-c15v(k))/s15lv(k)^2);
HFc16=r6uv*((e_v-c16v(k))/s16uv(k)^2)+r6lv*((e_v-c16v(k))/s16lv(k)^2);
HFc17=r7uv*((e_v-c17v(k))/s17uv(k)^2)+r7lv*((e_v-c17v(k))/s17lv(k)^2);
HFc18=r8uv*((e_v-c18v(k))/s18uv(k)^2)+r8lv*((e_v-c18v(k))/s18lv(k)^2);
HFc19=r9uv*((e_v-c19v(k))/s19uv(k)^2)+r9lv*((e_v-c19v(k))/s19lv(k)^2);

HFc21=r1uv*((ce_v-c21v(k))/s21uv(k)^2)+r1lv*((ce_v-c21v(k))/s21lv(k)^2);
HFc22=r2uv*((ce_v-c22v(k))/s22uv(k)^2)+r2lv*((ce_v-c22v(k))/s22lv(k)^2);
HFc23=r3uv*((ce_v-c23v(k))/s23uv(k)^2)+r3lv*((ce_v-c23v(k))/s23lv(k)^2);
HFc24=r4uv*((ce_v-c24v(k))/s24uv(k)^2)+r4lv*((ce_v-c24v(k))/s24lv(k)^2);
HFc25=r5uv*((ce_v-c25v(k))/s25uv(k)^2)+r5lv*((ce_v-c25v(k))/s25lv(k)^2);
HFc26=r6uv*((ce_v-c26v(k))/s26uv(k)^2)+r6lv*((ce_v-c26v(k))/s26lv(k)^2);
HFc27=r7uv*((ce_v-c27v(k))/s27uv(k)^2)+r7lv*((ce_v-c27v(k))/s27lv(k)^2);
HFc28=r8uv*((ce_v-c28v(k))/s28uv(k)^2)+r8lv*((ce_v-c28v(k))/s28lv(k)^2);
HFc29=r9uv*((ce_v-c29v(k))/s29uv(k)^2)+r9lv*((ce_v-c29v(k))/s29lv(k)^2);


%LCv=0.0;
c11v(k+1)=c11v(k)-LCv*H1*H2*Hc11*HFc11;
c12v(k+1)=c12v(k)-LCv*H1*H2*Hc12*HFc12;
c13v(k+1)=c13v(k)-LCv*H1*H2*Hc13*HFc13;
c14v(k+1)=c14v(k)-LCv*H1*H2*Hc14*HFc14;
c15v(k+1)=c15v(k)-LCv*H1*H2*Hc15*HFc15;
c16v(k+1)=c16v(k)-LCv*H1*H2*Hc16*HFc16;
c17v(k+1)=c17v(k)-LCv*H1*H2*Hc17*HFc17;
c18v(k+1)=c18v(k)-LCv*H1*H2*Hc18*HFc18;
c19v(k+1)=c19v(k)-LCv*H1*H2*Hc19*HFc19;

c21v(k+1)=c21v(k)-LCv*H1*H2*Hc11*HFc21;
c22v(k+1)=c22v(k)-LCv*H1*H2*Hc12*HFc22;
c23v(k+1)=c23v(k)-LCv*H1*H2*Hc13*HFc23;
c24v(k+1)=c24v(k)-LCv*H1*H2*Hc14*HFc24;
c25v(k+1)=c25v(k)-LCv*H1*H2*Hc15*HFc25;
c26v(k+1)=c26v(k)-LCv*H1*H2*Hc16*HFc26;
c27v(k+1)=c27v(k)-LCv*H1*H2*Hc17*HFc27;
c28v(k+1)=c28v(k)-LCv*H1*H2*Hc18*HFc28;
c29v(k+1)=c29v(k)-LCv*H1*H2*Hc19*HFc29;
%============================
Hs11u=0.5*((du1v-dUV)/(denUv+0.00001));
Hs12u=0.5*((du2v-dUV)/(denUv+0.00001));
Hs13u=0.5*((du3v-dUV)/(denUv+0.00001));
Hs14u=0.5*((du4v-dUV)/(denUv+0.00001));
Hs15u=0.5*((du5v-dUV)/(denUv+0.00001));
Hs16u=0.5*((du6v-dUV)/(denUv+0.00001));
Hs17u=0.5*((du7v-dUV)/(denUv+0.00001));
Hs18u=0.5*((du8v-dUV)/(denUv+0.00001));
Hs19u=0.5*((du9v-dUV)/(denUv+0.00001));

Hs11l=0.5*((du1v-dLV)/(denLv+0.00001)); %.....
Hs12l=0.5*((du2v-dLV)/(denLv+0.00001));
Hs13l=0.5*((du3v-dLV)/(denLv+0.00001));
Hs14l=0.5*((du4v-dLV)/(denLv+0.00001));
Hs15l=0.5*((du5v-dLV)/(denLv+0.00001));
Hs16l=0.5*((du6v-dLV)/(denLv+0.00001));
Hs17l=0.5*((du7v-dLV)/(denLv+0.00001));
Hs18l=0.5*((du8v-dLV)/(denLv+0.00001));
Hs19l=0.5*((du9v-dLV)/(denLv+0.00001));

HFs11u=r1uv*(((e_v-c11v(k))^2)/(s11uv(k)^3));
HFs12u=r2uv*(((e_v-c12v(k))^2)/(s12uv(k)^3));
HFs13u=r3uv*(((e_v-c13v(k))^2)/(s13uv(k)^3));
HFs14u=r4uv*(((e_v-c14v(k))^2)/(s14uv(k)^3));
HFs15u=r5uv*(((e_v-c15v(k))^2)/(s15uv(k)^3));
HFs16u=r6uv*(((e_v-c16v(k))^2)/(s16uv(k)^3));
HFs17u=r7uv*(((e_v-c17v(k))^2)/(s17uv(k)^3));
HFs18u=r8uv*(((e_v-c18v(k))^2)/(s18uv(k)^3));
HFs19u=r9uv*(((e_v-c19v(k))^2)/(s19uv(k)^3));

HFs11l=r1lv*(((e_v-c11v(k))^2)/(s11lv(k)^3));
HFs12l=r2lv*(((e_v-c12v(k))^2)/(s12lv(k)^3));
HFs13l=r3lv*(((e_v-c13v(k))^2)/(s13lv(k)^3));
HFs14l=r4lv*(((e_v-c14v(k))^2)/(s14lv(k)^3));
HFs15l=r5lv*(((e_v-c15v(k))^2)/(s15lv(k)^3));
HFs16l=r6lv*(((e_v-c16v(k))^2)/(s16lv(k)^3));
HFs17l=r7lv*(((e_v-c17v(k))^2)/(s17lv(k)^3));
HFs18l=r8lv*(((e_v-c18v(k))^2)/(s18lv(k)^3));
HFs19l=r9lv*(((e_v-c19v(k))^2)/(s19lv(k)^3));

HFs21u=r1uv*(((ce_v-c21v(k))^2)/(s21uv(k)^3));
HFs22u=r2uv*(((ce_v-c22v(k))^2)/(s22uv(k)^3));
HFs23u=r3uv*(((ce_v-c23v(k))^2)/(s23uv(k)^3));
HFs24u=r4uv*(((ce_v-c24v(k))^2)/(s24uv(k)^3));
HFs25u=r5uv*(((ce_v-c25v(k))^2)/(s25uv(k)^3));
HFs26u=r6uv*(((ce_v-c26v(k))^2)/(s26uv(k)^3));
HFs27u=r7uv*(((ce_v-c27v(k))^2)/(s27uv(k)^3));
HFs28u=r8uv*(((ce_v-c28v(k))^2)/(s28uv(k)^3));
HFs29u=r9uv*(((ce_v-c29v(k))^2)/(s29uv(k)^3));

HFs21l=r1lv*(((ce_v-c21v(k))^2)/(s21lv(k)^3)); %.....
HFs22l=r2lv*(((ce_v-c22v(k))^2)/(s22lv(k)^3));
HFs23l=r3lv*(((ce_v-c23v(k))^2)/(s23lv(k)^3));
HFs24l=r4lv*(((ce_v-c24v(k))^2)/(s24lv(k)^3));
HFs25l=r5lv*(((ce_v-c25v(k))^2)/(s25lv(k)^3));
HFs26l=r6lv*(((ce_v-c26v(k))^2)/(s26lv(k)^3));
HFs27l=r7lv*(((ce_v-c27v(k))^2)/(s27lv(k)^3));
HFs28l=r8lv*(((ce_v-c28v(k))^2)/(s28lv(k)^3));
HFs29l=r9lv*(((ce_v-c29v(k))^2)/(s29lv(k)^3));


%LSv=0.0;
s11uv(k+1)=s11uv(k)-LSv*H1*H2*Hs11u*HFs11u;
s12uv(k+1)=s12uv(k)-LSv*H1*H2*Hs12u*HFs12u;
s13uv(k+1)=s13uv(k)-LSv*H1*H2*Hs13u*HFs13u;
s14uv(k+1)=s14uv(k)-LSv*H1*H2*Hs14u*HFs14u;
s15uv(k+1)=s15uv(k)-LSv*H1*H2*Hs15u*HFs15u;
s16uv(k+1)=s16uv(k)-LSv*H1*H2*Hs16u*HFs16u;
s17uv(k+1)=s17uv(k)-LSv*H1*H2*Hs17u*HFs17u;
s18uv(k+1)=s18uv(k)-LSv*H1*H2*Hs18u*HFs18u;
s19uv(k+1)=s19uv(k)-LSv*H1*H2*Hs19u*HFs19u;

s21uv(k+1)=s21uv(k)-LSv*H1*H2*Hs11u*HFs21u;
s22uv(k+1)=s22uv(k)-LSv*H1*H2*Hs12u*HFs22u;
s23uv(k+1)=s23uv(k)-LSv*H1*H2*Hs13u*HFs23u;
s24uv(k+1)=s24uv(k)-LSv*H1*H2*Hs14u*HFs24u;
s25uv(k+1)=s25uv(k)-LSv*H1*H2*Hs15u*HFs25u;
s26uv(k+1)=s26uv(k)-LSv*H1*H2*Hs16u*HFs26u;
s27uv(k+1)=s27uv(k)-LSv*H1*H2*Hs17u*HFs27u;
s28uv(k+1)=s28uv(k)-LSv*H1*H2*Hs18u*HFs28u;
s29uv(k+1)=s29uv(k)-LSv*H1*H2*Hs19u*HFs29u;

s11lv(k+1)=s11lv(k)-LSv*H1*H2*Hs11l*HFs11l;
s12lv(k+1)=s12lv(k)-LSv*H1*H2*Hs12l*HFs12l;
s13lv(k+1)=s13lv(k)-LSv*H1*H2*Hs13l*HFs13l;
s14lv(k+1)=s14lv(k)-LSv*H1*H2*Hs14l*HFs14l;
s15lv(k+1)=s15lv(k)-LSv*H1*H2*Hs15l*HFs15l;
s16lv(k+1)=s16lv(k)-LSv*H1*H2*Hs16l*HFs16l;
s17lv(k+1)=s17lv(k)-LSv*H1*H2*Hs17l*HFs17l;
s18lv(k+1)=s18lv(k)-LSv*H1*H2*Hs18l*HFs18l;
s19lv(k+1)=s19lv(k)-LSv*H1*H2*Hs19l*HFs19l;

s21lv(k+1)=s21lv(k)-LSv*H1*H2*Hs11l*HFs21l;
s22lv(k+1)=s22lv(k)-LSv*H1*H2*Hs12l*HFs22l;
s23lv(k+1)=s23lv(k)-LSv*H1*H2*Hs13l*HFs23l;
s24lv(k+1)=s24lv(k)-LSv*H1*H2*Hs14l*HFs24l;
s25lv(k+1)=s25lv(k)-LSv*H1*H2*Hs15l*HFs25l;
s26lv(k+1)=s26lv(k)-LSv*H1*H2*Hs16l*HFs26l;
s27lv(k+1)=s27lv(k)-LSv*H1*H2*Hs17l*HFs27l;
s28lv(k+1)=s28lv(k)-LSv*H1*H2*Hs18l*HFs28l;
s29lv(k+1)=s29lv(k)-LSv*H1*H2*Hs19l*HFs29l;

% update values
cevo=cev;
e_v_prev = ev;

%=======================================

%Part 2: Update the actor
% Updating consequent Parameters
H1=-1*(XncW(k));
%H2=(ym3(k+3)-ym3(k+2))/(du3(k+1)-du3(k)+0.00001);
H2=1;
H11=0.5*(r1lw/(denLw+0.00001))+0.5*(r1uw/(denUw+0.00001));
H12=0.5*(r2lw/(denLw+0.00001))+0.5*(r2uw/(denUw+0.00001));
H13=0.5*(r3lw/(denLw+0.00001))+0.5*(r3uw/(denUw+0.00001));
H14=0.5*(r4lw/(denLw+0.00001))+0.5*(r4uw/(denUw+0.00001));
H15=0.5*(r5lw/(denLw+0.00001))+0.5*(r5uw/(denUw+0.00001));
H16=0.5*(r6lw/(denLw+0.00001))+0.5*(r6uw/(denUw+0.00001));
H17=0.5*(r7lw/(denLw+0.00001))+0.5*(r7uw/(denUw+0.00001));
H18=0.5*(r8lw/(denLw+0.00001))+0.5*(r8uw/(denUw+0.00001));
H19=0.5*(r9lw/(denLw+0.00001))+0.5*(r9uw/(denUw+0.00001));


%LLw=0.0;
a11w(k+1)=a11w(k)-LLw*H1*H2*H11*e_w;
a12w(k+1)=a12w(k)-LLw*H1*H2*H12*e_w;
a13w(k+1)=a13w(k)-LLw*H1*H2*H13*e_w;
a14w(k+1)=a14w(k)-LLw*H1*H2*H14*e_w;
a15w(k+1)=a15w(k)-LLw*H1*H2*H15*e_w;
a16w(k+1)=a16w(k)-LLw*H1*H2*H16*e_w;
a17w(k+1)=a17w(k)-LLw*H1*H2*H17*e_w;
a18w(k+1)=a18w(k)-LLw*H1*H2*H18*e_w;
a19w(k+1)=a19w(k)-LLw*H1*H2*H19*e_w;

a21w(k+1)=a21w(k)-LLw*H1*H2*H11*ce_w;
a22w(k+1)=a22w(k)-LLw*H1*H2*H12*ce_w;
a23w(k+1)=a23w(k)-LLw*H1*H2*H13*ce_w;
a24w(k+1)=a24w(k)-LLw*H1*H2*H14*ce_w;
a25w(k+1)=a25w(k)-LLw*H1*H2*H15*ce_w;
a26w(k+1)=a26w(k)-LLw*H1*H2*H16*ce_w;
a27w(k+1)=a27w(k)-LLw*H1*H2*H17*ce_w;
a28w(k+1)=a28w(k)-LLw*H1*H2*H18*ce_w;
a29w(k+1)=a29w(k)-LLw*H1*H2*H19*ce_w;
%=========================================================================
% Updating antecedent Parameters
Hc11=0.5*((du1w-dLW)/(denLw+0.00001))+0.5*((du1w-dUW)/(denUw+0.00001));
Hc12=0.5*((du2w-dLW)/(denLw+0.00001))+0.5*((du2w-dUW)/(denUw+0.00001));
Hc13=0.5*((du3w-dLW)/(denLw+0.00001))+0.5*((du3w-dUW)/(denUw+0.00001));
Hc14=0.5*((du4w-dLW)/(denLw+0.00001))+0.5*((du4w-dUW)/(denUw+0.00001));
Hc15=0.5*((du5w-dLW)/(denLw+0.00001))+0.5*((du5w-dUW)/(denUw+0.00001));
Hc16=0.5*((du6w-dLW)/(denLw+0.00001))+0.5*((du6w-dUW)/(denUw+0.00001));
Hc17=0.5*((du7w-dLW)/(denLw+0.00001))+0.5*((du7w-dUW)/(denUw+0.00001));
Hc18=0.5*((du8w-dLW)/(denLw+0.00001))+0.5*((du8w-dUW)/(denUw+0.00001));
Hc19=0.5*((du9w-dLW)/(denLw+0.00001))+0.5*((du9w-dUW)/(denUw+0.00001));

HFc11=r1uw*((e_w-c11w(k))/s11uw(k)^2)+r1lw*((e_w-c11w(k))/s11lw(k)^2);
HFc12=r2uw*((e_w-c12w(k))/s12uw(k)^2)+r2lw*((e_w-c12w(k))/s12lw(k)^2);
HFc13=r3uw*((e_w-c13w(k))/s13uw(k)^2)+r3lw*((e_w-c13w(k))/s13lw(k)^2);
HFc14=r4uw*((e_w-c14w(k))/s14uw(k)^2)+r4lw*((e_w-c14w(k))/s14lw(k)^2);
HFc15=r5uw*((e_w-c15w(k))/s15uw(k)^2)+r5lw*((e_w-c15w(k))/s15lw(k)^2);
HFc16=r6uw*((e_w-c16w(k))/s16uw(k)^2)+r6lw*((e_w-c16w(k))/s16lw(k)^2);
HFc17=r7uw*((e_w-c17w(k))/s17uw(k)^2)+r7lw*((e_w-c17w(k))/s17lw(k)^2);
HFc18=r8uw*((e_w-c18w(k))/s18uw(k)^2)+r8lw*((e_w-c18w(k))/s18lw(k)^2);
HFc19=r9uw*((e_w-c19w(k))/s19uw(k)^2)+r9lw*((e_w-c19w(k))/s19lw(k)^2);

HFc21=r1uw*((ce_w-c21w(k))/s21uw(k)^2)+r1lw*((ce_w-c21w(k))/s21lw(k)^2);
HFc22=r2uw*((ce_w-c22w(k))/s22uw(k)^2)+r2lw*((ce_w-c22w(k))/s22lw(k)^2);
HFc23=r3uw*((ce_w-c23w(k))/s23uw(k)^2)+r3lw*((ce_w-c23w(k))/s23lw(k)^2);
HFc24=r4uw*((ce_w-c24w(k))/s24uw(k)^2)+r4lw*((ce_w-c24w(k))/s24lw(k)^2);
HFc25=r5uw*((ce_w-c25w(k))/s25uw(k)^2)+r5lw*((ce_w-c25w(k))/s25lw(k)^2);
HFc26=r6uw*((ce_w-c26w(k))/s26uw(k)^2)+r6lw*((ce_w-c26w(k))/s26lw(k)^2);
HFc27=r7uw*((ce_w-c27w(k))/s27uw(k)^2)+r7lw*((ce_w-c27w(k))/s27lw(k)^2);
HFc28=r8uw*((ce_w-c28w(k))/s28uw(k)^2)+r8lw*((ce_w-c28w(k))/s28lw(k)^2);
HFc29=r9uw*((ce_w-c29w(k))/s29uw(k)^2)+r9lw*((ce_w-c29w(k))/s29lw(k)^2);

%LCw=0.0;
c11w(k+1)=c11w(k)-LCw*H1*H2*Hc11*HFc11;
c12w(k+1)=c12w(k)-LCw*H1*H2*Hc12*HFc12;
c13w(k+1)=c13w(k)-LCw*H1*H2*Hc13*HFc13;
c14w(k+1)=c14w(k)-LCw*H1*H2*Hc14*HFc14;
c15w(k+1)=c15w(k)-LCw*H1*H2*Hc15*HFc15;
c16w(k+1)=c16w(k)-LCw*H1*H2*Hc16*HFc16;
c17w(k+1)=c17w(k)-LCw*H1*H2*Hc17*HFc17;
c18w(k+1)=c18w(k)-LCw*H1*H2*Hc18*HFc18;
c19w(k+1)=c19w(k)-LCw*H1*H2*Hc19*HFc19;

c21w(k+1)=c21w(k)-LCw*H1*H2*Hc11*HFc21;
c22w(k+1)=c22w(k)-LCw*H1*H2*Hc12*HFc22;
c23w(k+1)=c23w(k)-LCw*H1*H2*Hc13*HFc23;
c24w(k+1)=c24w(k)-LCw*H1*H2*Hc14*HFc24;
c25w(k+1)=c25w(k)-LCw*H1*H2*Hc15*HFc25;
c26w(k+1)=c26w(k)-LCw*H1*H2*Hc16*HFc26;
c27w(k+1)=c27w(k)-LCw*H1*H2*Hc17*HFc27;
c28w(k+1)=c28w(k)-LCw*H1*H2*Hc18*HFc28;
c29w(k+1)=c29w(k)-LCw*H1*H2*Hc19*HFc29;
%============================
Hs11u=0.5*((du1w-dUW)/(denUw+0.00001));
Hs12u=0.5*((du2w-dUW)/(denUw+0.00001));
Hs13u=0.5*((du3w-dUW)/(denUw+0.00001));
Hs14u=0.5*((du4w-dUW)/(denUw+0.00001));
Hs15u=0.5*((du5w-dUW)/(denUw+0.00001));
Hs16u=0.5*((du6w-dUW)/(denUw+0.00001));
Hs17u=0.5*((du7w-dUW)/(denUw+0.00001));
Hs18u=0.5*((du8w-dUW)/(denUw+0.00001));
Hs19u=0.5*((du9w-dUW)/(denUw+0.00001));

Hs11l=0.5*((du1w-dLW)/(denLw+0.00001));
Hs12l=0.5*((du2w-dLW)/(denLw+0.00001));
Hs13l=0.5*((du3w-dLW)/(denLw+0.00001));
Hs14l=0.5*((du4w-dLW)/(denLw+0.00001));
Hs15l=0.5*((du5w-dLW)/(denLw+0.00001));
Hs16l=0.5*((du6w-dLW)/(denLw+0.00001));
Hs17l=0.5*((du7w-dLW)/(denLw+0.00001));
Hs18l=0.5*((du8w-dLW)/(denLw+0.00001));
Hs19l=0.5*((du9w-dLW)/(denLw+0.00001));

HFs11u=r1uw*(((e_w-c11w(k))^2)/(s11uw(k)^3));
HFs12u=r2uw*(((e_w-c12w(k))^2)/(s12uw(k)^3));
HFs13u=r3uw*(((e_w-c13w(k))^2)/(s13uw(k)^3));
HFs14u=r4uw*(((e_w-c14w(k))^2)/(s14uw(k)^3));
HFs15u=r5uw*(((e_w-c15w(k))^2)/(s15uw(k)^3));
HFs16u=r6uw*(((e_w-c16w(k))^2)/(s16uw(k)^3));
HFs17u=r7uw*(((e_w-c17w(k))^2)/(s17uw(k)^3));
HFs18u=r8uw*(((e_w-c18w(k))^2)/(s18uw(k)^3));
HFs19u=r9uw*(((e_w-c19w(k))^2)/(s19uw(k)^3));

HFs11l=r1lw*(((e_w-c11w(k))^2)/(s11lw(k)^3));
HFs12l=r2lw*(((e_w-c12w(k))^2)/(s12lw(k)^3));
HFs13l=r3lw*(((e_w-c13w(k))^2)/(s13lw(k)^3));
HFs14l=r4lw*(((e_w-c14w(k))^2)/(s14lw(k)^3));
HFs15l=r5lw*(((e_w-c15w(k))^2)/(s15lw(k)^3));
HFs16l=r6lw*(((e_w-c16w(k))^2)/(s16lw(k)^3));
HFs17l=r7lw*(((e_w-c17w(k))^2)/(s17lw(k)^3));
HFs18l=r8lw*(((e_w-c18w(k))^2)/(s18lw(k)^3));
HFs19l=r9lw*(((e_w-c19w(k))^2)/(s19lw(k)^3));

HFs21u=r1uw*(((ce_w-c21w(k))^2)/(s21uw(k)^3));
HFs22u=r2uw*(((ce_w-c22w(k))^2)/(s22uw(k)^3));
HFs23u=r3uw*(((ce_w-c23w(k))^2)/(s23uw(k)^3));
HFs24u=r4uw*(((ce_w-c24w(k))^2)/(s24uw(k)^3));
HFs25u=r5uw*(((ce_w-c25w(k))^2)/(s25uw(k)^3));
HFs26u=r6uw*(((ce_w-c26w(k))^2)/(s26uw(k)^3));
HFs27u=r7uw*(((ce_w-c27w(k))^2)/(s27uw(k)^3));
HFs28u=r8uw*(((ce_w-c28w(k))^2)/(s28uw(k)^3));
HFs29u=r9uw*(((ce_w-c29w(k))^2)/(s29uw(k)^3));

HFs21l=r1lw*(((ce_w-c21w(k))^2)/(s21lw(k)^3));
HFs22l=r2lw*(((ce_w-c22w(k))^2)/(s22lw(k)^3));
HFs23l=r3lw*(((ce_w-c23w(k))^2)/(s23lw(k)^3));
HFs24l=r4lw*(((ce_w-c24w(k))^2)/(s24lw(k)^3));
HFs25l=r5lw*(((ce_w-c25w(k))^2)/(s25lw(k)^3));
HFs26l=r6lw*(((ce_w-c26w(k))^2)/(s26lw(k)^3));
HFs27l=r7lw*(((ce_w-c27w(k))^2)/(s27lw(k)^3));
HFs28l=r8lw*(((ce_w-c28w(k))^2)/(s28lw(k)^3));
HFs29l=r9lw*(((ce_w-c29w(k))^2)/(s29lw(k)^3));


%LSw=0.0;
s11uw(k+1)=s11uw(k)-LSw*H1*H2*Hs11u*HFs11u;
s12uw(k+1)=s12uw(k)-LSw*H1*H2*Hs12u*HFs12u;
s13uw(k+1)=s13uw(k)-LSw*H1*H2*Hs13u*HFs13u;
s14uw(k+1)=s14uw(k)-LSw*H1*H2*Hs14u*HFs14u;
s15uw(k+1)=s15uw(k)-LSw*H1*H2*Hs15u*HFs15u;
s16uw(k+1)=s16uw(k)-LSw*H1*H2*Hs16u*HFs16u;
s17uw(k+1)=s17uw(k)-LSw*H1*H2*Hs17u*HFs17u;
s18uw(k+1)=s18uw(k)-LSw*H1*H2*Hs18u*HFs18u;
s19uw(k+1)=s19uw(k)-LSw*H1*H2*Hs19u*HFs19u;

s21uw(k+1)=s21uw(k)-LSw*H1*H2*Hs11u*HFs21u;
s22uw(k+1)=s22uw(k)-LSw*H1*H2*Hs12u*HFs22u;
s23uw(k+1)=s23uw(k)-LSw*H1*H2*Hs13u*HFs23u;
s24uw(k+1)=s24uw(k)-LSw*H1*H2*Hs14u*HFs24u;
s25uw(k+1)=s25uw(k)-LSw*H1*H2*Hs15u*HFs25u;
s26uw(k+1)=s26uw(k)-LSw*H1*H2*Hs16u*HFs26u;
s27uw(k+1)=s27uw(k)-LSw*H1*H2*Hs17u*HFs27u;
s28uw(k+1)=s28uw(k)-LSw*H1*H2*Hs18u*HFs28u;
s29uw(k+1)=s29uw(k)-LSw*H1*H2*Hs19u*HFs29u;

s11lw(k+1)=s11lw(k)-LSw*H1*H2*Hs11l*HFs11l;
s12lw(k+1)=s12lw(k)-LSw*H1*H2*Hs12l*HFs12l;
s13lw(k+1)=s13lw(k)-LSw*H1*H2*Hs13l*HFs13l;
s14lw(k+1)=s14lw(k)-LSw*H1*H2*Hs14l*HFs14l;
s15lw(k+1)=s15lw(k)-LSw*H1*H2*Hs15l*HFs15l;
s16lw(k+1)=s16lw(k)-LSw*H1*H2*Hs16l*HFs16l;
s17lw(k+1)=s17lw(k)-LSw*H1*H2*Hs17l*HFs17l;
s18lw(k+1)=s18lw(k)-LSw*H1*H2*Hs18l*HFs18l;
s19lw(k+1)=s19lw(k)-LSw*H1*H2*Hs19l*HFs19l;

s21lw(k+1)=s21lw(k)-LSw*H1*H2*Hs11l*HFs21l;
s22lw(k+1)=s22lw(k)-LSw*H1*H2*Hs12l*HFs22l;
s23lw(k+1)=s23lw(k)-LSw*H1*H2*Hs13l*HFs23l;
s24lw(k+1)=s24lw(k)-LSw*H1*H2*Hs14l*HFs24l;
s25lw(k+1)=s25lw(k)-LSw*H1*H2*Hs15l*HFs25l;
s26lw(k+1)=s26lw(k)-LSw*H1*H2*Hs16l*HFs26l;
s27lw(k+1)=s27lw(k)-LSw*H1*H2*Hs17l*HFs27l;
s28lw(k+1)=s28lw(k)-LSw*H1*H2*Hs18l*HFs28l;
s29lw(k+1)=s29lw(k)-LSw*H1*H2*Hs19l*HFs29l;

% update values
cewo=cew;
e_w_prev = ew;

end

%% -------- Plot results -------

outputFolder = 'figures';  % You can change this to any directory you like

% Create the folder if it doesn't exist
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end


% 1. Desired vs Actual Trajectory
figure;
plot(xr, yr, 'b--', 'LineWidth', 1.5); hold on;
plot(xm, ym, 'g-', 'LineWidth', 1.5);
xlabel('X position (m)');
ylabel('Y position (m)');
legend('Reference Trajectory','Proposed Controller');
axis equal;
grid on;
savefig(fullfile(outputFolder, 'traj_ACRL_tsk.fig'));

% 2. Desired vs Actual Linear Velocity
figure;
plot((0:N-1)*T, vd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, vm, 'g-', 'LineWidth', 2);
ylabel('Linear Velocity (m/s)');
xlabel('Time (s)');
title('Desired vs Actual Linear Velocity');
legend('vd','vm');
grid on;
savefig(fullfile(outputFolder, 'LinearVelocity_ACRL_tsk.fig'));

% 3. Desired vs Actual Angular Velocity
figure;
plot((0:N-1)*T, wd, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, wm, 'g-', 'LineWidth', 2);
ylabel('Angular Velocity (rad/s)');
xlabel('Time (s)');
title('Desired vs Actual Angular Velocity');
legend('wd','wm');
grid on;
savefig(fullfile(outputFolder, 'AngularVelocity_ACRL_tsk.fig'));
% 

% 6. X-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, xr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, xm, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('X Position (m)');
title('X-axis: Reference vs Actual');
legend('X_{ref}', 'ADRC-T1FPD');
grid on;
%savefig(fullfile(outputFolder, 'X_Position_t1.fig'));

% 7. Y-axis Position: Reference vs Actual
figure;
plot((0:N-1)*T, yr, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, ym, 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y-axis: Reference vs Actual');
legend('Y_{ref}', 'ADRC-IT2FPD');
grid on;
%savefig(fullfile(outputFolder, 'Y_Position_t1.fig'));

% 4. Desired vs Actual Theta (Orientation)
figure;
plot((0:N-1)*T, theta_r, 'b--', 'LineWidth', 2); hold on;
plot((0:N-1)*T, theta_m, 'g-', 'LineWidth', 2);
ylabel('Orientation \theta (rad)');
xlabel('Time (s)');
title('Desired vs Actual Orientation \theta');
legend('\theta_{ref}','Proposed Controller');
grid on;
%savefig(fullfile(outputFolder, 'ThetaOrientation_t1.fig'));

% 5. MAE Error Over Time
k=2:N;
t=0.1*k;
figure;
plot(t, MAEALL(k), 'g-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Absolute Position Error (m)');
legend('Proposed Controller');
grid on;
% savefig(fullfile(outputFolder, 'MAE_PositionError_ACRL_tsk.fig'));


% MAE and RMSE
% MAE and RMSE
RMSE  = sqrt(sum((EX2)+ (EY2))/N);
MAE = MAEALL(N);  % Overall MAE (optional average over dimensions)

% Display results
fprintf('--- Error Metrics ---\n');
fprintf('RMSE = %.4f m\n', RMSE);
fprintf('MAE  = %.4f m\n', MAE);