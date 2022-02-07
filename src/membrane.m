function square_oscillation()
global c n K rho L A Q_x Q_y
t0=0;
tf=2; 
n=3; %numebr of mode shapes
K=1e+2; % N/m % Tension in the surafce
rho= 1; % kg/m
L= 1;   % m
A = [0 0 0]';
phi_0 = pi/2*ones(2*n,1);
wave_speed = sqrt(K/rho);
for i=1:n
    lambda(:,i) = 2*L/i;
    omega(:,i)= wave_speed/lambda(i);
end

initial_conditions = 0*ones(2*2*n,1);

c= zeros(n,1);
Q_x= zeros(n,1);
Q_y= zeros(n,1);
options = odeset('RelTol',1e-12,'AbsTol',1e-12*ones(2*2*n,1));
[T,Z] = ode45(@dynamics,[t0 tf],initial_conditions,options);
%=============================== Plotting ================================%
%figure(1)
%plot(T,Z(:,1:3));

%figure(2)
%plot(T,Z(:,4:6));
[x,y] = meshgrid(0:0.01:1,0:0.01:1);
%x=(0:0.01:L)';
%y=(0:0.01:L)';
dt = (tf-t0)/100;
t= (t0:dt:tf)';
z = interp1(T,Z,t);

%figure(3)

for j=1:length(t)
    zz =zeros (length(x),length(y));
    %element = zeros (length(x),1);
    etha_temp = zeros (length(x),length(y));
    for i=1:n
        tau_x(j,i) = z(j,i);
        tau_y(j,i) = z(j,i+6);
        phi_x(:,i) = sin((i*pi/L)*x(1,:));
        phi_y(:,i) = sin((i*pi/L)*y(:,1));
        etha_matrix =tau_x(j,i)*(phi_x(:,i)*(phi_y(:,i))');
        
        etha_temp = etha_temp + etha_matrix;
    end
   % for i=1:n
    %    element(:) = etha(:,i)+ element(:);
   % end
    for i=1:length(x)
        for k=1:length(y)
            zz(i,k)= zz(i,k)+etha_temp(i,k);
            
        end
    end

    
    figure(4)
    %xlim([0 L]);
    s_1= surf(x,y, zz);
    shading interp
    view([1 1 1])
    %ylim([-2 2]);
    zlim ([-0.05 0.05]);
    pause(10*dt)
               
         
            
end
   
%===============================          ================================%
end
function dz = dynamics(t,z)
global c n K rho L Q_x Q_y
force= impact_force_model(t);
F = force(1);
x_i = force(2);
y_i = force(3);
for i=1:n
    c(i,1)= -(K/rho)*(i*pi/L)^2;
    Q_x(i,1)= F*sin(i*pi*x_i/L) ;
    Q_y(i,1)= F*sin(i*pi*y_i/L) ;
end
dz(1,1) = z(4);
dz(2,1) = z(5);
dz(3,1) = z(6);
dz(4,1) = c(1)*z(1)+Q_x(1);
dz(5,1) = c(2)*z(2)+Q_x(2);
dz(6,1) = c(3)*z(3)+Q_x(3);

dz(7,1) = z(10);
dz(8,1) = z(11);
dz(9,1) = z(12);
dz(10,1) = c(1)*z(7)+Q_y(1);
dz(11,1) = c(2)*z(8)+Q_y(2);
dz(12,1) = c(3)*z(9)+Q_y(3);



end
function force = impact_force_model(t)
global  L 
if t>=0.1 && t<=0.2
    F=10;
else
    F=0;
end
X_i = L/2;
Y_i = L/2;
force =[F,X_i,Y_i];
end