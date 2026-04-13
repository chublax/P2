a = [1,2];
aT = a';

% initial conditions

x0 = [0; 1] % mean of the state
P0 = [0.1 0; 0 0]; % cov of the state

% buffer
N = 10; % number of steps
x = zeros(2,N);
P = zeros(2,2,N);

[x1,P1] = stepOnce(x0,P0);
x(:,1) = x1;
P(:,:,1) = P1
for i = 2:N
    [xNext, PNext]= stepOnce(x())
    x(:,i) = xNext
    P(:,:,i) = PNext
end 

function [xNew,PNew] = stepOnce(x,P,u)
    A = [0.9 0.1; 0.1 0.95];



    xNew = A*x + [2 ; 0] + [0;0];
    
    w = [1; 0];
    varZeta = 0.4^2;
    
    PNew = A*P*A' + w*varZeta*w';


end

function [X2,P2] = MyPrediction(X1,P1,u1,dt,sv,sw)
    v = u1(1) %velocity input
    w = u2(2) % angular velocity input

    P2 = Jx * P1 * Jx'

    Pu = eye(2); %initialise as a 2x2 identity matrix
    Pu(1,1) = sv^2; % we want the variance = std. dev^2
    Pu(2,2) = sw^2; % remember to convert to radians!
    Qu = Ju*Pu*Ju'; % 3x2 * 2x2 * 2x3 
    P = P2 + Qu; % add them together and this will be your covariance matrix at k+1

    %Euler approximation
    X1(1) = X1(1) + dt*v*sin(heading);
    X1(2) = X1(2) + dt*v*cos(heading);
    X1(3) = X1(3) + dt*w;

    X2 = X1; % next step k+1

end


function Jx = 

function Ju = 


function 3x = jacobX(X,dt,u1)
heading = X(3)
v = u1(1)

