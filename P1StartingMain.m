function P1StartingMain()
    main1();
end

%==========================================================================
function main1()
    % Initial setup
    Api = xAPI4010v02();          % Creates API object
    FFF = zeros(20,1);            % Shared flags array
    FFF(1) = 1;                   % Run flag: 1 = keep running, 0 = stop
    FFF(2) = 0;                   % Pause flag: 1 = paused, 0 = running

    LastDepth = [];               % Stores latest depth image
    currGyroAtt = zeros(3,1);
    Main2();
    return;

%======================================================================
function Main2()
% Load dataset
s = 'dataSets/HH03/';
[ok, info] = Api.LoadDataSet(s);
if (ok < 0), return; end
    disp(info);
    ne = info.numEvents;
    fprintf('This dataset has %d sensors events, duration=[%.2f seconds](in sensors time).\n', ne, info.Duration);

    SamplesCounts = zeros(1,4);   % Counters for sensor events

    % Scope 1: Gyroscopes
    [ok, hhO1] = Api.IniScopesNChannels(3, 20, 500, [-80,80], 'IMU.Gyroscopes', {'Gx (°/s)', 'Gy (°/s)', 'Gz (°/s)'} );
    cO1x = 1;
    if (ok < 1), return; end

    set(hhO1(1), 'LineWidth', 3);
    set(hhO1(2), 'LineWidth', 3);
    set(hhO1(3), 'LineWidth', 3);

    k = 180/pi;   % radians to degrees

%%======================== Part A ================================
% Task 1: Additional IMU scopes

    % Accelerometer scope
    [ok2, hhO2] = Api.IniScopesNChannels( 3, 23, 500, [-2,2], 'IMU.Accelerometers', {'Ax (g)', 'Ay (g)', 'Az (g)'} );
    cO2x = 1;
    if (ok2 < 1), return; end

    set(hhO2(1), 'LineWidth', 3);
    set(hhO2(2), 'LineWidth', 3);
    set(hhO2(3), 'LineWidth', 3);

    % Magnetometer scope
    [ok3, hhO3] = Api.IniScopesNChannels( 3, 24, 500, [-1,1], 'IMU.Magnetometers', {'Mx', 'My', 'Mz'} );
    cO3x = 1;
    if (ok3 < 1), return; end

    set(hhO3(1), 'LineWidth', 3);
    set(hhO3(2), 'LineWidth', 3);
    set(hhO3(3), 'LineWidth', 3);

%%======================== Part B ================================
    % Attitude scope     3 channel, figure 27, show last 500, samples, y axis from -180 to 180 deg
    [okAtt, hhAtt] = Api.IniScopesNChannels(3, 27, 500, [-180,180], 'Estimated Attitude (RPY)', {'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'} );
    cAttx = 1; % initial write position
    if (okAtt < 1), return; end % stop scope if faled to create

    set(hhAtt(1), 'LineWidth', 2); 
    set(hhAtt(2), 'LineWidth', 2);
    set(hhAtt(3), 'LineWidth', 2);

    % Attitude estimator state
    currAtt = zeros(3,1);         % radians
    currGyroBias = zeros(3,1);    % rad/s
    tPrevSec = [];                % previous timestamp in seconds
    tb = 4;                       % bias estimation window (seconds)

    %% Image displays (RGB and depth)
        figure(21); clf();

        subplot(211);
        hiRGB = image(0);
        axis([0,320,0,240]);
        title('Last RGB frame');
        ca = gca();
        ca.XDir = 'reverse';

        subplot(212);
        hiDepth = imagesc(0);
        axis([0,320,0,240]);
        title('Last Depth frame');
        ca = gca();
        ca.XDir = 'reverse';

        %% 3D point cloud: raw camera frame
        figure(22); clf();
        hpoints = plot3(0,0,0,'b.','MarkerSize',1);
        axis equal;
        axis([0,2000,-1000,1000,-1000,1000]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('3D points (in camera CF)');
        grid on;
        rotate3d on;

        %%======================== Part A ================================
        % Task 2: filtered 3D points
        nearThresh = [0.1, 1.2];   % metres
        farThresh  = [1.2, 2.0];   % metres

        figure(26); clf();
        scaHanNear = scatter3(0,0,0,4,'b','filled'); hold on;
        scaHanFar  = scatter3(0,0,0,4,'r','filled'); hold off;
        axis equal;
        axis([0,2000,-1000,1000,-1000,1000]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('Filtered 3D points (near blue, far red)');
        grid on;
        rotate3d on;

        %%======================== Part E ================================
        % Compensated point cloud figure
        figure(28); clf();
        hpoints2 = plot3(0,0,0,'b.','MarkerSize',1);
        axis equal;
        axis([-1500,2000,-1500,1500,-1500,1500]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('3D points (compensated by attitude and altitude)');
        grid on;
        rotate3d on;

        currAltCm = 0;   % latest altitude estimate in cm
        currEAtt = zeros(3,1);   % [roll; pitch; yaw] in rad used by Part E

        %% Playback controls
        t0 = 0;
        hh = MkMenuInFigure(21);

        ec = 0;   % total events counter

        %==================================================================
        % Main event loop
        while (FFF(1))

            if (FFF(2))
                pause(0.4);
                continue;
            end

            % Read next measurement
            r = Api.getMeasurement();
            ec = ec + 1;

            t  = r.t;                    % time in 0.1 ms units
            ts = double(r.t) * 0.0001;  % time in seconds

            % Playback pacing for visibility
            if ((t - t0) > 500)
                pause(0.02);
                t0 = t;
                fprintf('Samples:imu=[%u],rgb=[%u],dpt=[%u];all:[%u];t=[%.3fs]\n', ...
                    SamplesCounts(1:3), ec, ts);
            end

            %--------------------------------------------------------------
            switch (r.ID)

                %==========================================================
                case 1   % IMU sample
                    SamplesCounts(1) = SamplesCounts(1) + 1;

                    accel = r.Data(1:3);       % g
                    gyros = r.Data(4:6) * k;   % deg/s
                    mag   = r.Data(7:9);       % magnetometer

                    % Part B: attitude prediction using gyro in rad/s
                    tSec = double(r.t) * 0.0001; % (convert to sec,api time unit is 0.1s so * 0.0001)

                    if isempty(tPrevSec) % No timestamp, first imu sample, store current time and wait for next sample
                        tPrevSec = tSec;
                    else
                        dt = tSec - tPrevSec; % (time difference between current and previous sample)
                        tPrevSec = tSec; % update current time

                        gyroRad = r.Data(4:6);  % get gyro data in rad/s
                        [currGyroBias, currAtt] = predAttit(currAtt, gyroRad, dt, tSec, tb, currGyroBias);
                        
                        currGyroAtt = currAtt;


                        rpyDeg = currAtt * (180/pi); % Convert roll, pitch, yaw from radians into degrees for plotting
                        cAttx = Api.PushDataInScopeChannels(hhAtt, cAttx, rpyDeg); % Push the new attitude sample into the attitude scope.
                    end

                    % Update IMU scopes
                    cO1x = Api.PushDataInScopeChannels(hhO1, cO1x, gyros);
                    cO2x = Api.PushDataInScopeChannels(hhO2, cO2x, accel);
                    cO3x = Api.PushDataInScopeChannels(hhO3, cO3x, mag);

                    continue;

                %==========================================================
                case 2   % Depth image
                    SamplesCounts(2) = SamplesCounts(2) + 1;
                    hiDepth.CData = r.Data;

                    [xx, yy, zz] = Api.DepthToXYZ(r.Data, 1);

                    % Part A: filtered display
                    Depth = [xx(:), yy(:), zz(:)];   % mm, camera frame
                    plot3DFiltered(nearThresh, farThresh, Depth, scaHanNear, scaHanFar);

                    % Raw point cloud display
                    hpoints.XData = xx;
                    hpoints.YData = yy;
                    hpoints.ZData = zz;

                    LastDepth = r.Data;
                    
                    % DEBUG: confirm values are reaching case 2
                    if(mod(SamplesCounts(2), 30) == 0)
                        fprintf('DEBUG case2: currAltCm=%.2f, currEAtt=[%.4f %.4f %.4f]\n', ...
                        currAltCm, currEAtt(1), currEAtt(2), currEAtt(3));
                    end


                    % Part E: compensate full point cloud using latest ROI estimates
                    ptsCam  = [xx(:), yy(:), zz(:)];
                    ptsPlat = compensateCamera(ptsCam, [0,20,0]);
                    ptsTrans = plotCompensatedCloud(ptsPlat, currEAtt, currAltCm);

                    if (mod(SamplesCounts(2), 20) == 0)
                        fprintf('PartE: currAltCm=%.2f cm | att=[%.3f %.3f %.3f] rad\n', ...
                            currAltCm, currEAtt(1), currEAtt(2), currEAtt(3));
                    end

                    hpoints2.XData = ptsTrans(:,1);
                    hpoints2.YData = ptsTrans(:,2);
                    hpoints2.ZData = ptsTrans(:,3);

                %==========================================================
                case 3   % RGB image
                    SamplesCounts(3) = SamplesCounts(3) + 1;
                    hiRGB.CData = r.Data;
                    continue;

                %==========================================================
                case 199
                    disp('here199');
                    PausePlayback();
                    continue;

                %==========================================================
                case 250   % Playback jump in time
                    fprintf('there has been a Jump in time, to tNew=[%u]\n', t);

                    t0 = t;
                    PausePlayback();
                    SamplesCounts(:) = 0;
                    ec = 0;

                    currAtt = zeros(3,1);
                    currGyroAtt = zeros(3,1);
                    currGyroBias = zeros(3,1);
                    tPrevSec = [];
                    currAltCm = 0;
                    currEAtt = zeros(3,1);

                    clear predAttit;

    continue;

                %==========================================================
                case 255   % End of dataset
                    fprintf('End of dataset has been reached.\n');
                    break;
            end
        end

        % Cleanup
        delete(hh);
        disp('BYE');
    end

    %======================================================================
    function PausePlayback()
        FFF(2) = 1;
        disp('Playback has been paused. Press button "on/off" to continue');
    end

    %======================================================================
    function MyCallbackEND(a,b)
        FFF(1) = 0;
    end

    function MyCallbackTogglePause(a,b)
        FFF(2) = 1 - FFF(2);
        fprintf('flagPaused=[%d]\n', FFF(2));
    end

    function MyCallbackJmpTo0(a,b)
        Api.JmpToTime(0);   % jump to t = 0 seconds
    end

    function MyCallbackETC1(a,b)
        % Example of user generated event in the loop
        Api.TriggerEvent(199, [1,2,3,4,5]);
        disp('user asks to generate event 199');
    end

    %======================================================================
    % Part C: ROI callback
    function MyCallbackGetROI(a,b)

        % Pause immediately
        FFF(2) = 1;
        disp('Playback has been paused for ROI processing. Press button "Play/Pause" to continue');

        % User selects ROI from RGB image
        figure(21);
        subplot(211);
        drawnow;
        roi = Api.SelectROI(21, []);
        if (roi.ok < 1)
            disp('ROI selection cancelled.');
            return;
        end

        pp = roi.pp; % Get ROI coordinates
        r1 = pp(1,2); r2 = pp(2,2); % r is row
        c1 = pp(1,1); c2 = pp(2,1);

        fprintf('Selected ROI: [columns]x[rows]=[c1:c2]x[r1:r2]=[%d:%d]x[%d:%d]\n', [c1,c2,r1,r2]);

        if isempty(LastDepth)
            disp('No depth frame available yet.');
            return;
        end

        % Extract 3D points from ROI
        [ok, xx, yy, zz] = Api.Get3DPointsFromROI([c1,c2], [r1,r2], LastDepth, 1);
        if (ok < 1 || isempty(xx))
            disp('No valid 3D points found in ROI.');
            return;
        end

        ptsCam = [xx(:), yy(:), zz(:)];   % mm, camera CF % flattens the ROI point matrices into an N x 3 list of points in the camera coordinate frame.

        % Show ROI points
        figure(25); clf();
        plot3(ptsCam(:,1), ptsCam(:,2), ptsCam(:,3), 'b.');
        axis equal;
        grid on;
        rotate3d on;
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title('3D points in last selected ROI (camera CF)');

        % Camera CF -> platform CF
        ptsPlat = compensateCamera(ptsCam, [0,20,0]);

        % Check planarity
        isPlane = checkIsPlane(ptsPlat, 20);

        if (~isPlane)
            disp('Selected region is not a plane.');
            title('Selected region is not a plane.');
            return;
        end

        % Estimate plane and attitude
        [nHat, centroid] = estimatePlaneNorm(ptsPlat);
        rpa = estimateRollPitchAltitude(nHat, centroid);   % [roll pitch altitude]

        currAltCm = rpa(3);
        currEAtt  = [rpa(1); rpa(2); 0] * pi/180;
        fprintf('DEBUG ROI: currAltCm=%.2f cm, currEAtt=[%.4f %.4f %.4f] rad\n', ...
        currAltCm, currEAtt(1), currEAtt(2), currEAtt(3));
        
        % --- Project 2: Plane Association ---
surfNames = {'no match', 'floor', 'wall1', 'door', 'wall2', 'ceiling'};
surfID = InferWhichWall(nHat, currGyroAtt, 15);   % currAtt is your gyro attitude in rad
fprintf('Plane association result: %s (ID = %d)\n', surfNames{surfID + 1}, surfID);

msg = sprintf(['Selected region is a plane with normal vector [%.4f %.4f %.4f], ' ...
               'roll = %.2f deg, pitch = %.2f deg, altitude = %.2f cm. ' ...
               'Association: %s (ID=%d)'], ...
               nHat(1), nHat(2), nHat(3), rpa(1), rpa(2), rpa(3), ...
               surfNames{surfID + 1}, surfID);

disp(msg);
title(msg);
return;
    end

    %======================================================================
    % GUI controls
    function hh = MkMenuInFigure(InThisFigure)
        figure(InThisFigure);

        currY = 1;
        hy = 20;
        px = 10;
        hyb = hy * 1.1;
        ddx = 80;

        hh(5) = CreateMyButton('ROI',        [px, currY, ddx, hy], @MyCallbackGetROI);       currY = currY + hyb;
        hh(4) = CreateMyButton('Play/Pause', [px, currY, ddx, hy], @MyCallbackTogglePause);  currY = currY + hyb;
        hh(3) = CreateMyButton('END',        [px, currY, ddx, hy], @MyCallbackEND);          currY = currY + hyb;
        hh(2) = CreateMyButton('Go to t=0',  [px, currY, ddx, hy], @MyCallbackJmpTo0);       currY = currY + hyb;
        hh(1) = CreateMyButton('ETC',        [px, currY, ddx, hy], @MyCallbackETC1);         currY = currY + hyb;

        set(hh(5), 'BackgroundColor', [1,1,0.2]);
        set(hh(4), 'BackgroundColor', [0,1,0.2]);
        set(hh(3), 'BackgroundColor', [1,0,0.2]);

        return;
    end
end

%==========================================================================
function h = CreateMyButton(strBla, position, MyCallback)
    h = uicontrol( ...
        'Style', 'pushbutton', ...
        'String', strBla, ...
        'Position', position, ...
        'Callback', MyCallback);
    return;
end

%==========================================================================
% implementation

%============================ PART A

%Plot3DFiltered
% Func for Part A Task 2


function plot3DFiltered(nearThresh, farThresh, Depth, scaHanNear, scaHanFar)
% plot3DFiltered
% Filters and visualises 3D points based on distance from the camera.
% Inputs:
%   nearThresh: [dmin dmax] in meters, default [0.1 1.2]
%   farThresh : [dmin dmax] in meters, default [1.2 2.0]
%   Depth     : N x 3 points in millimeters, camera frame
%   scaHanNear: scatter handle for near points
%   scaHanFar : scatter handle for far points

    if nargin < 1 || isempty(nearThresh)
        nearThresh = [0.1 1.2];
    end
    if nargin < 2 || isempty(farThresh)
        farThresh = [1.2 2.0];
    end
    if isempty(Depth)
        scaHanNear.XData = []; scaHanNear.YData = []; scaHanNear.ZData = [];
        scaHanFar.XData  = []; scaHanFar.YData  = []; scaHanFar.ZData  = [];
        return
    end

    X = Depth(:,1);
    Y = Depth(:,2);
    Z = Depth(:,3);

    Xm = X * 0.001;
    Ym = Y * 0.001;
    Zm = Z * 0.001;
%euclidean norm 
    d = sqrt(Xm.^2 + Ym.^2 + Zm.^2);

    valid = (d >= 0.1) & (d <= 2.0);

    nearMask = valid & (d >= nearThresh(1)) & (d <  nearThresh(2));
    farMask  = valid & (d >= farThresh(1))  & (d <= farThresh(2));

    scaHanNear.XData = X(nearMask);
    scaHanNear.YData = Y(nearMask);
    scaHanNear.ZData = Z(nearMask);

    scaHanFar.XData  = X(farMask);
    scaHanFar.YData  = Y(farMask);
    scaHanFar.ZData  = Z(farMask);
end

%====================== PART B

%preAttit



function [currGyroBias, currAtt] = predAttit(currAtt, gyro, dt, t, tb, currGyroBias)
% predAttit
% Simple attitude prediction from gyros with initial bias estimation.
%
% Inputs
%   currAtt       3x1 attitude [roll; pitch; yaw] in radians
%   gyro          3x1 angular velocity [wx; wy; wz] in rad/s
%   dt            timestep in seconds
%   t             current time in seconds
%   tb            bias estimation duration in seconds (default 4)
%   currGyroBias  3x1 gyro bias estimate in rad/s
%
% Outputs
%   currGyroBias  updated bias (running mean during first tb seconds)
%   currAtt       updated attitude in radians

    if nargin < 5 || isempty(tb)
        tb = 4;
    end

    persistent biasSum biasN printed
    if isempty(biasSum)
        biasSum = zeros(3,1);
        biasN = 0;
        printed = false;
    end

    if dt <= 0
        return
    end

    if t < tb
        biasSum = biasSum + gyro(:);
        biasN = biasN + 1;
        currGyroBias = biasSum / biasN;
        return
    end

    if ~printed
        fprintf('Gyro bias estimated at t=%.3f s: [%.6f %.6f %.6f] rad/s\n', t, currGyroBias(1), currGyroBias(2), currGyroBias(3));
        printed = true;
    end

    gyroCorr = gyro(:) - currGyroBias(:);

    % Euler integration of angular rates to angles (basic dead reckoning)
    currAtt = currAtt + gyroCorr * dt;

    % Optional wrapping for nicer plots
    currAtt = mod(currAtt + pi, 2*pi) - pi;
end


% ============================== PART C

% compensateCamera
% Func for Part C 2b

function ptsPlat=compensateCamera(ptsCF,att)
% Convert points from camera CF to platform CF using attitude offset.
% ptsCF: Nx3 points in mm, camera CF
% att : [roll pitch yaw] in degrees. Default is [0,20,0].

    if (nargin<2 || isempty(att)), att=[0,20,0]; end
    if (isempty(ptsCF)), ptsPlat=ptsCF; return ; end

    r=att(1)*pi/180;
    p=att(2)*pi/180;
    y=att(3)*pi/180;

    Rx=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
    Ry=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    Rz=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];

    R=Rz*Ry*Rx;

    ptsPlat=(R*ptsCF.').';
    return ;
end

%checkisPlane
function isPlane=checkIsPlane(ROIpts,tol)
% Check if ROI points belong to a plane within tolerance.
% ROIpts: Nx3 points in mm, platform CF
% tol  : tolerance in mm, default 20

    if (nargin<2 || isempty(tol)), tol=20; end
    if (isempty(ROIpts) || size(ROIpts,1)<3), isPlane=0; return ; end % error check 3 points or empty 

    [nHat,centroid]=estimatePlaneNorm(ROIpts); 

    dif=ROIpts-centroid.'; % difference between points 
    dist=abs(dif*nHat);   % mm, because nHat is unit and dif is mm (difference perp dist to plane)

    isPlane=all(dist<=tol);
    return ;
end


%estimatePlaneNorm

% Func for Part C
%3d points to 
function [normPlane,centroid]=estimatePlaneNorm(ROIpts)
% Estimate a best fit plane normal and centroid of points using SVD.
% ROIpts: Nx3 points in mm, platform CF
% normPlane: 3x1 unit normal
% centroid : 3x1 point on plane (mean of points)

    if (isempty(ROIpts))
        normPlane=[0;0;1];
        centroid=[0;0;0];
        return ;
    end

    % Compute centroid
    centroid=mean(ROIpts,1).';
    Q=ROIpts-centroid.';

    [~,~,V]=svd(Q,0);
    n=V(:,3);

    n=n/norm(n);

% enforce consistent direction: normal should point toward camera
% camera optical axis in platform CF (after 20 deg pitch down compensation)
camDir = [sin(20*pi/180); 0; cos(20*pi/180)];  % approx [0.34; 0; 0.94]
if dot(n, camDir) < 0
    n = -n;
end

    normPlane=n;
    return ;
end


%estimateRollpitchAltitude

% Func for Part C

function rollPitchAlt=estimateRollPitchAltitude(normPlane,centroid)
% Estimate roll, pitch (deg) and altitude (cm) from plane normal and centroid.
% normPlane: 3x1 unit normal in platform CF
% centroid : 3x1 point on plane (mm)
% output   : [roll pitch altitude] -> deg, deg, cm

    n=normPlane(:);
    n=n/norm(n);

    nx=n(1); ny=n(2); nz=n(3);

    roll = atan2(ny,nz)*180/pi;
    pitch= atan2(-nx,nz)*180/pi;

    h_mm=abs(dot(n,centroid(:)));
    h_cm=h_mm/10;

    rollPitchAlt=[roll,pitch,h_cm];
    return ;
end

% ============================== PART D

function rollPitch=estimateRollPitchCost(ptsCF,initRP)
% Estimate roll and pitch by minimizing a least-squares cost derived from floor-plane constraint.
% ptsCF  : Nx3 points on planar floor, expressed in platform CF (mm)
% initRP : [phi0 theta0] initial guess in radian (use [0,0])
% output : [roll pitch] in degree

    if (nargin<2 || isempty(initRP)), initRP=[0,0]; end
    if (isempty(ptsCF) || size(ptsCF,1)<10)
        rollPitch=[0,0];
        disp('estimateRollPitchCost: not enough points.');
        return ;
    end

    % make sure it is Nx3
    if (size(ptsCF,2)~=3)
        ptsCF=ptsCF.';
        if (size(ptsCF,2)~=3)
            rollPitch=[0,0];
            disp('estimateRollPitchCost: wrong input size.');
            return ;
        end
    end

    % use fminsearch to minimize scalar cost
    opt=optimset('Display','off','TolX',1e-6,'TolFun',1e-6,'MaxIter',200);

    x=fminsearch(@(u)CostFun(u,ptsCF), initRP(:).', opt);

    rollPitch = x*180/pi;
    return ;


    %-------------------------------------------------
    function J=CostFun(u,P)
        % u=[phi theta] in rad
        phi=u(1);
        th =u(2);

        c1=cos(phi); s1=sin(phi);
        c2=cos(th ); s2=sin(th );

        % roll about x, pitch about y
        Rx=[1 0 0; 0 c1 -s1; 0 s1 c1];
        Ry=[c2 0 s2; 0 1 0; -s2 0 c2];

        R=Ry*Rx;

        % remove translation (important). we only care about plane orientation, not altitude.
        Q=P-mean(P,1);

        % rotate demeaned points
        Qr=(R*Q.').';

        % floor constraint: after correct roll/pitch, all z should be ~0
        z=Qr(:,3);
        J=sum(z.^2);

    end

end

% ============================== PART E


%plotCompensatedCloud
function ptsTrans=plotCompensatedCloud(ptsCF,att,altitude)
% Transform point cloud using attitude (rad) and altitude (cm).
% ptsCF    : Nx3 points in platform CF (mm)
% att      : [roll pitch yaw] in rad
% altitude : scalar in cm
% ptsTrans : Nx3 transformed points (mm)

    if (isempty(ptsCF)), ptsTrans=ptsCF; return ; end
    if (nargin<2 || isempty(att)), att=[0,0,0]; end
    if (nargin<3 || isempty(altitude)), altitude=0; end

    r=att(1); p=att(2); y=att(3);

    cr=cos(r); sr=sin(r);
    cp=cos(p); sp=sin(p);
    cy=cos(y); sy=sin(y);

    Rx=[1 0 0; 0 cr -sr; 0 sr cr];
    Ry=[cp 0 sp; 0 1 0; -sp 0 cp];
    Rz=[cy -sy 0; sy cy 0; 0 0 1];

    R=Rz*Ry*Rx;

    % rotate to global (use inverse so we compensate the platform attitude)
    ptsTrans=(R.'*ptsCF.').';

    % apply vertical translation using altitude (cm -> mm)
    ptsTrans(:,3) = ptsTrans(:,3) - altitude*10;

    return ;
end


%============= proj 2

function ThisOne = InferWhichWall(nPlatform, attRad, tolDeg)
% InferWhichWall
% Associates a detected flat patch normal vector with a known surface in GCF.
%
% Inputs:
%   nPlatform : 3x1 unit normal vector of the flat patch, in platform CF
%   attRad    : [roll; pitch; yaw] current attitude estimate in radians
%   tolDeg    : angular tolerance in degrees (recommended: 15)
%
% Output:
%   ThisOne   : 0 = no match
%               1 = floor  (GCF normal [0;0;+1])
%               2 = wall1  (GCF normal [-1;0;0])
%               3 = door   (GCF normal [0;+1;0])
%               4 = wall2  (GCF normal [0;-1;0])
%               5 = ceil   (GCF normal [0;0;-1])

    if nargin < 3 || isempty(tolDeg), tolDeg = 15; end

    % Known surface normals in GCF (each row is one surface)
    knownNormals = [ 0,  0, +1;   % 1: floor
                    -1,  0,  0;   % 2: wall1
                     0, +1,  0;   % 3: door
                     0, -1,  0;   % 4: wall2
                     0,  0, -1];  % 5: ceiling

    % Build rotation matrix: platform CF -> GCF
    % Uses same Rx*Ry*Rz convention as your existing functions
    r = attRad(1); p = attRad(2); y = attRad(3);

    cr = cos(r); sr = sin(r);
    cp = cos(p); sp = sin(p);
    cy = cos(y); sy = sin(y);

    Rx = [1,  0,   0;
          0,  cr, -sr;
          0,  sr,  cr];

    Ry = [ cp, 0, sp;
            0, 1,  0;
          -sp, 0, cp];

    Rz = [cy, -sy, 0;
          sy,  cy, 0;
           0,   0, 1];

    R = Rz * Ry * Rx;   % platform -> global

    % Rotate the detected patch normal into GCF
    n = R * nPlatform(:);
    n = n / norm(n);   % ensure unit length

    tolRad = tolDeg * pi / 180;
    ThisOne = 0;   % default: no match

    for i = 1:5
        nRef = knownNormals(i, :)';
        % Angle between two unit vectors via dot product
        cosAng = dot(n, nRef);
        cosAng = max(-1, min(1, cosAng));   % clamp for numerical safety
        ang = acos(cosAng);
        if ang <= tolRad
            ThisOne = i;
            return;
        end
    end
end
% END OF FILE 
