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
    currPredAttAtLastDepth = zeros(3,1);
    currAltCm = 0;
    currEAtt = zeros(3,1);
    Main2();
    return;

%======================================================================
function Main2()
% Load dataset
s = 'dataSets/HH04/';
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
    % Attitude scope
    [okAtt, hhAtt] = Api.IniScopesNChannels(3, 27, 500, [-180,180], 'Estimated Attitude (RPY)', {'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'} );
    cAttx = 1;
    if (okAtt < 1), return; end

    set(hhAtt(1), 'LineWidth', 2);
    set(hhAtt(2), 'LineWidth', 2);
    set(hhAtt(3), 'LineWidth', 2);

    % Attitude estimator state
    currAtt = zeros(3,1);         % radians, IMU prediction state
    currGyroBias = zeros(3,1);    % rad/s
    tPrevSec = [];                % previous timestamp in seconds
    tb = 4;                       % bias estimation window (seconds)
    resetPredAttit();

    currPredAttAtLastDepth = zeros(3,1);   % predicted attitude linked to latest depth frame
    currAltCm = 0;                % latest floor altitude estimate in cm
    currEAtt  = zeros(3,1);       % [roll; pitch; yaw] from plane geometry, rad

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

                    tSec = double(r.t) * 0.0001;

                    if isempty(tPrevSec)
                        tPrevSec = tSec;
                    else
                        dt = tSec - tPrevSec;
                        tPrevSec = tSec;

                        gyroRad = r.Data(4:6);   % rad/s
                        [currGyroBias, currAtt] = predAttit(currAtt, gyroRad, dt, tSec, tb, currGyroBias);

                        rpyDeg = currAtt * (180/pi);
                        cAttx = Api.PushDataInScopeChannels(hhAtt, cAttx, rpyDeg);
                    end

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
                    currPredAttAtLastDepth = currAtt;

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
                    currGyroBias = zeros(3,1);
                    tPrevSec = [];
                    currAltCm = 0;
                    currEAtt = zeros(3,1);
                    currPredAttAtLastDepth = zeros(3,1);
                    resetPredAttit();

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

        ptsCam = [xx(:), yy(:), zz(:)];   % mm, camera CF

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
        surfID = InferWhichWall(nHat, currPredAttAtLastDepth, 15);
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

function plot3DFiltered(nearThresh, farThresh, Depth, scaHanNear, scaHanFar)

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

function resetPredAttit()
    clear predAttit
end

function ang = wrapToPiLocal(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function R = rpyRotationMatrix(att)
    r = att(1);
    p = att(2);
    y = att(3);

    Rx = [1 0 0;
          0 cos(r) -sin(r);
          0 sin(r)  cos(r)];

    Ry = [ cos(p) 0 sin(p);
              0   1   0;
          -sin(p) 0 cos(p)];

    Rz = [cos(y) -sin(y) 0;
          sin(y)  cos(y) 0;
             0       0   1];

    R = Rz * Ry * Rx;
end

function [currGyroBias, currAtt] = predAttit(currAtt, gyro, dt, t, tb, currGyroBias)

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
        fprintf('Gyro bias estimated at t=%.3f s: [%.6f %.6f %.6f] rad/s\n', ...
            t, currGyroBias(1), currGyroBias(2), currGyroBias(3));
        printed = true;
    end

    gyroCorr = gyro(:) - currGyroBias(:);

    phi = currAtt(1);
    th  = currAtt(2);

    sphi = sin(phi);
    cphi = cos(phi);
    cth  = cos(th);

    if abs(cth) < 1e-6
        cth = sign(cth + eps) * 1e-6;
    end

    phiDot = gyroCorr(1) + sphi * tan(th) * gyroCorr(2) + cphi * tan(th) * gyroCorr(3);
    thDot  = cphi * gyroCorr(2) - sphi * gyroCorr(3);
    psiDot = (sphi / cth) * gyroCorr(2) + (cphi / cth) * gyroCorr(3);

    currAtt = currAtt + dt * [phiDot; thDot; psiDot];

    currAtt(1) = wrapToPiLocal(currAtt(1));
    currAtt(2) = wrapToPiLocal(currAtt(2));
    currAtt(3) = wrapToPiLocal(currAtt(3));
end

% ============================== PART C

function ptsPlat = compensateCamera(ptsCF, att)

    if (nargin<2 || isempty(att)), att=[0,20,0]; end
    if (isempty(ptsCF)), ptsPlat=ptsCF; return ; end

    attRad = att(:).' * pi/180;
    R = rpyRotationMatrix(attRad);

    ptsPlat = (R * ptsCF.').';
    return ;
end

function isPlane = checkIsPlane(ROIpts, tol)

    if (nargin<2 || isempty(tol)), tol=20; end
    if (isempty(ROIpts) || size(ROIpts,1)<3), isPlane=0; return ; end

    ROIpts = ROIpts(all(isfinite(ROIpts),2), :);
    if (size(ROIpts,1)<3), isPlane=0; return ; end

    [nHat, centroid] = estimatePlaneNorm(ROIpts);

    dif = ROIpts - centroid.';
    dist = abs(dif * nHat);

    isPlane = all(dist <= tol);
    return ;
end

function [normPlane, centroid] = estimatePlaneNorm(ROIpts)

    if (isempty(ROIpts))
        normPlane=[0;0;1];
        centroid=[0;0;0];
        return ;
    end

    ROIpts = ROIpts(all(isfinite(ROIpts),2), :);
    if isempty(ROIpts)
        normPlane=[0;0;1];
        centroid=[0;0;0];
        return ;
    end

    centroid = mean(ROIpts,1).';
    Q = ROIpts - centroid.';

    [~,~,V] = svd(Q,0);
    n = V(:,3);

    if norm(n) < 1e-12
        normPlane = [0;0;1];
        return ;
    end

    n = n / norm(n);

    % enforce a consistent direction: prefer positive z
    if (n(3)<0), n=-n; end

    normPlane = n;
    return ;
end

function rollPitchAlt = estimateRollPitchAltitude(normPlane, centroid)

    n = normPlane(:);
    n = n / norm(n);
    if (n(3)<0), n=-n; end

    nx = n(1); ny = n(2); nz = n(3);

    roll  = atan2(ny, nz) * 180/pi;
    pitch = atan2(-nx, sqrt(ny^2 + nz^2)) * 180/pi;

    h_mm = abs(dot(n, centroid(:)));
    h_cm = h_mm/10;

    rollPitchAlt = [roll, pitch, h_cm];
    return ;
end

% ============================== PART D

function rollPitch = estimateRollPitchCost(ptsCF, initRP)

    if (nargin<2 || isempty(initRP)), initRP=[0,0]; end
    if (isempty(ptsCF) || size(ptsCF,1)<10)
        rollPitch=[0,0];
        disp('estimateRollPitchCost: not enough points.');
        return ;
    end

    if (size(ptsCF,2)~=3)
        ptsCF=ptsCF.';
        if (size(ptsCF,2)~=3)
            rollPitch=[0,0];
            disp('estimateRollPitchCost: wrong input size.');
            return ;
        end
    end

    opt=optimset('Display','off','TolX',1e-6,'TolFun',1e-6,'MaxIter',200);

    x=fminsearch(@(u)CostFun(u,ptsCF), initRP(:).', opt);

    rollPitch = x*180/pi;
    return ;

    function J = CostFun(u,P)
        phi = u(1);
        th  = u(2);

        R = rpyRotationMatrix([phi, th, 0]);

        Q = P - mean(P,1);
        Qr = (R.' * Q.').';

        z = Qr(:,3);
        J = sum(z.^2);
    end
end

% ============================== PART E

function ptsTrans = plotCompensatedCloud(ptsCF, att, altitude)

    if (isempty(ptsCF)), ptsTrans=ptsCF; return ; end
    if (nargin<2 || isempty(att)), att=[0,0,0]; end
    if (nargin<3 || isempty(altitude)), altitude=0; end

    R = rpyRotationMatrix(att(:).');

    % rotate to global (use inverse so we compensate the platform attitude)
    ptsTrans = (R.' * ptsCF.').';

    % apply vertical translation using altitude (cm -> mm)
    ptsTrans(:,3) = ptsTrans(:,3) - altitude*10;

    return ;
end

%============= proj 2

function ThisOne = InferWhichWall(nPlatform, attRad, tolDeg)

    if nargin < 3 || isempty(tolDeg), tolDeg = 15; end
    if isempty(nPlatform) || norm(nPlatform) < 1e-12
        ThisOne = 0;
        return;
    end

    knownNormals = [ 0,  0, +1;   % 1: floor
                    -1,  0,  0;   % 2: wall1
                     0, +1,  0;   % 3: door
                     0, -1,  0;   % 4: wall2
                     0,  0, -1];  % 5: ceiling

    R = rpyRotationMatrix(attRad(:).');   % platform -> global

    nPlatform = nPlatform(:) / norm(nPlatform(:));
    cand1 = R * nPlatform;
    cand2 = -cand1;

    tolRad = tolDeg * pi / 180;
    ThisOne = 0;
    bestAng = inf;
    bestID = 0;

    for c = 1:2
        if c == 1
            n = cand1;
        else
            n = cand2;
        end

        for i = 1:5
            nRef = knownNormals(i, :)';
            cosAng = dot(n, nRef);
            cosAng = max(-1, min(1, cosAng));
            ang = acos(cosAng);

            if ang < bestAng
                bestAng = ang;
                bestID = i;
            end
        end
    end

    if bestAng <= tolRad
        ThisOne = bestID;
    end
end
% END OF FILE