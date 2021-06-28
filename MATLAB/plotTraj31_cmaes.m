function [solns,freq] = plotTraj31_cmaes(vars,gen)
%close all
adone = 0;
FAILURE = 0;
loopmode = 1;
debugmode = 0;
shwtrajinfo = 0; % Will show AoA over time and TB angles


nano17 = 0;


% NORMAL RUN SETUP
cir = vars(1) > 0.5;
max_ang = floor(vars(2)*10)/10;
thk_ang = floor(vars(3)*10)/10;
rot_ang = floor(vars(4)*10)/10;
rot_int = floor(vars(5)*10)/10;
spdcode = floor(vars(6));
spdupv = floor(vars(7)*10)/10;
Kv = floor(vars(8)*10)/10;
hdev = floor(vars(9)*10)/10;
freq = floor(vars(10)*100)/100;


Rang = 0;
ellipsepct = 0.8;
modcir = 1;


solns = strcat(['Gen_',num2str(gen),'_C_',num2str(cir),...
    '_MaxAng_',num2str(max_ang),'_ThkAng_',num2str(thk_ang),'_RotAng_',...
    num2str(rot_ang),'_RotInt_',num2str(rot_int),'_SpdCde_',...
    num2str(spdcode),'_Spdupv_',num2str(spdupv),'_Kv_',num2str(Kv),...
    '_hdev_',num2str(hdev),'_freq_',num2str(freq)]);

disp(solns)
% %  ======== Input Parameters =========================
% spdcode = 1; % Which region to speed up
% spdupv = 2; % 1 = 100%, 2 = 200% to double speed [1 to 2]
% Kv = 0; % How much more like square wave [0 to 3]
% cir = 0; % Circle(1) or Figure 8(0) profile
% max_ang = 45; % Max Angle in Swept Angle
% thk_ang = 10; % Max Angle in Thickness Angle
% rot_ang = 90+45; % Rotation angle, 90 +/- rot_ang
% rot_int = 0; % Rotation phase lag
% =====================================================
%Rang = 0; % Profile Angle
nptsF = 80*4+1; % Should have form 4n + 1
savedata = 1; % Save Toggle
% =====================================================


if savedata == 1
    csvwrite('Trajectory.csv',vars)
end

wingL = 1; % Wing Length, Changes Nothing
rot_phase = rot_int;
if spdcode == 0
    spdup = 1;
else
    spdup = spdupv;
end

A = wingL*sind(max_ang);
B = wingL*sind(thk_ang);
% Setup Time for Speed Up
spdv = 1/(1+spdup);
time1 = linspace(0,spdv,floor(nptsF/2)+1);
time2 = linspace(spdv,1,floor(nptsF/2)+1);
time = [time1,time2(2:end)];
dif = zeros(1,length(time)-1)';

time_trans = zeros(1,length(time));
for i = 1:length(dif)
    dif(i) = time(i+1) - time(i);
end

if spdcode == 1
    sftnum = 0;
elseif spdcode == 2
    sftnum = (nptsF-1)/4;
elseif spdcode == 3
    sftnum = (nptsF-1)/2;
elseif spdcode == 4
    sftnum = 3*(nptsF-1)/4;
else
    sftnum = 0;
end

difr = circshift(dif,sftnum);
for i = 1:length(dif)
    time_trans(i+1) = time_trans(i) + difr(i);
end

if spdcode == 0
    time_trans = time;
end
%npts = 17;
npts = 10001;
%npts = 1000001;
if mod(npts,2) == 0
    npts = npts + 1;
end
Rmat = [cosd(Rang),-sin(Rang);sin(Rang),cos(Rang)];

% Create 2D Path
t_temp = linspace(0,pi,(npts-1)/2+1);
t = [t_temp,t_temp(2:end) + pi];

if cir == 1
    x = A*cos(t);
    y = B*sin(t);
    IKap = B^2/A;
    IKapm = 0.8660;
else
    x = A*cos(t);
    y = B*sin(2*t);
    IKap = 4*B^2/A;
    IKapm = 3.4641;
end

% =========================================================================
kappa = 5;
thk_frac = IKap/IKapm;
%thk_frac = thk_ang/60;
thk_fun = erf(120*thk_frac-0.8)/3.2+1/3.2 - (erf(-0.8)/3.2+1/3.2);
% =========================================================================
%thk_fun = erf(10*thk_ang/60-1.3)/2.5+1/2.5 - (erf(-1.3)/2.5+1/2.5);

if hdev > 0
    % Limit Max Amplitude of b-axis to 1/3 of Stroke Angle
    % hdev is the % of this length: i.e. hdev = 1 means use the max
    % amplitude of 1/3 of stroke angle while hdev = 0.5 means use half of
    % the max amplitude of 1/6 stroke angle
    hdevm = A/3;
    hdevu = hdev*hdevm;
    % Scale the x values to interpolate onto the new U shape
    xf = (-x+max(x))/(2*max(x));
    
    if modcir == 0
        % Use only the center ~80% of the ellipse to avoid high slope changes
        % near the corners
        t2 = linspace(pi*(1-ellipsepct)/2,pi*(1-(1-ellipsepct)/2),(npts-1)/2+1);
        % Average Y value to subtract off so the centerline is still zero
        %yac = -hdevu*((-cos((1-(1-ellipsepct)/2)*pi) + cos((1-ellipsepct)/2*pi))/(ellipsepct*pi));
        t2 = t2 + pi;
        t2 = fliplr(t2); % Flip so that the points start on the right
        t_temp2 = t2;
        
        % Initial Values of the U Shape to be interpolated onto
        xmod = x(1)/cos(t_temp2(1))*cos(t_temp2);
        ymod = hdevu*sin(t_temp2);
        ymodp = -(hdevu*cos(t_temp2))./(x(1)/cos(t_temp2(1))*sin(t_temp2)); % Slope
        ymodp2 = fliplr(ymodp);
        ymodpa = [ymodp,ymodp2(2:end-1),ymodp(1)];
        %yac = (max(ymod)+min(ymod))/2;
        yac = hdevu*(-cos(t_temp2(1)) + cos(t_temp2(end)))/abs(t_temp2(1)-t_temp2(end));
    else
        Ru = (A^2 + hdevu^2)/(2*hdevu);
        angu = 2*asin(A/Ru);
        
        t2 = linspace(pi/2 - angu/2, pi/2 + angu/2,(npts-1)/2+1);
        % Average Y value to subtract off so the centerline is still zero
        %yac = -hdevu*((-cos((1-(1-ellipsepct)/2)*pi) + cos((1-ellipsepct)/2*pi))/(ellipsepct*pi));
        t2 = t2 + pi;
        t2 = fliplr(t2); % Flip so that the points start on the right
        t_temp2 = t2;
        xmod = Ru*cos(t_temp2);
        ymod = Ru*sin(t_temp2);
        ymodp = -cos(t_temp2)./sin(t_temp2); % Slope
        ymodp2 = fliplr(ymodp);
        ymodpa = [ymodp,ymodp2(2:end-1),ymodp(1)];
        %yac = (max(ymod)+min(ymod))/2;
        yac = Ru*(-cos(t_temp2(1)) + cos(t_temp2(end)))/abs(t_temp2(1)-t_temp2(end));
    end
    
    %     figure
    %     plot(xmod,ymod,'k-','LineWidth',3)
    %     hold on
    %     for p = 2:length(ymod)-1
    %         if p ~= (length(ymod)+1)/2
    %             hline = 0.01;
    %             %plot([xmod(p)-hline*10,xmod(p)+hline*10],[ymod(p)-1/ymdr(p)*(-hline)*10,ymod(p)-1/ymdr(p)*(hline)*10],'g-')
    %             %hold on
    %             plot([xmod(p)-hline*10,xmod(p)+hline*10],[ymod(p)+ymodp(p)*(-hline)*10,ymod(p)+ymodp(p)*(hline)*10],'b-')
    %             hold on
    %         end
    %     end
    %     axis equal
    
    % Cumulatative Sum of Arc Length to Interpolate Onto since X is scaled
    ymaL = 0;
    cymaL = zeros(1,length(xmod)-1);
    for i = 2:length(ymod)
        dx2 = (xmod(i)-xmod(i-1))^2;
        dy2 = (ymod(i)-ymod(i-1))^2;
        ymaL = ymaL + sqrt(dx2 + dy2);
        cymaL(i) = cymaL(i-1) + sqrt(dx2 + dy2);
    end
    cymaLs = cymaL/cymaL(end);
    
    % Interpolate X Values onto the U Shape
    xmo = zeros(size(x));
    ymo = zeros(size(y));
    %     xmo2 = xmo;
    %     ymo2 = ymo;
    xmnM = xmo;
    ymnM = ymo;
    ymdr = xmo;
    %its = zeros(size(x));
    for i = 1:length(xf)
        %         xmLimRf = 0;
        
        it = find(xf(i)<cymaLs,1);
        eq = find(xf(i) == cymaLs,1);
        
        if isempty(eq) == 1
            xmLimR = xmod(it);
            xmLimL = xmod(it-1);
            ymLimR = ymod(it);
            ymLimL = ymod(it-1);
            s = (xf(i) - cymaLs(it-1))/(cymaLs(it) - cymaLs(it-1));
            xmn = (xmLimR - xmLimL)*s + xmLimL;
            ymn = (ymLimR - ymLimL)*s + ymLimL;
            ymdp = (ymodpa(it) - ymodpa(it-1))*s + ymodpa(it-1);
            ymdr(i) = -1/ymdp;
            xmo(i) = xmn;
            ymo(i) = ymn;
        else
            xmo(i) = xmod(eq);
            ymo(i) = ymod(eq);
        end
        
        
        m1c = ((length(ymdr) + 1)/2 + 1)/2;
        m2c = (length(ymdr) + 1)/2 + m1c - 1;
        ymdr(m1c) = Inf;
        ymdr(m2c) = Inf;
        ymodor = ymdr;
        
        if isinf(ymodor(i))
            xmnM(i) = xmo(i);
            ymnM(i) = ymo(i) + y(i);
        else
            if x(i) > 0 && y(i) > 0
                sp = -1;
            elseif x(i) > 0 && y(i) < 0
                sp = 1;
            elseif x(i) < 0 && y(i) > 0
                sp = 1;
            elseif x(i) < 0 && y(i) < 0
                sp = -1;
            else
                sp = 1;
            end
            xmnM(i) = xmo(i) + sp*sqrt(y(i)^2/(ymodor(i)^2+1));
            ymnM(i) = ymo(i) + ymodor(i)*(xmnM(i)-xmo(i));
        end
        
    end
    
    %     xmnM(1) = x(1);
    %     ymnM(1) = 0;
    %     xmnM(length((xmnM+1)/2)) = x(length((x+1)/2));
    %     ymnM(length((ymnM+1)/2)) = 0;
    %     xmnM(end) = xmnM(1);
    %     ymnM(end) = ymnM(1);
    
    if debugmode == 1
        figure
        plot(x,y,'k-','LineWidth',2)
        hold on
 
        plot(xmnM,ymnM-yac,'k--','LineWidth',2)

        axis equal
    end
    %plot(x,y,'g-')
    x = xmnM;
    y = ymnM-yac;
end



% Rotate System if Necessary
locO = [x;y];
locN = Rmat*locO;

% Convert 2D Path to 3D
spphi = zeros(1,npts);
rad = spphi;
spth = spphi;
for i = 1:npts
    spphi(i) = atan2(locN(2,i),locN(1,i));
    rad(i) = sqrt(locN(1,i)^2 + locN(2,i)^2);
    spth(i) = atan2(rad(i),wingL*cosd(max_ang));
end

xsp = sin(spth).*cos(spphi);
ysp = sin(spth).*sin(spphi);
zsp = cos(spth);

if debugmode == 1
    figure(1)
    plot3(xsp,ysp,zsp,'k-')
    axis equal
    hold all
end
% plot(xsp,ysp,'k.')
% axis equal

% Get Equally Spaced Points Based on Arc Length
aLs = checkaLS(xsp,ysp,zsp);
aLcs = buildcumsum(xsp,ysp,zsp);
aLcsm = [0,aLcs];

if spdcode > 0.5
    tvelc1 = linspace(0,1-spdv,npts); % Time for part 1 of cycle
    tvelc2 = linspace(0,spdv,npts); % Time for part 2 of cycle
    Amp1Velt = 1/(1-spdv)*aLs/2; % Based on sweep time needed distance to travel
    Amp2Velt = 1/spdv*aLs/2; % Calculated knowing integral of sin over half period
    
    %offset = kappa*0.25*max([Amp1Velt,Amp2Velt])*thk_ang/60;
    offset = kappa*0.25*min([Amp1Velt,Amp2Velt])*thk_fun;
    
    Amp1Vel = 1/(1-spdv)*(aLs/2-offset*(1-spdv)); % Based on sweep time needed distance to travel
    Amp2Vel = 1/spdv*(aLs/2-offset*spdv); % Calculated knowing integral of sin over half period
    
    if spdcode == 2 || spdcode == 4 % Change phase based on speed code
        %velphs = -pi/2;
        velphs = pi;
        nloop = 15; % More iterations required to find correct amplitude for velocity profile
    else
        velphs = 0;
        if spdcode == 0
            nloop = 1;
        else
            nloop = 8;
        end
    end
    
    for k = 1:nloop
        vel1 = Amp1Vel*sin(4*pi*tvelc1/((1-spdv)*2)+velphs-pi/2) + Amp1Vel + offset; % Initial guess of correct amplitude
        vel2 = Amp2Vel*sin(4*pi*tvelc2/(spdv*2)+velphs-pi/2) + Amp2Vel + offset; % Initial guess of correct amplitude
        tmerge = [tvelc1(1:end-1),tvelc1(end)+tvelc2(1:end)]; % Group all time together
        velmerge = [vel1(1:end-1),vel2(1:end)]; % Group all velocity together
        
        xa = tmerge;
        ya = velmerge;
        yp1 = vel1;
        yp2 = vel2;
        
        if spdcode == 1 || spdcode == 3
            SmoothPct = 0.1; % Little smoothing
        elseif spdcode == 2 || spdcode == 4
            SmoothPct = 0.4; % A LOT of smoothing required as profiles are jagged
        end
        
        xnn = zeros(size(xa));
        dif = zeros(1,length(xa)-1)';
        for i = 1:length(dif)
            dif(i) = xa(i+1) - xa(i);
        end
        
        % Remove last value so can put profiles back to back
        xa = xa(1:end-1);
        ya = ya(1:end-1);
        
        if debugmode == 1
            figure(90)
            plot(xa,ya)
            hold all
        end
        
        % Shift to proper location and Bezier curve fit join locations
        % Picks location between first and second part for Bezier fit
        % Picks location at end of second part for Bezier fit
        if spdcode == 1
            difr = circshift(dif,length(yp2)-1);
            difr(1) = difr(2);
            for i = 1:length(dif)
                xnn(i+1) = xnn(i) + difr(i);
            end
            ynn = circshift(ya',length(yp2)-1)';
            ynn = [ynn,ynn(1)];
            
            xnL = [xnn,xnn(end)+xnn(2:end)];
            ynL = [ynn,ynn(2:end)];
            
            % Second Bezier Point in Middle
            cpt21 = length(yp2);
            cpt22 = length(yp1) + length(yp2) - 1;
            
            % First Cut Point
            xtarg = (1-SmoothPct/2)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt11 = xi;
            xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt31 = xi;
            % Third Cut Point
            xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt12 = xi;
            xtarg = xnL(cpt22) + (SmoothPct/2)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt32 = xi;
        elseif spdcode == 2
            difr = circshift(dif,-(length(yp1)-1)/2);
            for i = 1:length(dif)
                xnn(i+1) = xnn(i) + difr(i);
            end
            ynn = circshift(ya',-(length(yp1)-1)/2)';
            ynn = [ynn,ynn(1)];
            cpt21 = (length(yp1)+1)/2;
            cpt22 = (length(yp1)+1)/2 + length(yp2) - 1;
            
            xnL = [xnn,xnn(end)+xnn(2:end)];
            ynL = [ynn,ynn(2:end)];
            
            % First Cut Point
            xtarg = (1-SmoothPct)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt11 = xi;
            xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt31 = xi;
            % Third Cut Point
            xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt12 = xi;
            xtarg = xnL(cpt22) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt32 = xi;
        elseif spdcode == 3
            xnn = [xa,1];
            ynn = [ya,ya(1)];
            xnL = [xnn,xnn(end)+xnn(2:end)];
            ynL = [ynn,ynn(2:end)];
            cpt21 = length(yp1);
            cpt22 = length(yp1) + length(yp2) - 1;
            
            % First Cut Point
            xtarg = (1-SmoothPct/2)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt11 = xi;
            xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt31 = xi;
            % Third Cut Point
            xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt12 = xi;
            xtarg = xnL(cpt22) + (SmoothPct/2)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt32 = xi;
        elseif spdcode == 4
            difr = circshift(dif,(length(yp2)-1)/2);
            for i = 1:length(dif)
                xnn(i+1) = xnn(i) + difr(i);
            end
            ynn = circshift(ya',(length(yp2)-1)/2)';
            ynn = [ynn,ynn(1)];
            xnL = [xnn,xnn(end)+xnn(2:end)];
            ynL = [ynn,ynn(2:end)];
            cpt21 = (length(yp2)+1)/2;
            cpt22 = (length(yp2)+1)/2 + length(yp1) - 1;
            
            % First Cut Point
            xtarg = (1-SmoothPct)*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt11 = xi;
            xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt31 = xi;
            % Third Cut Point
            xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
            [~,xi] = min(abs(xnL-xtarg));
            cpt12 = xi;
            xtarg = xnL(cpt22) + SmoothPct*xnL(cpt21);
            [~,xi] = min(abs(xnL-xtarg));
            cpt32 = xi;
        else
            xnn = [xa,1];
            ynn = [ya,ya(1)];
            xnL = [xnn,xnn(end)+xnn(2:end)];
            ynL = [ynn,ynn(2:end)];
        end
        
        if debugmode == 1
            figure(45)
            plot(xnn,ynn,'k-')
            hold on
            title('Merged Velocity Profile with Bezier Point Fix')
        end
        
        if spdcode == 1 || spdcode == 3
            xf = [xnn,1+xnn];
            yf = [ynn,ynn];
            
        elseif spdcode == 2 || spdcode == 4
            slopematchpct = 0.4; % Location of intermidiate point to match slope
            % Use 6 Pt Bezier Curve to Fit
            P01 = [xnL(cpt11), ynL(cpt11)];
            
            % Find top and bottom of discontinuity. With speed up profiles
            % the amplitude of the sine waves that have to get merged are
            % not the same and appear super jagged.
            v1s = xnL(cpt21-1);
            v2s = xnL(cpt21);
            v3s = xnL(cpt21 + 1);
            vs = [v1s,v2s,v3s];
            
            v1 = ynL(cpt21-1);
            v2 = ynL(cpt21);
            v3 = ynL(cpt21 + 1);
            vc = [v1,v2,v3];
            
            [~,ind2] = max(abs(vc));
            if ind2 == 1 || ind2 == 3
                ind1 = 2;
            elseif abs(vc(ind2+1)) < abs(vc(ind2-1))
                ind1 = 3;
            else
                ind1 = 1;
            end
            
            % Points for top and bottom of discontinuity
            if ind1 < ind2
                P21 = [vs(ind1),vc(ind1)];
                P31 = [vs(ind2),vc(ind2)];
                dif1 = abs(vc(ind1) - ynL(cpt11));
                dif2 = abs(vc(ind2) - ynL(cpt31));
            else
                P21 = [vs(ind2),vc(ind2)];
                P31 = [vs(ind1),vc(ind1)];
                dif1 = abs(vc(ind2) - ynL(cpt11));
                dif2 = abs(vc(ind1) - ynL(cpt31));
            end
            
            % Create pts between P01 - P21 and P31 - P51 so that the slope
            % is matched at P01 and P51
            slp1 = abs((ynL(cpt11+1) - ynL(cpt11-1))/((xnL(cpt11+1) - xnL(cpt11-1))));
            xm1 = slopematchpct*dif1/slp1;
            P11 = [xnL(cpt11) + xm1, ynL(cpt11) + slp1*xm1*sign(ynL(cpt11))];
            
            slp2 = abs((ynL(cpt31+1) - ynL(cpt31-1))/((xnL(cpt31+1) - xnL(cpt31-1))));
            xm2 = slopematchpct*dif2/slp2;
            P41 = [xnL(cpt31) - xm2, ynL(cpt31) + slp2*xm2*sign(ynL(cpt31))];
            
            P51 = [xnL(cpt31), ynL(cpt31)];
            PM1 = [P01;P11;P21;P31;P41;P51];
            
            % Bezier Formula
            tm = linspace(0,1);
            nBpts = length(PM1(:,1));
            ctx1 = zeros(size(tm));
            cty1 = ctx1;
            for j = 0:nBpts-1
                Bernpoly = nchoosek(nBpts-1,j)*tm.^j.*(1-tm).^(nBpts-1-j);
                ctx1 = ctx1 + Bernpoly.*PM1(j+1,1);
                cty1 = cty1 + Bernpoly.*PM1(j+1,2);
            end
            
            P02 = [xnL(cpt12), ynL(cpt12)];
            
            % Find top and bottom of discontinuity.
            v1s = xnL(cpt22-1);
            v2s = xnL(cpt22);
            v3s = xnL(cpt22 + 1);
            vs = [v1s,v2s,v3s];
            
            v1 = ynL(cpt22-1);
            v2 = ynL(cpt22);
            v3 = ynL(cpt22 + 1);
            vc = [v1,v2,v3];
            
            [~,ind2] = max(abs(vc));
            if ind2 == 1 || ind2 == 3
                ind1 = 2;
            elseif abs(vc(ind2+1)) < abs(vc(ind2-1))
                ind1 = 3;
            else
                ind1 = 1;
            end
            
            % Points of the discontinuity.
            if ind1 < ind2
                P22 = [vs(ind1),vc(ind1)];
                P32 = [vs(ind2),vc(ind2)];
                dif1 = abs(vc(ind1) - ynL(cpt12));
                dif2 = abs(vc(ind2) - ynL(cpt32));
            else
                P22 = [vs(ind2),vc(ind2)];
                P32 = [vs(ind1),vc(ind1)];
                dif1 = abs(vc(ind2) - ynL(cpt12));
                dif2 = abs(vc(ind1) - ynL(cpt32));
            end
            
            % Create pts between P02 - P22 and P32 - P52 so that the slope
            % is matched at P02 and P52
            slp1 = abs((ynL(cpt12+1) - ynL(cpt12-1))/((xnL(cpt12+1) - xnL(cpt12-1))));
            xm1 = slopematchpct*dif1/slp1;
            P12 = [xnL(cpt12) + xm1, ynL(cpt12) + slp1*xm1*sign(ynL(cpt12))];
            
            slp2 = abs((ynL(cpt32+1) - ynL(cpt32-1))/((xnL(cpt32+1) - xnL(cpt32-1))));
            xm2 = slopematchpct*dif2/slp2;
            P42 = [xnL(cpt32) - xm2, ynL(cpt32) + slp2*xm2*sign(ynL(cpt32))];
            
            P52 = [xnL(cpt32), ynL(cpt32)];
            PM2 = [P02;P12;P22;P32;P42;P52];
            
            % Bezier Formula
            tm = linspace(0,1);
            nBpts = length(PM2(:,1));
            ctx2 = zeros(size(tm));
            cty2 = ctx2;
            for j = 0:nBpts-1
                Bernpoly = nchoosek(nBpts-1,j)*tm.^j.*(1-tm).^(nBpts-1-j);
                ctx2 = ctx2 + Bernpoly.*PM2(j+1,1);
                cty2 = cty2 + Bernpoly.*PM2(j+1,2);
            end
            
            xf = [xnL(1:cpt11),ctx1(2:end-1),xnL(cpt31:cpt12),ctx2(2:end-1),xnL(cpt32:end)];
            yf = [ynL(1:cpt11),cty1(2:end-1),ynL(cpt31:cpt12),cty2(2:end-1),ynL(cpt32:end)];
        else
            xf = [xa,1+xa];
            yf = [ya,ya];
        end
        
        xff = [xf(1:end-1)-2,xf];
        yff = [yf(1:end-1),yf];
        
        if debugmode == 1
            figure(45)
            plot(xff,yff,'k-')
            hold on
        end
        
       
        xF = xff;
        yF = yff;
        
        % For plotting Bezier points
        if spdcode == 1 || spdcode == 3
            %             Pvx = [PM1(:,1)',PM2(:,1)',PM3(:,1)'];
            %             Pvy = [PM1(:,2)',PM2(:,2)',PM3(:,2)'];
            Pvx = NaN;
            Pvy = NaN;
        else
            Pvx = [PM1(:,1)',PM2(:,1)'];
            Pvy = [PM1(:,2)',PM2(:,2)'];
        end
        
        if debugmode == 1
            figure(45)
            plot(xF,yF,'b-')
            hold all
            plot(Pvx,Pvy,'ro')
            v = axis;
            axis([-1,2,v(3),v(4)])
        end
        
        [~,xi1] = min(abs(xF)); % Insert a perfect 'Zero' Start Point
        if xF(xi1) > 0 % Too big
            xlinV = 0;
            x0lin = xF(xi1-1);
            x1lin = xF(xi1);
            y0lin = yF(xi1-1);
            y1lin = yF(xi1);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            
            xfftemp = [xlinV,xF(xi1:end)];
            yfftemp = [ylinV,yF(xi1:end)];
        elseif xff(xi1) < 0 % Too small
            xlinV = 0;
            x0lin = xF(xi1);
            x1lin = xF(xi1+1);
            y0lin = yF(xi1);
            y1lin = yF(xi1+1);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            xfftemp = [xlinV,xF(xi1+1:end)];
            yfftemp = [ylinV,yF(xi1+1:end)];
        else
            xfftemp = xF(xi1:end);
            yfftemp = yF(xi1:end);
        end
        
        [~,xi2] = min(abs(xfftemp-1)); % Insert a perfect 'One' End Point
        if xfftemp(xi2) > 1 % Too big
            xlinV = 1;
            x0lin = xfftemp(xi2-1);
            x1lin = xfftemp(xi2);
            y0lin = yfftemp(xi2-1);
            y1lin = yfftemp(xi2);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [xfftemp(1:xi2-1),xlinV];
            VelVec = [yfftemp(1:xi2-1),ylinV];
        elseif xfftemp(xi2) < 1 % Too small
            xlinV = 1;
            x0lin = xfftemp(xi2);
            x1lin = xfftemp(xi2+1);
            y0lin = yfftemp(xi2);
            y1lin = yfftemp(xi2+1);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [xfftemp(1:xi2),xlinV];
            VelVec = [yfftemp(1:xi2),ylinV];
        else
            VelVecT = xfftemp(1:xi2);
            VelVec = yfftemp(1:xi2);
        end
        
        if debugmode == 1
            figure(100)
            plot([VelVecT(1:end-1)-1,VelVecT(1:end-1),VelVecT(1:end-1)+1],...
                [VelVec(1:end-1),VelVec(1:end-1),VelVec(1:end-1)])
            hold all
            title('Velocity Profile Corrections to Get Distance Traveled Correct')
        end
        
        % Find split points based on spdcode
        if spdcode == 1
            t_targ1 = 0;
            t_trag2 = spdv;
        elseif spdcode == 2
            t_targ1 = (1-spdv)/2;
            t_trag2 = t_targ1 + spdv;
        elseif spdcode == 3
            t_targ1 = 1-spdv;
            t_trag2 = 1;
        elseif spdcode == 4
            t_targ1 = 1-spdv/2;
            t_trag2 = spdv/2;
        else
            t_targ1 = 0;
            t_trag2 = 0.5;
        end
        
        [~,xi3] = min(abs(VelVecT-t_targ1)); % Insert a 'Perfect' First Split Point
        if VelVecT(xi3) > t_targ1 % Too big
            xlinV = t_targ1;
            x0lin = VelVecT(xi3-1);
            x1lin = VelVecT(xi3);
            y0lin = VelVec(xi3-1);
            y1lin = VelVec(xi3);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [VelVecT(1:xi3-1),xlinV,VelVecT(xi3:end)];
            VelVec = [VelVec(1:xi3-1),ylinV,VelVec(xi3:end)];
            %intsplt1 = xi3;
        elseif VelVecT(xi3) < t_targ1 % Too small
            xlinV = t_targ1;
            x0lin = VelVecT(xi3);
            x1lin = VelVecT(xi3+1);
            y0lin = VelVec(xi3);
            y1lin = VelVec(xi3+1);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [VelVecT(1:xi3),xlinV,VelVecT(xi3+1:end)];
            VelVec = [VelVec(1:xi3),ylinV,VelVec(xi3+1:end)];
            %intsplt1 = xi3+1;
        else
            %intsplt1 = xi3+1;
        end
        
        [~,xi4] = min(abs(VelVecT-t_trag2)); % Insert a 'Perfect' Second Split Point
        if VelVecT(xi4) > t_trag2 % Too big
            xlinV = t_trag2;
            x0lin = VelVecT(xi4-1);
            x1lin = VelVecT(xi4);
            y0lin = VelVec(xi4-1);
            y1lin = VelVec(xi4);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [VelVecT(1:xi4-1),xlinV,VelVecT(xi4:end)];
            VelVec = [VelVec(1:xi4-1),ylinV,VelVec(xi4:end)];
            %intsplt2 = xi4;
        elseif VelVecT(xi4) < t_trag2 % Too small
            xlinV = t_trag2;
            x0lin = VelVecT(xi4);
            x1lin = VelVecT(xi4+1);
            y0lin = VelVec(xi4);
            y1lin = VelVec(xi4+1);
            ylinV = (y0lin*(x1lin-xlinV) + y1lin*(xlinV-x0lin))/(x1lin-x0lin);
            VelVecT = [VelVecT(1:xi4),xlinV,VelVecT(xi4+1:end)];
            VelVec = [VelVec(1:xi4),ylinV,VelVec(xi4+1:end)];
            %intsplt2 = xi4+1;
        else
            %intsplt2 = xi4+1;
        end
        
        % Location of 'perfect' splits
        [~,xi3] = min(abs(VelVecT-t_targ1));
        [~,xi4] = min(abs(VelVecT-t_trag2));
        intsplt1 = xi3;
        intsplt2 = xi4;
        
        % Integrate to calculate distance traveled
        if spdcode == 1
            int_split1 = trapz(VelVecT(1:intsplt2),abs(VelVec(1:intsplt2))) - offset*spdv;
            int_split2 = trapz(VelVecT(intsplt2:end),abs(VelVec(intsplt2:end)))  - offset*(1-spdv);
        elseif spdcode == 2
            int_split1 = trapz(VelVecT(intsplt1:intsplt2),abs(VelVec(intsplt1:intsplt2))) - offset*spdv;
            
            int_split21 = trapz(VelVecT(intsplt2:end),abs(VelVec(intsplt2:end)));
            int_split22 = trapz(VelVecT(1:intsplt1),abs(VelVec(1:intsplt1)));
            int_split2 = int_split21 + int_split22 - offset*(1-spdv);
        elseif spdcode == 3
            int_split1 = trapz(VelVecT(intsplt1:end),abs(VelVec(intsplt1:end))) - offset*spdv;
            int_split2 = trapz(VelVecT(1:intsplt1),abs(VelVec(1:intsplt1))) - offset*(1-spdv);
        elseif spdcode == 4
            int_split11 = trapz(VelVecT(intsplt1:end),abs(VelVec(intsplt1:end)));
            int_split12 = trapz(VelVecT(1:intsplt2),abs(VelVec(1:intsplt2)));
            int_split1 = int_split11 + int_split12 - offset*spdv;
            
            int_split2 = trapz(VelVecT(intsplt2:intsplt1),abs(VelVec(intsplt2:intsplt1))) - offset*(1-spdv);
        end
        
        % Update velocity amplitude so that the distance traveled is
        % correct. Distance traveled is based on arc length of all points
        dgoal1 = aLs/2 - offset*spdv;
        %int_split1
        dgoal2 = aLs/2 - offset*(1-spdv);
        %int_split2
        
        switch spdcode
            case 1
                Amp1Vel = Amp1Vel*dgoal2/int_split2;
                Amp2Vel = Amp2Vel*dgoal1/int_split1;
            case 2
                Amp1Vel = Amp1Vel*dgoal2/int_split2;
                Amp2Vel = Amp2Vel*dgoal1/int_split1;
            case 3
                Amp1Vel = Amp1Vel*dgoal2/int_split2;
                Amp2Vel = Amp2Vel*dgoal1/int_split1;
            case 4
                Amp1Vel = Amp1Vel*dgoal2/int_split2;
                Amp2Vel = Amp2Vel*dgoal1/int_split1;
        end
    end
    
    cumVel = cumtrapz(VelVecT,abs(VelVec));
    
    
else
    
    VelVecT = linspace(0,1,2*npts+1);
    %offset = kappa*0.25*aLs*thk_ang/60;
    offset = kappa*0.25*aLs*thk_fun;
    %VelVec = aLs*pi/2*sin(2*pi*VelVecT);
    VelVec = (aLs-offset)*sin(4*pi*VelVecT-pi/2)+(aLs-offset) + offset;
    cumVel = cumtrapz(VelVecT,abs(VelVec));
    
end

if debugmode == 1
    figure
    plot(VelVecT,VelVec,'k-')
    title('Velocity Profile')
    v = axis;
    axis([0,1,0,v(4)])
end

aLseg = aLs/(nptsF-1);
%aLseg = aLs/(npts-1);
xeq = zeros(1,nptsF);
%xeq = zeros(1,npts);
yeq = xeq;
zeq = xeq;
teq = xeq;
xeq(1) = xsp(1);
yeq(1) = ysp(1);
zeq(1) = zsp(1);

tteq = linspace(0,1,nptsF);
xteq = xeq;
yteq = xeq;
zteq = xeq;

xteq(1) = xsp(1);
yteq(1) = ysp(1);
zteq(1) = zsp(1);

for i = 2:nptsF-1
    
    
    goalt = tteq(i);
    [~,tind] = min(abs(VelVecT - goalt));
    
    
   
    
    if VelVecT(tind) - goalt == 0
        dgoal = cumVel(tind);
    elseif VelVecT(tind) - goalt < 0 % Too small
        dval1 = cumVel(tind);
        dval2 = cumVel(tind+1);
        tval1 = VelVecT(tind);
        tval2 = VelVecT(tind+1);
        slp = (dval2-dval1)/(tval2-tval1);
        dgoal = dval1 + (goalt - tval1)*slp;
    elseif VelVecT(tind) - goalt > 0 % Too big
        dval1 = cumVel(tind-1);
        dval2 = cumVel(tind);
        tval1 = VelVecT(tind-1);
        tval2 = VelVecT(tind);
        slp = (dval2-dval1)/(tval2-tval1);
        dgoal = dval1 + (goalt - tval1)*slp;
    end
    
    [~,dind] = min(abs(aLcsm - dgoal));
    
    if aLcsm(dind) - dgoal == 0
        xteq(i) = xsp(dind);
        yteq(i) = ysp(dind);
        zteq(i) = zsp(dind);
    elseif aLcsm(dind) - dgoal < 0 % Too small
        xp1 = xsp(dind);
        yp1 = ysp(dind);
        zp1 = zsp(dind);
        
        xp2 = xsp(dind + 1);
        yp2 = ysp(dind + 1);
        zp2 = zsp(dind + 1);
        
        vx = xp2 - xp1;
        vy = yp2 - yp1;
        vz = zp2 - zp1;
        
        dtot = sqrt((xp1-xp2)^2 + (yp1-yp2)^2 + (zp1-zp2)^2);
        dd = abs(aLcsm(dind) - dgoal);
        xteq(i) = xp1 + dd/dtot*vx;
        yteq(i) = yp1 + dd/dtot*vy;
        zteq(i) = zp1 + dd/dtot*vz;
    else % Too big
        xp1 = xsp(dind - 1);
        yp1 = ysp(dind - 1);
        zp1 = zsp(dind - 1);
        
        xp2 = xsp(dind);
        yp2 = ysp(dind);
        zp2 = zsp(dind);
        
        vx = xp2 - xp1;
        vy = yp2 - yp1;
        vz = zp2 - zp1;
        vx = -vx;
        vy = -vy;
        vz = -vz;
        
        dtot = sqrt((xp1-xp2)^2 + (yp1-yp2)^2 + (zp1-zp2)^2);
        dd = abs(aLcsm(dind) - dgoal);
        xteq(i) = xp2 + dd/dtot*vx;
        yteq(i) = yp2 + dd/dtot*vy;
        zteq(i) = zp2 + dd/dtot*vz;
    end
end

xeq(end) = xeq(1);
yeq(end) = yeq(1);
zeq(end) = zeq(1);
teq(end) = 1;

xteq(end) = xteq(1);
yteq(end) = yteq(1);
zteq(end) = zteq(1);

xeq = xteq;
yeq = yteq;
zeq = zteq;
teq = tteq;


% Set Up Angles
if abs(rot_ang) > 0
    rot_spdcode = spdcode;
    % nv = spdupv;
    phs = rot_phase;
    % if rot_spdcode == 0
    %     n = 1;
    % else
    %     n = nv;
    % end
    m2 = spdv;
    m1 = 1-m2;
    %m = m1;
    x1 = linspace(0,m1,10001);
    x2 = linspace(0,m2,10001);
    % x1 = linspace(0,m,10001);
    % x2 = linspace(0,1-m,10001);
    SmoothPct = 0.1; % How much of total curve to smooth, currently 10%
    K1 = 0; % How close to square wave 0 = sine, increasing more square
    K2 = Kv;
    A = rot_ang; % Max + angle on 90 for first half cycle
    B = A; % Max - angle on 90 for second half cycle
    % if K1 < 0.1
    %     yp1 = A*sin(2*m*pi*x1 + phs);
    % else
    %     yp1 = A*tanh(K1*sin(2*pi*m*x1 + phs))/tanh(K1);
    % end
    if rot_spdcode == 2 || rot_spdcode == 4
        phsm = pi/2;
    else
        phsm = 0;
    end   
   
    if K2 < 0.1
        yp2 = -B*sin(pi*x2/m2 + phs + phsm);
    else
        yp2 = -B*tanh(K2*sin(pi/m2*x2 + phs + phsm))/tanh(K2);
    end    
   
    if K2 > 0
        phs2 = real(asin(atanh(-yp2(end)*tanh(K2)/B)/K2));
    else
        phs2 = asin(yp2(end)/A);
    end
    % if rot_spdcode < 0.1
    %     yp1 = A*sin(2*m*pi*x1 + phs2);
    % else
    
    if K2 == 0
        yp1 = A*sin(pi*x1/m1 + phs2);
    else
        yp1 = A*tanh(K2*sin(pi/m1*x1 + phs2))/tanh(K2);
    end
    
    chk1 = abs(yp2(end) - yp1(1)) < max([abs(yp1(2)-yp1(1)),abs(yp2(end)-yp2(end-1))]);
    slp2 = (yp2(end) - yp2(end-1));
    slp1 = (yp1(2) - yp1(1));
    chk2 = sign(slp1) == sign(slp2);
    
    if chk1 ~= 1 || chk2 ~= 1
        soln1 = phs2;
        soln2 = phs2 + pi;
        soln3 = pi - soln1;
        soln4 = 2*pi - soln1;
        solnsV = [soln1,soln2,soln3,soln4];
        bstsoln = zeros(1,4);
        
        if debugmode == 1
            figure
        end
        for n = 1:4
            if K2 == 0
                yp1 = A*sin(pi*x1/m1 + solnsV(n));
            else
                yp1 = A*tanh(K2*sin(pi/m1*x1 + solnsV(n)))/tanh(K2);
            end
            chk1 = abs(yp2(end) - yp1(1)) < max([abs(yp1(4)-yp1(1)),abs(yp2(end)-yp2(end-4))]);
            slp2 = (yp2(end) - yp2(end-1));
            slp1 = (yp1(2) - yp1(1));
            
            if abs(mod(phs2,pi/2)) < 1e-6
                chk2 = 1;
            else
                chk2 = sign(slp1) == sign(slp2);
            end
            
            bstsoln(n) =   chk1 + chk2;
            if debugmode == 1
                plot(x1,yp1)
                hold all
            end
        end
        
        if sum(bstsoln == 2) < 1
            msgbox('Error Generating Path')
            finish
        else            
            phsuV = solnsV(bstsoln == 2);
            phsu = phsuV(1);
        end
        
        if K2 == 0
            yp1 = A*sin(pi*x1/m1 + phsu);
        else
            yp1 = A*tanh(K2*sin(pi/m1*x1 + phsu))/tanh(K2);
        end
    end
    
       
    if A == 0
        yp1 = zeros(size(yp1));
        yp2 = yp1;
    end
    
    xa = [x1(1:end),m1+x2(2:end)];
    ya = [yp1(1:end),yp2(2:end)];
    
    if debugmode == 1
        figure
        plot(x1,yp1,'k-')
        hold all
        plot(x2,yp2,'r-')
        hold all
        title('Pieces of Rotation Angle Trajectory')
    end
    %plot(xa,ya,'b-')
    %hold all
    
    % plot(xa,ya)
    
    xnn = zeros(size(xa));
    dif = zeros(1,length(xa)-1)';
    for i = 1:length(dif)
        dif(i) = xa(i+1) - xa(i);
    end
    
    % Remove last value so can put profiles back to back
    xa = xa(1:end-1);
    ya = ya(1:end-1);
    
    % Shift to proper location and Bezier curve fit join locations
    if rot_spdcode == 1
        difr = circshift(dif,length(yp2)-1);
        difr(1) = difr(2);
        for i = 1:length(dif)
            xnn(i+1) = xnn(i) + difr(i);
        end
        ynn = -circshift(ya',length(yp2)-1)';
        ynn = [ynn,ynn(1)];
        
        xnL = [xnn,xnn(end)+xnn(2:end)];
        ynL = [ynn,ynn(2:end)];
        
        % Second Bezier Point in Middle
        cpt21 = length(yp2);
        cpt22 = length(yp1) + length(yp2) - 1;
        
        % First Cut Point
        xtarg = (1-SmoothPct/2)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt11 = xi;
        xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt31 = xi;
        % Third Cut Point
        xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt12 = xi;
        xtarg = xnL(cpt22) + (SmoothPct/2)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt32 = xi;
    elseif rot_spdcode == 2
        difr = circshift(dif,-(length(yp1)-1)/2);
        for i = 1:length(dif)
            xnn(i+1) = xnn(i) + difr(i);
        end
        ynn = -circshift(ya',-(length(yp1)-1)/2)';
        ynn = [ynn,ynn(1)];
        cpt21 = (length(yp1)+1)/2;
        cpt22 = (length(yp1)+1)/2 + length(yp2) - 1;
        
        xnL = [xnn,xnn(end)+xnn(2:end)];
        ynL = [ynn,ynn(2:end)];
        
        % First Cut Point
        xtarg = (1-SmoothPct)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt11 = xi;
        xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt31 = xi;
        % Third Cut Point
        xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt12 = xi;
        xtarg = xnL(cpt22) + SmoothPct*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt32 = xi;
    elseif rot_spdcode == 3
        xnn = [xa,1];
        ynn = [ya,ya(1)];
        xnL = [xnn,xnn(end)+xnn(2:end)];
        ynL = [ynn,ynn(2:end)];
        cpt21 = length(yp1);
        cpt22 = length(yp1) + length(yp2) - 1;
        
        % First Cut Point
        xtarg = (1-SmoothPct/2)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt11 = xi;
        xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt31 = xi;
        % Third Cut Point
        xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt12 = xi;
        xtarg = xnL(cpt22) + (SmoothPct/2)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt32 = xi;
    elseif rot_spdcode == 4
        difr = circshift(dif,(length(yp2)-1)/2);
        for i = 1:length(dif)
            xnn(i+1) = xnn(i) + difr(i);
        end
        ynn = circshift(ya',(length(yp2)-1)/2)';
        ynn = [ynn,ynn(1)];
        xnL = [xnn,xnn(end)+xnn(2:end)];
        ynL = [ynn,ynn(2:end)];
        cpt21 = (length(yp2)+1)/2;
        cpt22 = (length(yp2)+1)/2 + length(yp1) - 1;
        
        % First Cut Point
        xtarg = (1-SmoothPct)*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt11 = xi;
        xtarg = xnL(cpt21) + (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt31 = xi;
        % Third Cut Point
        xtarg = xnL(cpt22) - (SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xnL-xtarg));
        cpt12 = xi;
        xtarg = xnL(cpt22) + SmoothPct*xnL(cpt21);
        [~,xi] = min(abs(xnL-xtarg));
        cpt32 = xi;
    else
        xnn = [xa,1];
        ynn = [ya,ya(1)];
        xnL = [xnn,xnn(end)+xnn(2:end)];
        ynL = [ynn,ynn(2:end)];
    end
    
    if rot_spdcode > 0.5
        slopematchpct = 0.1;
        % Use Quadratic Bezier Curve to Fit
        P01 = [xnL(cpt11), ynL(cpt11)];
        P21 = [xnL(cpt21), ynL(cpt21)];
        P41 = [xnL(cpt31), ynL(cpt31)];
        
        dif1 = abs(ynL(cpt21) - ynL(cpt11));
        dif2 = abs(ynL(cpt21) - ynL(cpt31));
        
        % Create pts between P01 - P21 and P21 - P41 so that the slope
        % is matched at P01 and P41
        slp1 = abs((ynL(cpt11+1) - ynL(cpt11-1))/((xnL(cpt11+1) - xnL(cpt11-1))));
        xm1 = slopematchpct*dif1/slp1;
        P11 = [xnL(cpt11) + xm1, ynL(cpt11) + slp1*xm1*sign(ynL(cpt11+1) - ynL(cpt11-1))];
        
        slp2 = abs((ynL(cpt31+1) - ynL(cpt31-1))/((xnL(cpt31+1) - xnL(cpt31-1))));
        xm2 = slopematchpct*dif2/slp2;
        P31 = [xnL(cpt31) - xm2, ynL(cpt31) + slp2*xm2*sign(ynL(cpt31-1) - ynL(cpt31+1))];
        
        PM1 = [P01;P11;P21;P31;P41];
        
        % Bezier Formula
        tm = linspace(0,1);
        nBpts = length(PM1(:,1));
        ctx1 = zeros(size(tm));
        cty1 = ctx1;
        for j = 0:nBpts-1
            Bernpoly = nchoosek(nBpts-1,j)*tm.^j.*(1-tm).^(nBpts-1-j);
            ctx1 = ctx1 + Bernpoly.*PM1(j+1,1);
            cty1 = cty1 + Bernpoly.*PM1(j+1,2);
        end
        
        %     tm = linspace(0,1);
        %     ctx1 = (tm.^2-2*tm+1)*P01(1) + (-2*tm.^2+2*tm)*P11(1) + tm.^2*P21(1);
        %     cty1 = (tm.^2-2*tm+1)*P01(2) + (-2*tm.^2+2*tm)*P11(2) + tm.^2*P21(2);
        
        P02 = [xnL(cpt12), ynL(cpt12)];
        P22 = [xnL(cpt22), ynL(cpt22)];
        P42 = [xnL(cpt32), ynL(cpt32)];
        
        dif1 = abs(ynL(cpt22) - ynL(cpt12));
        dif2 = abs(ynL(cpt22) - ynL(cpt32));
        
        % Create pts between P02 - P22 and P22 - P41 so that the slope
        % is matched at P01 and P41
        slp1 = abs((ynL(cpt12+1) - ynL(cpt12-1))/((xnL(cpt12+1) - xnL(cpt12-1))));
        xm1 = slopematchpct*dif1/slp1;
        P12 = [xnL(cpt12) + xm1, ynL(cpt12) + slp1*xm1*sign(ynL(cpt12+1) - ynL(cpt12-1))];
        
        slp2 = abs((ynL(cpt32+1) - ynL(cpt32-1))/((xnL(cpt32+1) - xnL(cpt32-1))));
        xm2 = slopematchpct*dif2/slp2;
        P32 = [xnL(cpt32) - xm2, ynL(cpt32) + slp2*xm2*sign(ynL(cpt32-1) - ynL(cpt32+1))];
        
        PM2 = [P02;P12;P22;P32;P42];
        
        % Bezier Formula
        tm = linspace(0,1);
        nBpts = length(PM2(:,1));
        ctx2 = zeros(size(tm));
        cty2 = ctx2;
        for j = 0:nBpts-1
            Bernpoly = nchoosek(nBpts-1,j)*tm.^j.*(1-tm).^(nBpts-1-j);
            ctx2 = ctx2 + Bernpoly.*PM2(j+1,1);
            cty2 = cty2 + Bernpoly.*PM2(j+1,2);
        end
        
        %     ctx2 = (tm.^2-2*tm+1)*P02(1) + (-2*tm.^2+2*tm)*P12(1) + tm.^2*P22(1);
        %     cty2 = (tm.^2-2*tm+1)*P02(2) + (-2*tm.^2+2*tm)*P12(2) + tm.^2*P22(2);
        
        xf = [xnL(1:cpt11),ctx1(2:end-1),xnL(cpt31:cpt12),ctx2(2:end-1),xnL(cpt32:end)];
        yf = [ynL(1:cpt11),cty1(2:end-1),ynL(cpt31:cpt12),cty2(2:end-1),ynL(cpt32:end)];
    else
        xf = [xa,1+xa];
        yf = [ya,ya];
    end
    
    xff = [xf(1:end-1)-2,xf];
    yff = [yf(1:end-1),yf];
    
    % Special Points
    if rot_spdcode > 0.5
        [~,xi] = min(abs(xff));
        cpt2 = xi;
        xtarg = (SmoothPct/2)*xnL(cpt21);
        [~,xi] = min(abs(xff-xtarg));
        cpt3 = xi;
        
        xtarg = -(SmoothPct/2)*(xnL(cpt22)-xnL(cpt21));
        [~,xi] = min(abs(xff-xtarg));
        cpt1 = xi;
        
        P0 = [xff(cpt1), yff(cpt1)];
        P2 = [xff(cpt2), yff(cpt2)];
        P4 = [xff(cpt3), yff(cpt3)];
        
        dif1 = abs(ynL(cpt2) - ynL(cpt1));
        dif2 = abs(ynL(cpt2) - ynL(cpt3));
        
        % Create pts between P0 - P2 and P2 - P4 so that the slope
        % is matched at P0 and P4
        slp1 = abs((yff(cpt1+1) - yff(cpt1-1))/((xff(cpt1+1) - xff(cpt1-1))));
        xm1 = slopematchpct*dif1/slp1;
        P1 = [xff(cpt1) + xm1, yff(cpt1) + slp1*xm1*sign(yff(cpt1+1) - yff(cpt1-1))];
        
        slp2 = abs((yff(cpt3+1) - yff(cpt3-1))/((xff(cpt3+1) - xff(cpt3-1))));
        xm2 = slopematchpct*dif2/slp2;
        P3 = [xff(cpt3) - xm2, yff(cpt3) + slp2*xm2*sign(yff(cpt3-1) - yff(cpt3+1))];
        
        PM3 = [P0;P1;P2;P3;P4];
        
        % Bezier Formula
        tm = linspace(0,1);
        nBpts = length(PM3(:,1));
        ctx3 = zeros(size(tm));
        cty3 = ctx3;
        for j = 0:nBpts-1
            Bernpoly = nchoosek(nBpts-1,j)*tm.^j.*(1-tm).^(nBpts-1-j);
            ctx3 = ctx3 + Bernpoly.*PM3(j+1,1);
            cty3 = cty3 + Bernpoly.*PM3(j+1,2);
        end
        
        %     ctx3 = (tm.^2-2*tm+1)*P0(1) + (-2*tm.^2+2*tm)*P1(1) + tm.^2*P2(1);
        %     cty3 = (tm.^2-2*tm+1)*P0(2) + (-2*tm.^2+2*tm)*P1(2) + tm.^2*P2(2);
        
        xF = [xff(1:cpt1),ctx3(2:end-1),xff(cpt3:end)];
        yF = [yff(1:cpt1),cty3(2:end-1),yff(cpt3:end)];
        
        Pvx = [PM1(:,1)',PM2(:,1)',PM3(:,1)',];
        Pvy = [PM1(:,2)',PM2(:,2)',PM3(:,2)',];
        
        %     Pvx = [P01(1),P11(1),P21(1),P02(1),P12(1),P22(1),P0(1),P1(1),P2(1)];
        %     Pvy = [P01(2),P11(2),P21(2),P02(2),P12(2),P22(2),P0(2),P1(2),P2(2)];
    else
        xF = xff;
        yF = yff;
    end
    
    % figure
    % plot(xF,yF-90,'k-')
    % hold on
    
    if rot_spdcode > 0.5
        %plot(Pvx,Pvy,'k.','MarkerSize',15)
    end
    
    % Find angles closest to given times initially
    angF = zeros(1,nptsF);
    for i = 1:nptsF-1
        [~,ti] = min(abs(xF-teq(i)));
        if teq(i) - xF(ti) > 0 % Ahead of value
            angF(i) = yF(ti) + (yF(ti+1) - yF(ti))/(xF(ti+1) - xF(ti)) * (teq(i)-xF(ti));
        elseif teq(i) - xF(ti) < 0 % Behind value
            angF(i) = yF(ti-1) + (yF(ti) - yF(ti-1))/(xF(ti) - xF(ti-1)) * (teq(i)-xF(ti-1));
        else
            angF(i) = yF(ti);
        end
    end
    
else
    angF = zeros(1,nptsF);
end

angF(end) = angF(1);
angF = angF + 90;

if nano17 == 1
    angF = angF - 30 + 90; 
end
if debugmode == 1
    figure
    plot(xF,yF+90,'k-')
    hold on
    plot(teq,angF,'rx','MarkerSize',15)
    hold on
    if rot_spdcode > 0.5
        plot(Pvx,Pvy+90,'bo')
    end
    title('Rotation Trajectory with Bezier Smooth')
end

% figure
% plot3(xeq,yeq,zeq,'k.')
% hold on
% axis equal
% lwdth = 0.05/2;
% for i = 1:nptsF
%     plot3([xeq(i) + lwdth*cosd(angF(i)), xeq(i) - lwdth*cosd(angF(i))],...
%         [yeq(i) + lwdth*sind(angF(i)),yeq(i) - lwdth*sind(angF(i))],...
%         [zeq(i),zeq(i)],'k-')
%     hold on
%     plot3(xeq(i) + lwdth*cosd(angF(i)),yeq(i) + lwdth*sind(angF(i)),zeq(i),'r.')
%     hold on
%     plot3(xeq(i) - lwdth*cosd(angF(i)),yeq(i) - lwdth*sind(angF(i)),zeq(i),'g.')
%     hold on
% end

ngt = abs(angF-180) > 180;

if debugmode == 1 || shwtrajinfo == 1
    figure
    plot(angF-180)
    title('AoA Over Time')
end
zax = [xeq',yeq',zeq'];
xax = zeros(size(zax));
yax = xax;
% xaxt = xax;
% yaxt = yax;
% TB = xax;
rotmat = zeros(3,3,nptsF);
rotmat_save = zeros(3*nptsF,3);
for i = 1:nptsF
    zax(i,:) = zax(i,:)/norm(zax(i,:));
    t = cross(zax(i,:),[cosd(angF(i)),sind(angF(i)),0]);
    %     figure
    %     plot3([0,cos(alpha(i))],[0,sin(alpha(i))],[0,0])
    %     hold on
    %     plot3([0,zax(i,1)],[0,zax(i,2)],[0,zax(i,3)])
    %     hold on
    %     plot3([0,t(1)],[0,t(2)],[0,t(3)])
    %     axis equal
    xax(i,:) = cross(zax(i,:),t);
    xax(i,:) = xax(i,:)/norm(xax(i,:));
    yax(i,:) = cross(zax(i,:),xax(i,:));
    yax(i,:) = yax(i,:)/norm(yax(i,:));
    %xaxt(i,:) = yax(i,:);
    %yaxt(i,:) = cross(zax(i,:),yax(i,:));
    rotmat(:,:,i) = [xax(i,:);yax(i,:);zax(i,:)]';
    rotmat_save(3*(i-1)+1:3*i,:) = [xax(i,:);yax(i,:);zax(i,:)]';    
    %TB(i,:) = rotm2TBanglesDeg(rotmat(:,:,i));
end

if debugmode == 1
    figure
    plot3(xeq,yeq,zeq,'k.')
    hold on
    axis equal
    
    scl = 0.1;
    for i = 1:nptsF
        plot3([xeq(i), xeq(i) + scl*xax(i,1)],...
            [yeq(i),yeq(i) + scl*xax(i,2)],...
            [zeq(i),zeq(i) + + scl*xax(i,3)],'r-')
        hold on
        plot3([xeq(i), xeq(i) + scl*yax(i,1)],...
            [yeq(i),yeq(i) + scl*yax(i,2)],...
            [zeq(i),zeq(i) + + scl*yax(i,3)],'g-')
        hold on
        plot3([xeq(i), xeq(i) + scl*zax(i,1)],...
            [yeq(i),yeq(i) + scl*zax(i,2)],...
            [zeq(i),zeq(i) + scl*zax(i,3)],'b-')
        hold on
    end
    title('Trajectory 3D')
end

FAILURE = 0;
q = zeros(nptsF,4);
TB = zeros(nptsF,3);
for i = 1:nptsF
    q(i,:) = rotm2quatc(rotmat(:,:,i));
    TB(i,:) = rotm2TBanglesDeg(rotmat(:,:,i));
    if ngt(i)
        q(i,:) = -q(i,:);
        TB(i,1) = -180 - (180 - TB(i,1));
    end
    if q(i,2)^2 + q(i,3)^2> (1-cosd(50))/2
        i
        FAILURE = 1;
        disp('FAIL')
    end
end

TB = TB(1:end-1,:);
dTB = zeros(size(TB));
for j = 1:3
    for i = 1:length(TB(:,1))
        if i > 1 && i < length(TB(:,1))
            dTB(i,j) = abs(TB(i+1,j)-TB(i-1,j));
        elseif i == 1
            dTB(i,j) = abs(TB(i+1,j)-TB(end,j));
        elseif i == length(TB(:,1))
            dTB(i,j) = abs(TB(i-1,j)-TB(1,j));
        end
    end
end

sdTB = sum(dTB,2);
[~,minind] = min(sdTB);
TB = circshift(TB,-(minind-1));
TB = [TB;TB(1,:)];

nseries = 10;
fx = teq';
qF = zeros(size(q));
for j = 1:4
    fnc = q(:,j);
    F_fnc = zeros(size(fnc));
    for i = 1:nseries
        if i == 1
            a0 = 2*trapz(teq,fnc);
            F_fnc = F_fnc + a0/2;
        end
        an = 2*trapz(teq,fnc.*cos(2*pi*i.*fx));
        bn = 2*trapz(teq,fnc.*sin(2*pi*i.*fx));
        
        F_fnc = F_fnc + an.*cos(2*pi*i.*fx) + bn.*sin(2*pi*i.*fx);
    end
    qF(:,j) = F_fnc;
end

qF(end,:) = qF(1,:);

nseries = 10;
fx = teq';
TBF = zeros(size(TB));
for j = 1:3
    fnc = TB(:,j);
    F_fnc = zeros(size(fnc));
    for i = 1:nseries
        if i == 1
            a0 = 2*trapz(teq,fnc);
            F_fnc = F_fnc + a0/2;
        end
        an = 2*trapz(teq,fnc.*cos(2*pi*i.*fx));
        bn = 2*trapz(teq,fnc.*sin(2*pi*i.*fx));
        
        F_fnc = F_fnc + an.*cos(2*pi*i.*fx) + bn.*sin(2*pi*i.*fx);
    end
    TBF(:,j) = F_fnc;
end

TBF(end,:) = TBF(1,:);



if shwtrajinfo == 1
figure
subplot(1,2,1)
plot(teq,q(:,1),'k-')
hold all
plot(teq,qF(:,1),'k--')
hold all
plot(teq,q(:,2),'r-')
hold all
plot(teq,qF(:,2),'r--')
hold all
plot(teq,q(:,3),'g-')
hold all
plot(teq,qF(:,3),'g--')
hold all
plot(teq,q(:,4),'b-')
hold all
plot(teq,qF(:,4),'b--')
title('Quaternions')
v = axis;
nq = length(q(:,1));
axis([0,1,v(3),v(4)])

subplot(1,2,2)
plot(teq,TB(:,1),'k-')
hold all
plot(teq,TBF(:,1),'k--')
hold all
plot(teq,TB(:,2),'r-')
hold all
plot(teq,TBF(:,2),'r--')
hold all
plot(teq,TB(:,3),'g-')
hold all
plot(teq,TBF(:,3),'g--')
hold all
title('TB Angles')
v = axis;
axis([0,1,v(3),v(4)])
end
% q = qF;
% TB = TBF;

if FAILURE == 1
    q = zeros(nptsF,4);
    TB = zeros(nptsF,3);
end
%end

if savedata == 1
    %csvttleQ = strcat(['Trajectories\',solns,'_Q.csv']);
    csvttleTB = strcat(['C:\Users\jmsch\Documents\Caltech\Research\SURF 2021\Fin Code\Caltech-CAST-Fish-Robot\MATLAB\trajectorydata\Trajectories\',solns,'_TB.csv']);
    %csvwrite(csvttleQ,[teq',q])
    csvwrite(csvttleTB,[teq'/freq,TB])
end

if loopmode == 1        
    if shwtrajinfo == 1
    h = figure(1);
    set(h,'Position',[60,60,1920/4,1080/4])
    h = figure(2);
    set(h,'Position',[80+1920/4,60,1920/4,1080/4])
    end
    %Cecilia
    Ventana = msgbox('Trajectory Done!');
    set(Ventana,'Units','pixels','Position',[1300 300 170 70])
    waitfor(Ventana)
   % waitfor(msgbox('Trajectory Done!'))
end

return

% function sumAL = checkaL(x,y)
% sumAL = 0;
% for i = 2:length(x)
%     sumAL = sumAL + sqrt((x(i)-x(i-1))^2+(y(i)-y(i-1))^2);
% end
% return

function sumAL = checkaLS(x,y,z)
sumAL = 0;
for i = 2:length(x)
    sumAL = sumAL + sqrt((x(i)-x(i-1))^2 + ...
        (y(i)-y(i-1))^2 + (z(i)-z(i-1))^2);
end
return

function d = getdistance(xv,yv,zv)
d = sqrt((xv(2)-xv(1))^2 + (yv(2)-yv(1))^2 + (zv(2)-zv(1))^2);
return

function valV = buildcumsum(x,y,z)
distV = zeros(1,length(x)-1);
valV = distV;

for i = 1:length(x)-1
    distV(i) = sqrt((x(i)-x(i+1))^2 + (y(i)-y(i+1))^2 + (z(i)-z(i+1))^2);
    if i == 1
        valV(1) = distV(1);
    else
        valV(i) = valV(i-1) + distV(i);
    end
end
return

function quat = rotm2quatc( R )
%ROTM2QUAT Convert rotation matrix to quaternion
%   Q = ROTM2QUAT(R) converts a 3D rotation matrix, R, into the corresponding
%   unit quaternion representation, Q. The input, R, is an 3-by-3-by-N matrix
%   containing N orthonormal rotation matrices.
%   The output, Q, is an N-by-4 matrix containing N quaternions. Each
%   quaternion is of the form q = [w x y z], with a scalar number as
%   the first value. Each element of Q must be a real number.
%
%   If the input matrices are not orthonormal, the function will
%   return the quaternions that correspond to the orthonormal matrices
%   closest to the imprecise matrix inputs.
%
%
%   Example:
%      % Convert a rotation matrix to a quaternion
%      R = [0 0 1; 0 1 0; -1 0 0];
%      q = rotm2quat(R)
%
%   References:
%   [1] I.Y. Bar-Itzhack, "New method for extracting the quaternion from a
%       rotation matrix," Journal of Guidance, Control, and Dynamics,
%       vol. 23, no. 6, pp. 1085-1087, 2000
%
%   See also quat2rotm

%   Copyright 2014-2016 The MathWorks, Inc.

%#codegen

%robotics.internal.validation.validateRotationMatrix(R, 'rotm2quat', 'R');

% Pre-allocate output
quat = zeros(size(R,3), 4, 'like', R);

% Calculate all elements of symmetric K matrix
K11 = R(1,1,:) - R(2,2,:) - R(3,3,:);
K12 = R(1,2,:) + R(2,1,:);
K13 = R(1,3,:) + R(3,1,:);
K14 = R(3,2,:) - R(2,3,:);

K22 = R(2,2,:) - R(1,1,:) - R(3,3,:);
K23 = R(2,3,:) + R(3,2,:);
K24 = R(1,3,:) - R(3,1,:);

K33 = R(3,3,:) - R(1,1,:) - R(2,2,:);
K34 = R(2,1,:) - R(1,2,:);

K44 = R(1,1,:) + R(2,2,:) + R(3,3,:);

% Construct K matrix according to paper
K = [...
    K11,    K12,    K13,    K14;
    K12,    K22,    K23,    K24;
    K13,    K23,    K33,    K34;
    K14,    K24,    K34,    K44];

K = K ./ 3;

% For each input rotation matrix, calculate the corresponding eigenvalues
% and eigenvectors. The eigenvector corresponding to the largest eigenvalue
% is the unit quaternion representing the same rotation.
for i = 1:size(R,3)
    [eigVec,eigVal] = eig(K(:,:,i),'vector');
    [~,maxIdx] = max(real(eigVal));
    quat(i,:) = real([eigVec(4,maxIdx) eigVec(1,maxIdx) eigVec(2,maxIdx) eigVec(3,maxIdx)]);
    
    % By convention, always keep scalar quaternion element positive.
    % Note that this does not change the rotation that is represented
    % by the unit quaternion, since q and -q denote the same rotation.
    if quat(i,1) < 0
        quat(i,:) = -quat(i,:);
    end
end

return

function vars = decipher(bin)
v1 = bin(1); % 1 digit cir
v2 = bin(2:6); % 5 digits max_ang
v3 = bin(7:10); % 4 digits thk_ang
v4 = bin(11:12); % 2 digits rot_ang
v5 = bin(13:16); % 5 digits rotation int
v61 = bin(17); % 1 digits Toggle Speed Up
v62 = bin(18:19); % 2 digits spdcode
v7 = bin(21:23); % 3 digits spdupv
v8 = bin(24:25); % 2 digits Kv
v9 = bin(26:28); % 3 digits Rotang
v10 = bin(29:30); % 2 digits freq

v1f = v1; % cir
v2f = 17 + bin2dec(num2str(v2)); % max_ang
v3f = bin2dec(num2str(v3)); % thk_ang
v4f = bin2dec(num2str(v4)) * 45; % rot_ang
v5f = bin2dec(num2str(v5)); % rotation int
if v61 == 1
    v6f = bin2dec(num2str(v62)) + 1; % v61 = 1 then spdcode given
else
    v6f = 0; % if v61 = 0 then spdcode given as zero
end
v7f = 1 + min([bin2dec(num2str(v7))/10,0.5]); % spdupv
v8f = bin2dec(num2str(v8)); % Kv
v9f = bin2dec(num2str(v9))*7; % Rot_ang [0,49]
v10f = bin2dec(num2str(v10))/10 + 0.3; % freq [0.3,0.6]
vars = [v1f,v2f,v3f,v4f,v5f,v6f,v7f,v8f,v9f,v10f];
return

function TBangles = rotm2TBanglesDeg(rotm)
r11 = rotm(1,1);
r12 = rotm(1,2);
r13 = rotm(1,3);
r23 = rotm(2,3);
r33 = rotm(3,3);

beta = atan2(r13,sqrt(r11^2+r12^2));
alpha = atan2(-r12/cos(beta),r11/cos(beta));
gamma = atan2(-r23/cos(beta),r33/cos(beta));

TBangles = zeros(1,3);
TBangles(1) = alpha*180/pi;
TBangles(2) = beta*180/pi;
TBangles(3) = gamma*180/pi;
return

function vars = randomvals
vars = zeros(1,8);
vars(1) = (rand > 0.5);
vars(2) = floor(rand*40);
vars(3) = floor(rand*40);
vars(4) = floor(rand*50);
vars(5) = floor(rand*pi*100)/100;
vars(6) = randi(5)-1;
if vars(6) == 0
    vars(7) = 1;
else
    vars(7) = 1+floor(rand*0.5*10)/10;
end
vars(8) = floor(rand*4*10)/10;
return