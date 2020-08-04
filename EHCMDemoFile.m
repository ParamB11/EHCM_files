% ------------------------------------------------------------------------
% This file is part of the EHCM Toolbox v1
%
% The EHCM Toolbox - Energy Hub Component Modeling for Model Predictive Control.
% Copyright (C) 2015  Automatic Control Laboratory, ETH Zurich.
%
% The EHCM Toolbox is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% The EHCM Toolbox is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with the EHCM Toolbox.  If not, see <http://www.gnu.org/licenses/>.
%
% ------------------------------------------------------------------------


%% System Configuration
% Directory change
currDir = pwd;
cd(getEHCMDir());
% Buildings and Energy Hub devices

nEHcomp = 7;
EHcomp = cell(nEHcomp,1);

% input streams (grid)
EHcomp{1} = EHConverter('inElec',{'inElec'},{'elec'},0,0,0,1);
inputPrefix = [{'in'}]; % denote the input prefix (can be the grid, hubs, PV etc...)

% conversion units (we can assign as many balancing nodes we want and different connections)
EHcomp{2} = EHConverter('HPump',{'elec'},{'heat'},0,0,0,3);
EHcomp{3} = EHConverter('AChil',{'elec'},{'cool'},0,0,0,0.7);

% output streams (heating,electricity,cooling of buildings)
EHcomp{4} = EHConverter('outElec',{'elec'},{'outElec'},0,0,0,1);
EHcomp{5} = EHConverter('outCool',{'cool'},{'outCool'},0,0,0,1);
EHcomp{6} = EHConverter('outHeat',{'heat'},{'outHeat'},0,0,0,1);
EHcomp{7} = EHBattery('Battery',5,{'elec'});
outputPrefix = [{'out'}]; % denote the output prefix (can be buildings, hubs, etc...)


% building configuration
nEHBcomp = 1;
EHBcomp = cell(nEHBcomp,1);
modelName = 'bd1';
bdfile = 'DemoBuilding';
nodeName = 'out';   % node of the EH at which the building is connected
fileName = 'EHFM';
EHBcomp{1} = EHBuilding(modelName,bdfile,nodeName,fileName);


%% Simulation parameters set up
MM = 2;
DD = 8;
HH = 1; % hours start at zero (check the weather files for conventions)
sim.init = getDate(MM,DD,HH);

sim.horizon = 48; % two days simulation
startTimeHor = 1;

Np = 12;    % prediction horizon


Qs = 100;   % slack variable penalization

elDayPrice = 0.145; % day tariff of electricity
elNightPrice = 0.097; % night tariff of electricity

%% Disturbance vector evaluation over the horizon
load('disturbSMA_2007v2');

nDist = length(disturb.identifiers);
disturbVar = sdpvar(nDist,sim.horizon,'full');

% get the disturbance vector for the buildings and any other component that
% may be affected by weather disturbances (e.g. heat pump)

distVecBd = cell(nEHBcomp,1);
for ii = 1:nEHBcomp
    % generate the correct disturbance vector
    distVecBd{ii} = getDistVec(EHBcomp{ii},disturb,disturbVar,sim,Np);
end


%% building bounds (to be adjusted according to season)
bounds = cell(nEHBcomp,1);
for ii = 1:nEHBcomp
    bounds{ii}.tlbd = 21;  % temperature lower bound day
    bounds{ii}.tubd = 25;  % temperature upper bound day
    bounds{ii}.tlbn = 10;  % temperature lower bound night
    bounds{ii}.tubn = 40;  % temperature upper bound night
end


%% initial current state of the buildings
x0EH = cell(nEHcomp,1);
for ii = 1:nEHcomp
    x0EH{ii} = EHcomp{ii}.discreteTimeModel.x0;
end

x0EHB = cell(nEHBcomp,1);
for ii = 1:nEHBcomp
    x0EHB{ii} = EHBcomp{ii}.discreteTimeModel.x0;
end

%% simulate the system in the receding horizon fashion
realEH = cell(sim.horizon,1);
realEHB = cell(sim.horizon,1);
realObj = zeros(sim.horizon,1);

for simInst = startTimeHor:sim.horizon
    
    fprintf('Iteration %d started\n',simInst);
    tstart = tic;
    %% Disturbance vector
    if simInst == startTimeHor
        distVec = cell(nEHBcomp,1);
        distVecReal = cell(nEHBcomp,1);
    end
    for ii = 1:nEHBcomp
        % generate the correct disturbance vector
        nDistbd = length(EHBcomp{ii}.discreteTimeModel.identifiers.v);
 
        distVec{ii}.vpred = squeeze(distVecBd{ii}.vpred(simInst,:,:));
        distVec{ii}.vvar = distVec{ii}.vpred;
        
        % realizations to be used for update
        distVecReal{ii} = distVecBd{ii}.vreal(simInst,:);
    end
    
    %% The energy hub optimization problem
        [EH,EHB,COM] = getHubOptProb(EHcomp,EHBcomp,inputPrefix,...
            outputPrefix,Np,distVec,x0EH,x0EHB);
    
    %% buildings objective function formulation - slack variables
    
    
    % building objective function (lower and upper soft constraint on temperature)
    for ii = 1:nEHBcomp
        
        discSS = EHBcomp{ii}.discreteTimeModel;
        xbd = EHB.xEHB{ii};
        conbd = EHB.conEHB{ii};
        objbd = EHB.objEHB{ii};
        
        tlbd = bounds{ii}.tlbd;
        tubd = bounds{ii}.tubd;
        tlbn = bounds{ii}.tlbn;
        tubn = bounds{ii}.tubn;
        
        % find the temperature indices to get the soft constraints
        xtemp = find(strncmpi('x_Z',discSS.identifiers.x,3));
        
        % slack variables for the temperatures
        nrooms = length(xtemp);
        if simInst == startTimeHor
            % create the sdp variables only at the first iteration.
            if ii == 1
                % initialize cell array.
                xiEHB = cell(nEHBcomp,1);
            end
            xibd = sdpvar(nrooms,Np,'full');
            xiEHB{ii} = xibd;
        else
            xibd = xiEHB{ii};
        end
        
        for kk = 1:Np
            
            % check the daytime to decide the bounds
            dt = sim.init + simInst + (kk-1);
            if mod(dt,24)>=5 && mod(dt,24)<=23
                tlb = tlbd;
                tub = tubd;
            else
                tlb = tlbn;
                tub = tubn;
            end
            
            % slack variables constraints
            for jj = 1:nrooms
                xt = xbd(xtemp(jj),kk+1);
                conbd = conbd + [xibd(jj,kk) >= 0];
                conbd = conbd + [xibd(jj,kk) >= Qs*xt - Qs*tub];
                conbd = conbd + [xibd(jj,kk) >= -Qs*xt + Qs*tlb];
            end
            
            % slack variables objective function
            objbd = objbd + ones(1,nrooms)*xibd(:,kk);
        end
        
        EHB.conEHB{ii} = conbd;
        EHB.objEHB{ii} = objbd;
        EHB.xiEHB{ii} = xibd;
        
    end
    
    
    %% Optimization Problem
    
    % grid control variables
    ninput = size(EH.inputs.inElec,1);
    uelec = EH.inputs.inElec;
    
    % constraint set of the problem
    contot = EH.conEH;
    for ii = 1:nEHBcomp
        contot = contot + EHB.conEHB{ii};
    end
    contot = contot + COM.conCOM;
    
    % tariff depends on daytime
    % check the daytime to decide the bounds (to be adjusted according to
    % season)
    tariff = zeros(Np,ninput);
    for kk = 1:Np
        dt = sim.init + simInst + (kk-1);
        if mod(dt,24)>=5 && mod(dt,24)<=23
            tariff(kk,:) = elDayPrice*ones(1,ninput);
        else
            tariff(kk,:) = elNightPrice*ones(1,ninput);
        end
    end
    
    % Linear objective function
    % common objective
    objEH = EH.objEH;
    for kk = 1:Np
        objEH = objEH + tariff(kk)*uelec(:,kk);
    end
    
    % local objectives
    objEHB = 0;
    for ii = 1:nEHBcomp
        objEHB = objEHB + EHB.objEHB{ii};
    end
    
    objtot = objEH + objEHB;
    
    
    %% solve the optimization problem
    
    optimize(contot,objtot)
    
    %% save the state and inputs for plots at the end

    for ii = 1:nEHcomp
        realEH{simInst}.xEH{ii} = x0EH{ii};
        realEH{simInst}.uEH{ii} = value(EH.uEH{ii}(:,1));
    end
    realEH{simInst}.inputs.inElec = realEH{simInst}.uEH{1};
    realObj(simInst) = value(objtot);
    for ii = 1:nEHBcomp
        realEHB{simInst}.xEHB{ii} = x0EHB{ii};
        realEHB{simInst}.uEHB{ii} = value(EHB.uEHB{ii}(:,1));
       
    end
    
    %% evolve the state according the optimal input
    for ii = 1:nEHcomp
        At = EHcomp{ii}.discreteTimeModel.ss.A;
        Bt = EHcomp{ii}.discreteTimeModel.ss.B;
        
        % update the state vector (e.g. batteries & heat pump)
        xn = At*value(x0EH{ii}) + Bt*realEH{simInst}.uEH{ii};
        
        x0EH{ii} = xn;
    end
    
    for ii = 1:nEHBcomp
        At = EHBcomp{ii}.discreteTimeModel.ss.A;
        But = EHBcomp{ii}.discreteTimeModel.ss.Bu;
        Bxut = EHBcomp{ii}.discreteTimeModel.ss.Bxu;
        Bvut = EHBcomp{ii}.discreteTimeModel.ss.Bvu;
        Bvt = EHBcomp{ii}.discreteTimeModel.ss.Bv;
        
        nu = size(EHBcomp{ii}.discreteTimeModel.identifiers.u,1);
        
        Bxuet = [];
        for jj = 1:nu
            Bxuet = [Bxuet,Bxut(:,:,jj)*x0EHB{ii}];
        end
        
        
        Bvuet = [];
        for jj = 1:nu
            Bvuet = [Bvuet,Bvut(:,:,jj)*distVecReal{ii}(1,:)'];
        end
        
        Buet = But + Bxuet + Bvuet;
        
        % update the state vector
        xnb = At*x0EHB{ii} + Buet*realEHB{simInst}.uEHB{ii} + Bvt*distVecReal{ii}(1,:)';
        
        x0EHB{ii} = xnb;
        
    end
    
    fprintf('Iteration %d finished in %.2f\n',simInst,toc(tstart));
    
end

%% Print results
fprintf('Simulation Successfully Ended\n');

uelectemp = zeros(1,sim.horizon);
for kk = 1:sim.horizon
    uelectemp(:,kk) = realEH{kk}.inputs.inElec(1,1);
end

xtemp = zeros(5,sim.horizon);
ubd = zeros(1,sim.horizon);
lbd = zeros(1,sim.horizon);

for kk = 1:sim.horizon
    
    xtemp(:,kk) = sum(realEHB{kk}.xEHB{1}(1:5,1))/5;
    dt = sim.init + (kk-1);
    if mod(dt,24)>=5 && mod(dt,24)<=23
        tlb = 21;
        tub = 25;
    else
        tlb = 10;
        tub = 40;
    end
    
    ubd(kk) = tub;
    lbd(kk) = tlb;
end

xbat = zeros(2,sim.horizon);
for kk = 1:sim.horizon
    xbat(:,kk) = realEH{kk}.xEH{7}(1:2,1);
end

figure;
plot(1:sim.horizon,value(xtemp))
hold on;
plot(1:sim.horizon,ubd,'r--');
plot(1:sim.horizon,lbd,'r--');
title('Room Temperature');
xlabel('Time');
ylabel('Temperature C');

figure(2)
plot(1:sim.horizon,value(uelectemp))
title('Grid energy consumption');
xlabel('Time');
ylabel('kW');

figure(3)
plot(1:sim.horizon,sum(value(xbat)),'c--')
title('Battery Charging State');
xlabel('Time');
ylabel('kW');


% go back to previous directory
cd(currDir)
