classdef EHBattery < handle
    % EHBattery This class contains all the relevant properties of a battery
    % storage element. It captures the basic dynamics of a lead acid Kinetic Battery
    % Model (KiBaM). 
    % Derivation details can be found in the following paper:
    % [1] E. I. Vrettos and S. A. Papathanassiou. "Operating policy and optimal sizing 
    % of a high penetration RES-BESS system for small isolated grids." 
    %    IEEE Transactions on Energy Conversion, 26(3):744?756, 2011.
    %
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
    
    properties (SetAccess = private)
        modelName
        discreteTimeModel=[];
        batParams=[];
    end
    
    methods
        function obj = EHBattery(batteryName,nomBat,nodeIO)
            %% Set basic parameters
            Bat.x = 2 ;                                           % KiBaM charges, Q1 & Q2
            Bat.u = 2 ;                                           % discharge power Pd & charge power Pc
            
            
            %% Technical data
            
            % Inverter and battery efficiencies
            Bat.ninvch = 0.93;
            Bat.ninvdis = 0.92;
            Bat.nbattrt = 0.86;
            Bat.nbattc = sqrt(Bat.nbattrt);
            Bat.nbattd = sqrt(Bat.nbattrt);
            
            % Battery model parameters
            Bat.batterynom = nomBat;                                 % Battery nominal capacity in kWh
            Bat.batterymin1 = 0.4 * Bat.batterynom;         % SOCmin1: for battery charge/discharge control
            Bat.batterymin2 = 0.8 * Bat.batterymin1;        % SOCmin2: for battery charge/discharge control
            Bat.batterymin = 0.3 * Bat.batterynom;          % Minimum allowed SOC from manufacturer
            Bat.k = 1.24;                                         % KIBAM rate constant (k)
            Bat.c = 0.315;                                        % KIBAM capacity ratio (c)
            Bat.Qmax = Bat.batterynom;                      % KIBAM Qmax parameter is set equal to nominal capacity
            Bat.Q1max = Bat.c *Bat.batterynom;        % KIBAM Qmax parameter is set equal to nominal capacity
            Bat.Q2max = (1-Bat.c) *Bat.batterynom;    % KIBAM Qmax parameter is set equal to nominal capacity
            Bat.ac = 1;
            Bat.Pconv = 8;        % Battery converter rating [kW] (educated guess number based on SMA inverter limitation)
            Bat.loss = 1 - 6.9444e-006;                              % Internal loss
            Bat.effdis = 1; % dynamical matrices contain the charging and discharging efficiencies
            Bat.effch = 1;
            Bat.Ts = 1; % 1h sampling time
            Bat.dt = 1;
            Bat.nodeIO = nodeIO;
            
            obj.batParams = Bat;
            obj.modelName = batteryName;
            obj.getBatDynamics(Bat.Ts);
            obj.getBatConstraints;
        end
        
        function getBatDynamics(self,Ts)
            
            Bat = self.batParams;
            Bat.dt2 = Ts;
            %% Matrices for MPC with Kinetic Battery Model to calculate the SOC
            
            %% 1 hour step system
            Bat.A(1,1) =  Bat.loss *(Bat.c + (1-Bat.c) * exp(-Bat.k * Bat.dt2));
            Bat.A(1,2) =  Bat.loss *(Bat.c * (1-exp(-Bat.k * Bat.dt2)));
            Bat.A(2,1) =  Bat.loss *((1-Bat.c) * (1-exp(-Bat.k * Bat.dt2)));
            Bat.A(2,2) =  Bat.loss *((1-Bat.c) + Bat.c * exp(-Bat.k * Bat.dt2));
            %   for dischage power to grid (>0)
            Bat.B(1,1) =  ( exp(-Bat.k * Bat.dt2)*(1-Bat.c)+Bat.c*(-Bat.k*Bat.dt2+1)-1)/Bat.k  / (Bat.ninvdis * Bat.nbattd);
            Bat.B(2,1) =  ( ((Bat.c - 1) * (Bat.k * Bat.dt2 - 1 + exp(-Bat.k * Bat.dt2)))/Bat.k ) / (Bat.ninvdis * Bat.nbattd);
            %   for charge power from grid  (>0)
            Bat.B(1,2) =  -( exp(-Bat.k * Bat.dt2)*(1-Bat.c)+Bat.c*(-Bat.k*Bat.dt2+1)-1)/Bat.k  * (Bat.ninvch * Bat.nbattc);
            Bat.B(2,2) =  -( ((Bat.c - 1) * (Bat.k * Bat.dt2 - 1 + exp(-Bat.k * Bat.dt2)))/Bat.k) * (Bat.ninvch * Bat.nbattc);
            
            %% return the matrices
            % sampling time
            self.batParams.Ts = Ts;
            
            % discrete dynamics
            self.discreteTimeModel.ss.A = Bat.A;
            self.discreteTimeModel.ss.B = Bat.B;
            self.discreteTimeModel.ss.C = zeros(size(Bat.A));
            self.discreteTimeModel.ss.D = zeros(size(Bat.B));
            
            % identifiers
            mt = length(self.modelName);
            mt = min(mt,10);
            % create the identifiers
            nu = size(self.discreteTimeModel.ss.B,1); % number of inputs
            nx = size(self.discreteTimeModel.ss.A,1); % number of states
            ny = size(self.discreteTimeModel.ss.C,1);
            uid = cell(nu,1);
            xid = cell(nx,1);
            yid = cell(ny,1);
            for ii = 1:nu
                tsu = sprintf(['u_',self.modelName(1:mt),'_',num2str(ii)]);
                tsx = sprintf(['x_',self.modelName(1:mt),'_',num2str(ii)]);
                tsy = sprintf(['y_',self.modelName(1:mt),'_',num2str(ii)]);
                uid{ii} = tsu;
                xid{ii} = tsx;
                yid{ii} = tsy;
            end
            self.discreteTimeModel.identifiers.u = uid;
            self.discreteTimeModel.identifiers.x = xid;
            self.discreteTimeModel.identifiers.y = yid;
            
            % initial state
            self.discreteTimeModel.x0 = [self.batParams.batterynom/2;0];
            
            % efficiency matrix
            effMatrix = cell(2,3);
            effMatrix{1,1} = uid{1}; effMatrix{1,2} = Bat.effdis; effMatrix{1,3} = Bat.nodeIO{1};
            effMatrix{2,1} = Bat.nodeIO{1}; effMatrix{2,2} = Bat.effch; effMatrix{2,3} = uid{2};
            self.discreteTimeModel.effMatrix = effMatrix;
        end
        
        function getBatConstraints(self)
            
            % This function returns the constraints of the battery dynamics
            % in the form Fx*x + Fu*u <= g
            %%
            BatParams = self.batParams;
            BatParams.dt = self.batParams.Ts;
            
            %% efficiently create the constraints using yalmip
            xtent = sdpvar(4,1);
            xbat = xtent(1:2,1);
            ubat = xtent(3:4,1);
            
            % constraints on battery charging capacity
            con = [ubat>=0];
            con = con + [xbat>=0];
            con = con + [0.2*BatParams.batterynom <= xbat(1)+xbat(2) <= BatParams.batterynom];
            
            % Maximum discharge power (based on equation 8 of Vrettos and Papathanassiou)
            Pmax = BatParams.ninvdis * BatParams.nbattd * (BatParams.k * xbat(1) * exp(-BatParams.k * BatParams.dt) + ...
                (xbat(1)+xbat(2)) * BatParams.k * BatParams.c * (1-exp(-BatParams.k * BatParams.dt))) / (1-exp(-BatParams.k * BatParams.dt) + ...
                BatParams.c * (BatParams.k * BatParams.dt -1 + exp(-BatParams.k * BatParams.dt)));
            
            
            con = con + [ubat(1) <= BatParams.Pconv];
            con = con + [ubat(1) <= Pmax];
            
            % Maximum Charge power (based on equation 9 of Vrettos and Papathanassiou)
            Pmax1 =(((-BatParams.k) * BatParams.c * BatParams.Qmax + ...
                BatParams.k * xbat(1) * exp(-BatParams.k * BatParams.dt) + ...
                (xbat(1)+xbat(2)) * BatParams.k * BatParams.c * (1-exp(-BatParams.k * BatParams.dt))) /...
                (1-exp(-BatParams.k * BatParams.dt) + BatParams.c * (BatParams.k * BatParams.dt - 1 + exp (-BatParams.k * BatParams.dt)))) / (BatParams.ninvch * BatParams.nbattc);
            
            Pmax2 = ((1-exp(-BatParams.ac * BatParams.dt)) * (1.2*BatParams.Qmax-(xbat(1)+xbat(2)))/BatParams.dt) / (BatParams.ninvch * BatParams.nbattc);
            
            
            con = con + [ubat(2) <= BatParams.Pconv];
            con = con + [ubat(2) <= -Pmax1];
            con = con + [ubat(2) <= Pmax2];
            
            dI = getModelMatrices(con,0);
            
            % return values
            self.discreteTimeModel.constraints.Fx = full(dI.A(:,1:2));
            self.discreteTimeModel.constraints.Fu = full(dI.A(:,3:4));
            self.discreteTimeModel.constraints.g = full(dI.b);
        end
    end
    
end

