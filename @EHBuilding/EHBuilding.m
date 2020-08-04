classdef EHBuilding < handle
    %----------------------------------------------------------------------
    % EHBattery This class implements a interface to the BRCM toolbox that.
    % is used to derive the resistive-capacitive model of any building. For
    % more details, please refer to www.brcm.ethz.ch
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
    properties
        rootPath
        modelPath
        modelName
        fileName
        nodeName
        constraintPath
        costPath
        BRCMmodel = [];
        discreteTimeModel = [];
        Ts_hrs;
    end
    
    methods
        function obj = EHBuilding(modelName,pathName,nodeName,fileName,Ts)
            if nargin < 2
                error('Building file not provided');
            elseif nargin == 2
                fileName = 'S1';
                confile = [fileName,filesep,'Constraints3'];
                costfile = [fileName,filesep,'Costs3'];
                nodeName = 'out';   % node of the EH at which the building is connected
                Ts = 1;   % 1h discretization
            elseif nargin == 3
                fileName = 'S1';
                confile = [fileName,filesep,'Constraints3'];
                costfile = [fileName,filesep,'Costs3'];
                Ts = 1;   % 1h discretization
            elseif nargin == 4
                confile = [fileName,filesep,'Constraints3'];
                costfile = [fileName,filesep,'Costs3'];
                Ts = 1;   % 1h discretization
            elseif nargin < 5
                error('Not all inputs were provided to construct the building');
            end
            
            % Important note: To get the constraints, you should provide
            % the current state and disturbance vector of the model (so as
            % to discretize them internally).
            obj.Ts_hrs = Ts;
            obj.modelPath = [pwd,filesep,pathName];
            obj.constraintPath = [pwd, filesep, pathName, filesep, confile];
            obj.costPath = [pwd, filesep, pathName, filesep, costfile];
            obj.fileName = fileName;
            obj.modelName = modelName;
            obj.rootPath = pwd;
            obj.nodeName = nodeName;
            obj.generateModel;
            obj.discretize(Ts);
            obj.efficiencyMatrix;
            obj.constraints;
            fprintf('The dynamics-constraints of bulding: %s were generated\n',obj.modelName);
        end
        
        function generateModel(self)
            %--------------------------------------------------------------
            % genModel(modelParms)
            %
            % Returns the continuous model of the specified building
            %--------------------------------------------------------------
            
            buildingIdentifier  = self.modelName;
            EHFModelDataDir     = [self.modelPath,filesep,self.fileName];
            thermalModelDataDir = [self.modelPath,filesep,'ThermalModel'];
            
            %% Generate model
            
            % Clear model
            self.BRCMmodel = [];
            
            % 1) Create building
            self.BRCMmodel = Building(buildingIdentifier);
            
            % 2) Load the thermal model
            self.BRCMmodel.loadThermalModelData(thermalModelDataDir);
            
            % 3) Declare external heat flux models that should be included
            
            % Heat exchange with ambient air and solar gains
            EHFModelClassFile = 'BuildingHull.m';                                         % This is the m-file defining this EHF model's class.
            EHFModelDataFile = [EHFModelDataDir,filesep,'buildinghull'];                  % This is the spreadsheet containing this EHF model's specification.
            EHFModelIdentifier = 'BuildingHull';                                          % This string identifies the EHF model uniquely
            self.BRCMmodel.declareEHFModel(EHFModelClassFile,EHFModelDataFile,EHFModelIdentifier);
            
            % Ventilation
            EHFModelClassFile = 'AHU.m';
            EHFModelDataFile = [EHFModelDataDir,filesep,'ahu'];
            [~, ~, ahu_raw] = xlsread(EHFModelDataFile);
            [r_heat,c_heat] = find(strcmp('hasHeater', ahu_raw));
            [r_cool,c_cool] = find(strcmp('hasCooler', ahu_raw));
            if (ahu_raw{r_heat, c_heat+1} == 0) && (ahu_raw{r_cool, c_cool+1} == 0)
            else
                EHFModelIdentifier = 'AHU';
                self.BRCMmodel.declareEHFModel(EHFModelClassFile,EHFModelDataFile,EHFModelIdentifier);
            end
            
            % InternalGains
            EHFModelClassFile = 'InternalGains.m';
            EHFModelDataFile = [EHFModelDataDir,filesep,'internalgains'];
            EHFModelIdentifier = 'IG';
            self.BRCMmodel.declareEHFModel(EHFModelClassFile,EHFModelDataFile,EHFModelIdentifier);
            
            % TABS
            EHFModelClassFile = 'BEHeatfluxes.m';
            EHFModelDataFile = [EHFModelDataDir,filesep,'BEHeatfluxes'];
            EHFModelIdentifier = 'TABS';
            self.BRCMmodel.declareEHFModel(EHFModelClassFile,EHFModelDataFile,EHFModelIdentifier);
            
            % Radiators
            EHFModelClassFile = 'Radiators.m';
            EHFModelDataFile = [EHFModelDataDir,filesep,'radiators'];
            EHFModelIdentifier = 'Rad';
            self.BRCMmodel.declareEHFModel(EHFModelClassFile,EHFModelDataFile,EHFModelIdentifier);
            
            % 4) Generate full model
            
            self.BRCMmodel.generateBuildingModel;
        end
        
        function discretize(self, Ts_hrs)
            %--------------------------------------------------------------
            % Returns a discretized model of the continuous model
            % Note: Run first self.getModel to create the continuous model.
            %--------------------------------------------------------------
            
            
            self.BRCMmodel.building_model.setDiscretizationStep(Ts_hrs);
            self.BRCMmodel.building_model.discretize();
            
            self.discreteTimeModel.ss = self.BRCMmodel.building_model.discrete_time_model;
            
            % set the identifiers with the building name at the end - so as
            % to be identified as different variables
            nx = size(self.BRCMmodel.building_model.identifiers.x,1);
            nq = size(self.BRCMmodel.building_model.identifiers.q,1);
            nu = size(self.BRCMmodel.building_model.identifiers.u,1);
            nv = size(self.BRCMmodel.building_model.identifiers.v,1);
            ny = size(self.BRCMmodel.building_model.identifiers.y,1);
            
            self.discreteTimeModel.identifiers.x = cell(nx,1);
            self.discreteTimeModel.identifiers.q = cell(nq,1);
            self.discreteTimeModel.identifiers.u = cell(nu,1);
            self.discreteTimeModel.identifiers.v = cell(nv,1);
            self.discreteTimeModel.identifiers.y = cell(ny,1);
            for ii = 1:nx
                self.discreteTimeModel.identifiers.x{ii} = sprintf([self.BRCMmodel.building_model.identifiers.x{ii},...
                    '_',self.modelName]);
            end
            for ii = 1:nq
                self.discreteTimeModel.identifiers.q{ii} = sprintf([self.BRCMmodel.building_model.identifiers.q{ii},...
                    '_',self.modelName]);
            end
            for ii = 1:nu
                self.discreteTimeModel.identifiers.u{ii} = sprintf([self.BRCMmodel.building_model.identifiers.u{ii},...
                    '_',self.modelName]);
            end
            for ii = 1:nv
                self.discreteTimeModel.identifiers.v{ii} = sprintf([self.BRCMmodel.building_model.identifiers.v{ii},...
                    '_',self.modelName]);
            end
            for ii = 1:ny
                self.discreteTimeModel.identifiers.y{ii} = sprintf([self.BRCMmodel.building_model.identifiers.y{ii},...
                    '_',self.modelName]);
            end
            
            
            % initial state
            nx = length(self.discreteTimeModel.identifiers.x);
            self.discreteTimeModel.x0 = 23*ones(nx,1);
            
        end
        
        function efficiencyMatrix(self)
            %--------------------------------------------------------------
            % Returns the efficiency matrix (input node - efficiency - output node (bulding input)
            % for every input of the model
            % Note: Run first self.getModel to create the continuous model.
            %--------------------------------------------------------------
            
            [~, ~, costs_raw]   = xlsread(self.costPath);
            costParameters = struct();
            
            % AHU
            [r_begin,c_begin] = find(strcmp('Begin AHU', costs_raw));
            [~      ,c_end  ] = find(strcmp('End AHU', costs_raw));
            for n = c_begin:c_end
                eval(['costParameters.' costs_raw{r_begin+1, n} '= costs_raw{r_begin+2, n};' ]);
            end
            
            % TABS
            [r_begin,c_begin] = find(strcmp('Begin TABS', costs_raw));
            [~      ,c_end  ] = find(strcmp('End TABS', costs_raw));
            for n = c_begin:c_end
                eval(['costParameters.' costs_raw{r_begin+1, n} '= costs_raw{r_begin+2, n};' ]);
            end
            
            % Radiator
            [r_begin,c_begin] = find(strcmp('Begin Radiator', costs_raw));
            [~      ,c_end  ] = find(strcmp('End Radiator', costs_raw));
            for n = c_begin:c_end
                eval(['costParameters.' costs_raw{r_begin+1, n} '= costs_raw{r_begin+2, n};' ]);
            end
            
            costParameters = self.getkWCostParams(costParameters); % Transform the costs to kW
            costVec = self.BRCMmodel.building_model.getCostVector(costParameters);
            effVec = 1./costVec;
            
            % create the efficiency matrix for the building
            nu = length(self.discreteTimeModel.identifiers.u);
            outputNode = self.discreteTimeModel.identifiers.u;
            inputNode = self.identifyNode(outputNode,self.nodeName);
            
            effMatrix = cell(nu,3);
            effMatrix(:,1) = inputNode;
            for ii = 1:nu
                effMatrix{ii,2} = effVec(ii);
            end
            effMatrix(:,3) = outputNode;
            
            self.discreteTimeModel.effMatrix = effMatrix;
            
        end
        
        function constraints(self,x0,v0)
            %--------------------------------------------------------------
            % constraints = constraints(self,x0,v0)
            % conFileName : File name containing the constraints of the
            % specific building.
            % Returns constraints for the local BRCMmodel as specified by the local constraint file.
            %--------------------------------------------------------------
            
            
            %% Generate constraints
            
            % Generate Fx, Fu, Fv, such that Fx*x+Fu*u+Fv*v <= g based on
            % BRCM model
            
            
            [~, ~, constr_raw]   = xlsread(self.constraintPath);
            constraintsParameters = struct();
            
            % AHU
            [r_begin,c_begin] = find(strcmp('Begin AHU', constr_raw));
            [~      ,c_end  ] = find(strcmp('End AHU', constr_raw));
            for n = c_begin:c_end
                eval(['constraintsParameters.' constr_raw{r_begin+1, n} '= constr_raw{r_begin+2, n};' ]);
            end
            if nargin < 3
                constraintsParameters.AHU.x = repmat(constraintsParameters.AHU.x, length(self.BRCMmodel.building_model.identifiers.x), 1);
                constraintsParameters.AHU.v_fullModel = repmat(constraintsParameters.AHU.v_fullModel, length(self.BRCMmodel.building_model.identifiers.v), 1);
            else
                constraintsParameters.AHU.x = x0;
                constraintsParameters.AHU.v_fullModel = v0;
            end
            % BuildingHull
            [r_begin,c_begin] = find(strcmp('Begin Building Hull', constr_raw));
            [~      ,c_end  ] = find(strcmp('End Building Hull', constr_raw));
            for n = c_begin:c_end
                eval(['constraintsParameters.' constr_raw{r_begin+1, n} '= constr_raw{r_begin+2, n};' ]);
            end
            
            % Zone specific
            [r_begin,c_begin] = find(strcmp('Begin Zones', constr_raw));
            [r_end  ,c_end  ] = find(strcmp('End Zones', constr_raw));
            for m = r_begin+1:2:r_end-1
                for n = c_begin:c_end
                    eval(['constraintsParameters.' constr_raw{m, n} '= constr_raw{m+1, n};' ]);
                end
            end
            % Generate constraints using BRCM Toolbox
            [Fx,Fu,Fv,g] = self.BRCMmodel.building_model.getConstraintsMatrices(constraintsParameters);
            
            if ~isempty(nonzeros(Fx))
                error('Control-input constraint matrix cannot be generated. State constraint matrix has to be empty!')
            end
            if ~isempty(nonzeros(Fv))
                error('Control-input constraint matrix cannot be generated. Distrubance-input constraint matrix has to be empty!')
            end
            
            % contraint set
            self.discreteTimeModel.constraints.Fx = Fx;
            self.discreteTimeModel.constraints.Fu = Fu;
            self.discreteTimeModel.constraints.Fv = Fv;
            self.discreteTimeModel.constraints.g = g;
            self.discreteTimeModel.constraints.parameters = constraintsParameters;
        end       
        
    end
    
        
    
    methods (Static = true)
        function inputNode = identifyNode(outputNode,nodeName)
            % get the input vector for the energy hub according
            nm = length(outputNode);
            inputNode = cell(nm,1);
            for ii = 1:nm
                if strncmp('u_blinds',outputNode{ii},length('u_blinds'))
                    inputNode{ii} = sprintf([nodeName,'Elec']);
                elseif strncmp('u_AHU',outputNode{ii},length('u_AHU'))
                    inputNode{ii} = sprintf([nodeName,'Elec']);
                elseif strncmp('u_BEH_FloorHeating',outputNode{ii},length('u_BEH_FloorHeating'))
                    inputNode{ii} = sprintf([nodeName,'Elec']);
                elseif strncmp('u_BEH_cTABS',outputNode{ii},length('u_BEH_cTABS'))
                    inputNode{ii} = sprintf([nodeName,'Cool']);
                elseif strncmp('u_BEH_hTABS',outputNode{ii},length('u_BEH_hTABS'))
                    inputNode{ii} = sprintf([nodeName,'Heat']);
                elseif strncmp('u_BEH_Cooled',outputNode{ii},length('u_BEH_Cooled'))    % CooledCeiling and CooledBE
                    inputNode{ii} = sprintf([nodeName,'Cool']);
                elseif strncmp('u_BEH_Heated',outputNode{ii},length('u_BEH_Heated'))    % HeatedBE
                    inputNode{ii} = sprintf([nodeName,'Heat']);
                elseif strncmp('u_rad',outputNode{ii},length('u_rad'))
                    inputNode{ii} = sprintf([nodeName,'Heat']);
                end
            end
        end
        
        function costParametersNew = getkWCostParams(costParameters)
            % check the integrity of values
            basevalue = costParameters.AHU.costPerJouleCooled;
            if basevalue~= costParameters.AHU.costPerJouleHeated
                error('Wrong cost vectors - check your file')
            elseif basevalue~= costParameters.Rad.costPerJouleHeated
                error('Wrong cost vectors - check your file')
            elseif basevalue~=costParameters.TABS.costPerJouleHeated
                error('Wrong cost vectors - check your file')
            elseif basevalue~=costParameters.TABS.costPerJouleCooled
                error('Wrong cost vectors - check your file')
            end
            
            % create the new output struct in kW
            basevalue = basevalue * 1000;
            
            % AHU
            costParameters.AHU.costPerJouleCooled = costParameters.AHU.costPerJouleCooled/basevalue;
            costParameters.AHU.costPerJouleHeated = costParameters.AHU.costPerJouleHeated/basevalue;
            costParameters.AHU.costPerKgAirTransported = costParameters.AHU.costPerKgAirTransported/basevalue;
            costParameters.AHU.costPerKgCooledByEvapCooler = costParameters.AHU.costPerKgCooledByEvapCooler/basevalue;
            
            % TABS
            costParameters.TABS.costPerJouleCooled = costParameters.TABS.costPerJouleCooled/basevalue;
            costParameters.TABS.costPerJouleHeated = costParameters.TABS.costPerJouleHeated/basevalue;
            
            % Rad
            costParameters.Rad.costPerJouleHeated = costParameters.Rad.costPerJouleHeated/basevalue;
            
            costParametersNew = costParameters;
        end
    end
end

