classdef EHConverter < handle
    %----------------------------------------------------------------------
    % EHBattery This class is the basic conversion class of the energy hub
    % components and should be inherited by any other class of the toolbox.
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
        discreteTimeModel
    end
    
    methods
        function obj = EHConverter(convName,inputNode,outputNode,A,B,C,D,Fx,Fu,g)
            if nargin < 3
                error('No inputs to class constructor');
            elseif nargin ==3
                A = 0;
                B = 0;
                C = 0;
                D = 1;
                Fx = 0; % default case positive inputs only
                Fu = -1;
                g = 0;
            elseif nargin < 7
                % some of A,B,C,D were not given
                error('Not all the state matrices were given');
            elseif nargin == 7
                % do nothing - take the matrices as provided (default case
                % positive inputs)
                nx = size(A,1);
                nu = size(B,1);
                
                Fx = zeros(nu,nx);
                Fu = -eye(nu,nu);
                g = zeros(nu,1);
            elseif nargin < 10
                % some of Fx,Fu,g were not given
                error('Not all the constraints matrices were given');
            end
            obj.modelName = convName;
            obj.setDynamics(A,B,C,D,Fx,Fu,g,inputNode,outputNode);
        end
        
        function setDynamics(self,A,B,C,D,Fx,Fu,g,inputNode,outputNode)
            
            mt = length(self.modelName);
            mt = min(mt,10);
            % create the identifiers
            nu = size(B,1); % number of inputs
            nx = size(A,1); % number of states
            ny = size(C,1);
            uid = cell(nu,1);
            xid = cell(nx,1);
            yid = cell(ny,1);
            for ii = 1:nu
                tsu = sprintf(['u_',self.modelName(1:mt),'_',num2str(ii)]);
                uid{ii} = tsu;
            end
            for ii = 1:nx
                tsx = sprintf(['x_',self.modelName(1:mt),'_',num2str(ii)]);
                xid{ii} = tsx;
            end
            for ii = 1:ny
                tsy = sprintf(['y_',self.modelName(1:mt),'_',num2str(ii)]);
                yid{ii} = tsy;
            end
            self.discreteTimeModel.identifiers.u = uid;
            self.discreteTimeModel.identifiers.x = xid;
            self.discreteTimeModel.identifiers.y = yid;
            
            % create the dynamics
            self.discreteTimeModel.ss.A = A;
            self.discreteTimeModel.ss.B = B;
            self.discreteTimeModel.ss.C = C;
            self.discreteTimeModel.ss.D = D;
            
            % initial state
            self.discreteTimeModel.x0 = zeros(nx,1);
            
            % create the constraints
            self.discreteTimeModel.constraints.Fx = Fx;
            self.discreteTimeModel.constraints.Fu = Fu;
            self.discreteTimeModel.constraints.g = g;
            
            % crete the efficiency matrix
            effMatrix = cell(nu+ny,3);
            for ii = 1:nu
                effMatrix{ii,1} = inputNode{ii};
                effMatrix{ii,2} = 1;
                effMatrix{ii,3} = uid{ii};
            end
            for ii = 1:ny
                effMatrix{nu+ii,1} = yid{ii};
                effMatrix{nu+ii,2} = 1;
                effMatrix{nu+ii,3} = outputNode{ii};
            end
            self.discreteTimeModel.effMatrix = effMatrix;
        end
    end
end