classdef VrepConnector
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        sim;				%Similar to fd
        clientID;			%Used for server connection and server requests
        robot_joints = []	%List of joint handles
        step_time_vrep;		%Integration step used for simulation
    end
    
    methods
        function obj = VrepConnector(port, step_time_vrep)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            addpath vrep_lib/;						%Adding the APIs to the path
            obj.step_time_vrep = step_time_vrep;	
            obj.sim = remApi('remoteApi');			%RemoteAPI object
            obj.sim.simxFinish(-1);
            obj.clientID = obj.sim.simxStart('127.0.0.1', port, true, true, 5000, 5);
            if (obj.clientID > -1)
                disp('Connected to simulator');
            else
                disp('Error in connection');
            end
            % enable the synchronous mode on the client: (integration step on call)
            obj.sim.simxSynchronous(obj.clientID, true);
            % start the simulation:
            obj.sim.simxStartSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
            for i = 1:7 
                [~,obj.robot_joints(i)] = obj.sim.simxGetObjectHandle(obj.clientID, strcat('LBR_iiwa_14_R820_joint',int2str(i)), obj.sim.simx_opmode_blocking);
            end
        
            for i = 1:7
                [~, joint_pos] = obj.sim.simxGetJointPosition(obj.clientID, obj.robot_joints(i), obj.sim.simx_opmode_streaming);
            end
        end
        
        function Close(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.sim.simxStopSimulation(obj.clientID, obj.sim.simx_opmode_blocking);
            obj.sim.simxFinish(-1);
            obj.sim.delete();
        end
        
        function ApplyControl(obj, u, delta_t)
            for i = 1:7
                obj.sim.simxSetJointTargetVelocity(obj.clientID, obj.robot_joints(i), u(i), obj.sim.simx_opmode_oneshot);
            end
            for i = 1:(delta_t/obj.step_time_vrep)				%Number of integrations in delta_t
                obj.sim.simxSynchronousTrigger(obj.clientID);	%Triggering the integration
                % To overcome delay in values according to (Remote API modus operandi) document  
            end
            obj.sim.simxGetPingTime(obj.clientID);				%Synchronizing
        end
        
        function q = GetState(obj)
            q = zeros(7,1);
            for i=1:7
                [~, q(i)] = obj.sim.simxGetJointPosition(obj.clientID, obj.robot_joints(i), obj.sim.simx_opmode_buffer);
            end
        end
    end
end

