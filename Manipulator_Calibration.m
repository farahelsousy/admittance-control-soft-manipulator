function [Zero_Position] = Manipulator_Calibration(DXL_IDs,DXL_Orients)
% Modifier: Qinghua Guan
% 2022 07 26
% Make it compatible for multiple motors
% Calibrate the zero postion of the manipulator
% motor number:[]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2017 ROBOTIS CO., LTD.
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Author: Ryu Woon Jung (Leon)

%
% *********     Sync Read and Sync Write Example      *********
%
%
% Available Dynamixel model on this example : All models using Protocol 2.0
% This example is designed for using two Dynamixel PRO 54-200, and an USB2DYNAMIXEL.
% To use another Dynamixel model, such as X series, see their details in E-Manual(emanual.robotis.com) and edit below variables yourself.
% Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
%

% clc;
% clear all;
%%
Ctrl_type=2;%  1: PWM control motor,2: currrent control motor

lib_name = '';

if strcmp(computer, 'PCWIN')
    lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
    lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
    lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
    lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
    lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
end

% Control table address
ADDR_TORQUE_ENABLE          = 64;                 % Control table address is different in Dynamixel model
ADDR_GOAL_POSITION          = 116;
ADDR_PRESENT_POSITION       = 132;

% info. related to CURRENT
% 0 ~ 1750 * 1[mA]
ADDR_CURRENT_LIMIT           = 38;
ADDR_GOAL_CURRENT           = 102;
ADDR_PRESENT_CURRENT       = 126;
LEN_CURRENT_LIMIT       = 2;
LEN_GOAL_CURRENT       = 2;
LEN_PRESENT_CURRENT    = 2;
CURRENT_LIMIT = 1750;% nominal value.

% info. related to PWM
% 0 ~ 885
ADDR_PWM_LIMIT           = 36;
ADDR_GOAL_PWM           = 100;
ADDR_PRESENT_PWM       = 124;
ADDR_PWM_SLOPE           = 62;
LEN_PWM_LIMIT       = 2;
LEN_GOAL_PWM       = 2;
LEN_PRESENT_PWM    = 2;
LEN_SLOPE_PWM    = 1;
PWM_LIMIT = 885;
PWM_SLOPE_LIMIT    = 255;

ADDR_Operating_Mode         = 11;                 % address for Operating_Mode
% Data Byte Length
LEN_GOAL_CURRENT       = 2;
LEN_PRESENT_CURRENT    = 2;
LEN_GOAL_POSITION      = 4;
LEN_PRESENT_POSITION   = 4;

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Operating_Mod
if Ctrl_type==1;
    Operating_Mode =               16;          
elseif Ctrl_type==2;
    Operating_Mode =               0;    
end
                                            % 0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and current. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/current controllers.
                                            % 1	Velocity Control Mode	This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
                                            % 3(Default)	Position Control Mode	This mode controls current. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating current range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
                                            % 4	Extended Position Control Mode(Multi-turn)	This mode controls current. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
                                            % 5	Current-based Position Control Mode	This mode controls both current and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both current and current control such as articulated robots or grippers.
                                            % 16	PWM Control Mode (Voltage Control Mode)	This mode directly controls PWM output. (Voltage Control Mode)


% Default setting
Motor_Num=size(DXL_IDs,2);
% DXL_IDs=Motor_IDs;
% DXL_Orients=Motor_Orents;
% DXL_IDs                     = [107,103,106, 114,110,104, 111,105,108];% Dynamixel ID: 1, 116, 118
% DXL_Orients                  = [-1 -1 -1, 1 1 1, 1 1 1];
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM9';       % Check which port is being used on your controller
% ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
if Ctrl_type==1
    DXL_MINIMUM_PWM_VALUE  = 50*ones(1,Motor_Num);     % Dynamixel will rotate between these values
    DXL_MAXIMUM_PWM_VALUE  = 100*ones(1,Motor_Num); % and these value (note that the Dynamixel would not move when the current value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MINIMUM_PWM_VALUE  = DXL_MINIMUM_PWM_VALUE.*DXL_Orients
    DXL_MAXIMUM_PWM_VALUE  = DXL_MAXIMUM_PWM_VALUE.*DXL_Orients
elseif Ctrl_type==2
    DXL_MINIMUM_CURRENT_VALUE  = 100*ones(1,Motor_Num);     % Dynamixel will rotate between these values
    DXL_MAXIMUM_CURRENT_VALUE  = 200*ones(1,Motor_Num); % and these value (note that the Dynamixel would not move when the current value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
    DXL_MINIMUM_CURRENT_VALUE  = DXL_MINIMUM_CURRENT_VALUE.*DXL_Orients
    DXL_MAXIMUM_CURRENT_VALUE  = DXL_MAXIMUM_CURRENT_VALUE.*DXL_Orients
end

DXL_MOVING_STATUS_THRESHOLD = 1;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

% Initialize Groupsyncwrite Structs
if Ctrl_type==1
    groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_PWM, LEN_GOAL_PWM);
elseif Ctrl_type==2
    groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT);
end

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_addparam_current_result = false;              % AddParam result
dxl_addparam_pwm_result = false;              % AddParam result
dxl_addparam_position_result = false;              % AddParam result
dxl_getdata_result = false;               % GetParam result

if Ctrl_type==1
    dxl_goal_pwm = [DXL_MAXIMUM_PWM_VALUE; DXL_MINIMUM_PWM_VALUE];         % Goal PWM;         % Goal PWM
elseif Ctrl_type==2
    dxl_goal_current = [DXL_MAXIMUM_CURRENT_VALUE; DXL_MINIMUM_CURRENT_VALUE];         % Goal current;         % Goal current
end

dxl_error = 0;                              % Dynamixel error
dxl1_present_current = 0;                  % Present current
dxl2_present_current = 0;


% Open port
if (openPort(port_num))
    fprintf('Succeeded to open the port!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port!\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Succeeded to change the baudrate!\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end


% Enable Dynamixel Torque and set Operating_Mode for all motors
%%
for i=1:Motor_Num
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_Operating_Mode,Operating_Mode);%Set Operating Mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel #%d has been successfully connected \n', DXL_IDs(i));
    end
end

% Initialize Groupsyncread Structs for Present Current, PWM, Position
groupread_current_num  = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
groupread_pwm_num  = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_PWM, LEN_PRESENT_PWM);
groupread_position_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);







while 1
    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end

    % Add Dynamixel goal pam/current values of all motors to the Syncwrite storage
    for i=1:Motor_Num
        if Ctrl_type==1
            dxl_addparam_pwm_result = groupSyncWriteAddParam(groupwrite_num, DXL_IDs(i), typecast(int16(dxl_goal_pwm(index,i)), 'uint16'), LEN_GOAL_PWM);
            if dxl_addparam_pwm_result ~= true
                fprintf('[ID:%03d] groupSyncWritePWM addparam failed', DXL_IDs(i));
                return;
            end
        elseif Ctrl_type==2
            dxl_addparam_current_result = groupSyncWriteAddParam(groupwrite_num, DXL_IDs(i), typecast(int16(dxl_goal_current(index,i)), 'uint16'), LEN_GOAL_CURRENT);
            if dxl_addparam_current_result ~= true
                fprintf('[ID:%03d] groupSyncWriteCurrent addparam failed', DXL_IDs(i));
                return;
            end
        end
    end

    % Syncwrite goal pwm/current     
    groupSyncWriteTxPacket(groupwrite_num);     
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(groupwrite_num);
    if index==1
        pause(5)% to wait the motor get the stable position
    elseif index==2
        pause(0.5)
    end


    while 1
        % Add parameter storage for Dynamixels present current value
        for i=1:Motor_Num
            dxl_addparam_current_result = groupSyncReadAddParam(groupread_current_num, DXL_IDs(i));
            if dxl_addparam_current_result ~= true
                fprintf('[ID:%03d] groupSyncRead addparam current failed', DXL_IDs(i));
                return;
            end
        end

        % Add parameter storage for Dynamixels present pwm value
        for i=1:Motor_Num
            dxl_addparam_pwm_result = groupSyncReadAddParam(groupread_pwm_num, DXL_IDs(i));
            if dxl_addparam_pwm_result ~= true
                fprintf('[ID:%03d] groupSyncRead addparam current failed', DXL_IDs(i));
                return;
            end
        end

        % Add parameter storage for Dynamixels present position value
        for i=1:Motor_Num
            dxl_addparam_position_result = groupSyncReadAddParam(groupread_position_num, DXL_IDs(i));
            if dxl_addparam_position_result ~= true
                fprintf('[ID:%03d] groupSyncRead addparam position failed', DXL_IDs(i));
                return;

            end
        end

        % Syncread present current
        if Ctrl_type==2

            groupSyncReadTxRxPacket(groupread_current_num);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            end

            % Check if groupsyncread data of all Dynamixels is available
            for i=1:Motor_Num
                dxl_getdata_result = groupSyncReadIsAvailable(groupread_current_num, DXL_IDs(i), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
                if dxl_getdata_result ~= true
                    fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_IDs(i));
                    return;
                end
            end

            % Get all Dynamixels present current value
            flag=0;
            for i=1:Motor_Num
                dxl_present_currents(i) = groupSyncReadGetData(groupread_current_num, DXL_IDs(i), ADDR_PRESENT_CURRENT, LEN_PRESENT_CURRENT);
                if i==Motor_Num
                    fprintf('[ID:%03d] PresCur:%03d\n', DXL_IDs(i), typecast(uint16(dxl_present_currents(i)), 'int16'));
                else
                    fprintf('[ID:%03d] PresCur:%03d\t', DXL_IDs(i), typecast(uint16(dxl_present_currents(i)), 'int16'));
                end

                if (abs(dxl_goal_current(index,i) - typecast(uint16(dxl_present_currents(i)), 'int16')) > DXL_MOVING_STATUS_THRESHOLD)
                    flag=flag+1;
                end

            end

            % Clear syncread parameter storage of current
            groupSyncReadClearParam(groupread_current_num);
        end


        if Ctrl_type==1
            % Syncread present pwm
            groupSyncReadTxRxPacket(groupread_pwm_num);
            dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
            if dxl_comm_result ~= COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
            end

            % Check if groupsyncread data of all Dynamixels is available
            for i=1:Motor_Num
                dxl_getdata_result = groupSyncReadIsAvailable(groupread_pwm_num, DXL_IDs(i), ADDR_PRESENT_PWM, LEN_PRESENT_PWM);
                if dxl_getdata_result ~= true
                    fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_IDs(i));
                    return;
                end
            end

            % Get all Dynamixels present pwm value
            flag=0;
            for i=1:Motor_Num
                dxl_present_pwms(i) = groupSyncReadGetData(groupread_pwm_num, DXL_IDs(i), ADDR_PRESENT_PWM, LEN_PRESENT_PWM);
                if (abs(dxl_goal_pwm(index,i) - typecast(uint16(dxl_present_pwms(i)), 'int16')) > DXL_MOVING_STATUS_THRESHOLD)
                    flag=flag+1;
                end
            end

            for i=1:Motor_Num
                if i==Motor_Num
                    fprintf('[ID:%03d] GoalPWM:%03d\n', DXL_IDs(i), dxl_goal_pwm(index,i));
                else
                    fprintf('[ID:%03d] GoalPWM:%03d\t', DXL_IDs(i), dxl_goal_pwm(index,i));
                end
            end

            for i=1:Motor_Num
                if i==Motor_Num
                    fprintf('[ID:%03d] PresPWM:%03d\n', DXL_IDs(i), typecast(uint16(dxl_present_pwms(i)), 'int16'));
                else
                    fprintf('[ID:%03d] PresPWM:%03d\t', DXL_IDs(i), typecast(uint16(dxl_present_pwms(i)), 'int16'));
                end
                if (abs(dxl_goal_pwm(index,i) - typecast(uint16(dxl_present_pwms(i)), 'int16')) > DXL_MOVING_STATUS_THRESHOLD)
                    flag=flag+1;
                end
            end

            % Clear syncread parameter storage of PWM
            groupSyncReadClearParam(groupread_pwm_num);
        end

        % Syncread present position
        groupSyncReadTxRxPacket(groupread_position_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        % Check if groupsyncread data of all Dynamixels is available
        for i=1:Motor_Num
            dxl_getdata_result = groupSyncReadIsAvailable(groupread_position_num, DXL_IDs(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            if dxl_getdata_result ~= true
                fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_IDs(i));
                %                 return;
            end
        end
        % Get all Dynamixels present position value
        for i=1:Motor_Num
            dxl_present_positions(i) = groupSyncReadGetData(groupread_position_num, DXL_IDs(i), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            if i==Motor_Num
                fprintf('[ID:%03d]  PresPos:%03d\n', DXL_IDs(i), typecast(uint32(dxl_present_positions(i)), 'int32'));
            else
                fprintf('[ID:%03d]  PresPos:%03d\t', DXL_IDs(i), typecast(uint32(dxl_present_positions(i)), 'int32'));
            end

        end
        % Clear syncread parameter storage of position
        groupSyncReadClearParam(groupread_position_num);
        %         flag=0
        if flag==0
            break;
        end


    end

    %Change goal pwm
    if index == 1
        index = 2;
    else
        index = 1;
    end
end


%save the calibrated zero position
Zero_Position=dxl_present_positions;
% save('Calibrated Zero Position','Zero_Position');

% Disable Dynamixel#1 Torque
% for i=1:Motor_Num
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
%     dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
%     dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
%     if dxl_comm_result ~= COMM_SUCCESS
%         fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%     elseif dxl_error ~= 0
%         fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%     end
% end


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);

close all;
% clear all;


end