% Editor: Qinghua Guan
% 2022 07 26
% Make it compatible for multiple motors
% Calibrate the zero postion of the manipulator 
% enable the manipulator
% control the manipulator with positions
% disable the manipulator
% motor number:[]

clc
clear
%

% set manipulator
Motor_IDs=[1 2 3 4 5 6 7 8 9]+10;
Motor_Orients=[1 1 1 1 1 1 1 1 1];
Motor_num=size(Motor_IDs,2);
DXL_IDs=[1 2 3 4 5 6 7 8 9];
DXL_Orients=[1 1 1 1 1 1 1 1 1];
%% Calibrate Manipulator
Zero_Position = Manipulator_Calibration(Motor_IDs,Motor_Orients);
Zero_Position_mod=Zero_Position 
% Zero_Position_mod=mod(Zero_Position,4096) 
save(['Calibration-',date],"Zero_Position_mod")
%%
% Set the manipulator motors
% Load libraries
% Set: Control table address
     % Data Byte Length
     % Protocol version
     % Operating_Mod
     % Default setting: Dxl_ID,BAUDRATE,DEVICENAME(portname)    
% set and open port

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


%Control table address
ADDR_PRO_TORQUE_ENABLE          = 64;                 % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION          = 116;

ADDR_PRO_GOAL_CURRENT          = 102;
ADDR_PRO_PRESENT_CURRENT       = 126;

ADDR_PRO_PRESENT_POSITION       = 132;
ADDR_PRO_Operating_Mode         = 11;                 % address for Operating_Mode
% Data Byte Length
LEN_PRO_GOAL_POSITION       = 4;
LEN_PRO_PRESENT_POSITION    = 4;

LEN_PRO_GOAL_CURRENT       = 2;
LEN_PRO_PRESENT_CURRENT    = 2;
% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Operating_Mod
Operating_Mode =               0;          % 0	Current Control Mode	DYNAMIXEL only controls current(torque) regardless of speed and position. This mode is ideal for a gripper or a system that only uses current(torque) control or a system that has additional velocity/position controllers.
                                           % 1	Velocity Control Mode	This mode controls velocity. This mode is identical to the Wheel Mode(endless) from existing DYNAMIXEL. This mode is ideal for wheel-type robots.
                                           % 3(Default)	Position Control Mode	This mode controls position. This mode is identical to the Joint Mode from existing DYNAMIXEL. Operating position range is limited by the Max Position Limit(48) and the Min Position Limit(52). This mode is ideal for articulated robots that each joint rotates less than 360 degrees.
                                           % 4	Extended Position Control Mode(Multi-turn)	This mode controls position. This mode is identical to the Multi-turn Position Control from existing DYNAMIXEL. 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for multi-turn wrists or conveyer systems or a system that requires an additional reduction gear. Note that Max Position Limit(48), Min Position Limit(52) are not used on Extended Position Control Mode.
                                           % 5	Current-based Position Control Mode	This mode controls both position and current(torque). Up to 512 turns are supported(-256[rev] ~ 256[rev]). This mode is ideal for a system that requires both position and current control such as articulated robots or grippers.
                                           % 16	PWM Control Mode (Voltage Control Mode)	This mode directly controls PWM output. (Voltage Control Mode)


% Default setting
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM9';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'

TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 40;           % Dynamixel moving status threshold

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
groupwrite_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_CURRENT, LEN_PRO_GOAL_CURRENT);

% Initialize Groupsyncread Structs for Present Position
groupread_num = groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);


index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_addparam_result = false;              % AddParam result
dxl_getdata_result = false;               % GetParam result


dxl_error = 0;                              % Dynamixel error
dxl1_present_position = 0;                  % Present position
dxl2_present_position = 0;

% 
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




% Define the motion of each steps

%geometry of Arm
L0_seg=200
N_ele=12
R_arm=18;%mm
T_angle=([0 120 240 60  180 300 0 120 240]+0)*pi/180;%The position angle of three tendons 
pulley_radius=8%mm
% load('Calibration-21-Jul-2024.mat')
% Zero_Position=Zero_Position_mod
% Zero_Position

 
Dxl_Passive_Current=[20 20 20 20 20 20 20 20 20]; 

% Dxl_Movement_Positions=flip(Dxl_Movement_Positions,1)
Stiff_num=size(Dxl_Passive_Current,1)

%%
% Enable_Manipulator and run motions
%Enable Dynamixel Torque and set Operating_Mode for all motors
DXL_IDs= Motor_IDs;
Motor_num=size(DXL_IDs,2);
for i=1:Motor_num
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);    
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_Operating_Mode,Operating_Mode);%Set Operating Mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);    
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

% Add parameter storage for Dynamixels present position value
% if output multi-information(exp position and current), clear the read
% storage before the next read
for i=1:Motor_num
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_IDs(i));
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_IDs(i));
        return;
    end
end



index=0;
while 1

    if input('Press any key to continue! (or input e to quit!)\n', 's') == ESC_CHARACTER
        break;
    end
% for index=1:Move_num
    index=index+1;
    % Add Dynamixel goal position values of all motors to the Syncwrite storage
    validInput = false;
    if input('Press any key to continue! (or input c to change current!)\n', 's') == "c" || index ==1
        while ~ validInput
            
            ip_curr=input("please enter current");
            
            if isscalar(ip_curr) && isnumeric(ip_curr) && ip_curr == round(ip_curr)
                
                validInput =true;
                Dxl_Passive_Current=[ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr] ;% the current 
                Stiff_num=size(Dxl_Passive_Current,1)
            end
        
            
        end
        for i=1:Motor_num
            dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_IDs(i), typecast(int32(Dxl_Passive_Current(1,i)), 'uint32'), LEN_PRO_GOAL_CURRENT);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL_IDs(i));
                return;
            end
        end
    end
    % Syncwrite goal position
    groupSyncWriteTxPacket(groupwrite_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear syncwrite parameter storage
    groupSyncWriteClearParam(groupwrite_num);
    h=figure(1);  
    set(gcf,'position',[100 100 900 400]);
    positions=[]; 
    while ishandle(h)
        %         Syncread present position
        groupSyncReadTxRxPacket(groupread_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end

        % Check if groupsyncread data of all Dynamixels is available
        for i=1:Motor_num
            dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL_IDs(i), ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if dxl_getdata_result ~= true
                fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_IDs(i));
                return;
            end
        end

        %Get all Dynamixels present position value
        flag=0;
        Flag_V=zeros(1,Motor_num);
        for i=1:Motor_num
            dxl_present_positions(i) = groupSyncReadGetData(groupread_num, DXL_IDs(i), ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            if (abs(Dxl_Passive_Current(1,i) - typecast(uint32(dxl_present_positions(i)), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
                flag=flag+1;
                Flag_V(i)=1;
            end
        end    %


        if flag==0
            break;
        end
        present_positions=typecast(uint32(dxl_present_positions), 'int32');
        if size(positions,1)<50
            positions=[positions;present_positions];
        else             
            positions=[positions(2:end,:);present_positions ];
        end
        D_TL=(double(positions)-Zero_Position)/4095*pi*8;

        D_PAgl=360*D_TL/(10*pi);

        for ii=1:3
            Deform_arm(1,(1:3)+(ii-1)*3)=TL2Deform_seg( D_TL(end,(1:3)+(ii-1)*3),R_arm,T_angle((1:3)+(ii-1)*3) );
        end
        last_D_TL=tail(D_TL, 1);
        [SM_ele,SM_sec]=Config_Arm_draw(Deform_arm,L0_seg,N_ele,1);
        result = pyrun("import numpy as np; res = mlp.predict(np.array([x, y, z,a,b,c,e,f,g,h]).reshape(1, 10))", ...
                   "res", x=last_D_TL(1), y=last_D_TL(2), z=last_D_TL(3), ...
                   a=last_D_TL(4),b=last_D_TL(5),c=last_D_TL(6),e=last_D_TL(7), ...
                   f=last_D_TL(8),g=last_D_TL(9),h=[SM_sec(3:3,4:4,2)]);
        
        disp(result);
        result = max(20, min(double(result), 500));
        
        ip_curr=round(result);
        Dxl_Passive_Current=[ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr ip_curr] ;% the current 
        Stiff_num=size(Dxl_Passive_Current,1)
        for i=1:Motor_num
            dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_IDs(i), typecast(int32(Dxl_Passive_Current(1,i)), 'uint32'), LEN_PRO_GOAL_CURRENT);
            if dxl_addparam_result ~= true
                fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL_IDs(i));
                return;
            end
        end
        % Syncwrite goal position
        groupSyncWriteTxPacket(groupwrite_num);
        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        end
        % Clear syncwrite parameter storage
        groupSyncWriteClearParam(groupwrite_num);

        
        subplot(1,2,1)
        plot(D_PAgl)
        legend
        subplot(1,2,2) 
        hold off
        [SM_ele,SM_sec]=Config_Arm_draw(Deform_arm,L0_seg,N_ele,1);
        axis equal
        axis([-400 400 -400 400 -100 600]) 
        xlabel('x')
        ylabel('y')
        box on 
        grid on         
        ax = gca;
        ax.XAxisLocation = 'origin';
        ax.YAxisLocation = 'origin';
        
        pause(0.01);

    end
end
%%
% Disable Dynamixel#1 Torque
for i=1:Motor_num
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end


% Close port
closePort(port_num);

% Unload Library
unloadlibrary(lib_name);
%
close all;
clear all;










