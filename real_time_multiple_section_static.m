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

tendon1=[];
tendon2=[];
tendon3=[];
tendon4=[];
tendon5=[];
tendon6=[];
tendon7=[];
tendon8=[];
tendon9=[];
pos1=[];
pos2=[];
pos3=[];

pyrun("import MultipleSection");% Import necessary Python libraries
pyrun("import numpy as np");
pyrun("import tensorflow as tf");
pyrun("from tensorflow.keras.models import load_model");
pyrun("from sklearn.preprocessing import StandardScaler");
pyrun("from sklearn.model_selection import train_test_split");

% Load the model
pyrun("model = load_model('mlp_multiple7.keras')");
%% Calibrate Manipulator
Zero_Position = Manipulator_Calibration(Motor_IDs,Motor_Orients);
Zero_Position_mod=Zero_Position 
% Zero_Position_mod=mod(Zero_Positi3on,4096) 
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
L0_seg=100
N_ele=12
R_arm=18;%mm
T_angle=([0 120 240 60  180 300 0 120 240]+0)*pi/180;%The position angle of three tendons 
pulley_radius=8%mm
% load('Calibration-25-Sep-2024.mat')
% Zero_Position=Zero_Position_mod
% Zero_Position
current=0
 
Dxl_Passive_Current=[200 200 200 200 200 200 200 200 200];% the current 

% Dxl_Movement_Positions=flip(Dxl_Movement_Positions,1)
Stiff_num=size(Dxl_Passive_Current,1)

%%
% Enable Dynamixel Torque and set Operating_Mode for all motors
DXL_IDs = Motor_IDs;
Motor_num = size(DXL_IDs, 2);
recorded_positions = []; % Initialize an array to store positions
positions = [] ;
lengths = [] ;
for i = 1:Motor_num
    % Set Operating Mode and enable Torque for each motor
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_Operating_Mode, Operating_Mode);
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_IDs(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
    dxl_comm_result = getLastTxRxResult(port_num , PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel #%d has been successfully connected \n', DXL_IDs(i));
    end
end

% Add parameter storage for Dynamixels' present position value
for i = 1:Motor_num
    dxl_addparam_result = groupSyncReadAddParam(groupread_num, DXL_IDs(i));
    if dxl_addparam_result ~= true
        fprintf('[ID:%03d] groupSyncRead addparam failed', DXL_IDs(i));
        return;
    end
end

% Start timing for 2 minutes
tic;
while toc < 5
    disp("move")
end 
while toc < 10  % Loop for 120 seconds (2 minutes)
    % Syncwrite goal position values for each motor
    for i = 1:Motor_num
        pause(0.1)
        Dxl_Passive_Current=[current current current current current current current current current];% the current 
        disp(current)
        % Dxl_Movement_Positions=flip(Dxl_Movement_Positions,1)
        Stiff_num=size(Dxl_Passive_Current,1);
        dxl_addparam_result = groupSyncWriteAddParam(groupwrite_num, DXL_IDs(i), typecast(int32(Dxl_Passive_Current(1, i)), 'uint32'), LEN_PRO_GOAL_CURRENT);
        if dxl_addparam_result ~= true
            fprintf('[ID:%03d] groupSyncWrite addparam failed', DXL_IDs(i));
            return;
        end
    end

    % Transmit Syncwrite packet
    groupSyncWriteTxPacket(groupwrite_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Clear syncwrite parameters for next iteration
    groupSyncWriteClearParam(groupwrite_num);

    % Syncread present position values of all motors
    groupSyncReadTxRxPacket(groupread_num);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    end

    % Store current positions in an array
    current_positions = zeros(1, Motor_num);
    for i = 1:Motor_num
        dxl_getdata_result = groupSyncReadIsAvailable(groupread_num, DXL_IDs(i), ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
        if dxl_getdata_result ~= true
            fprintf('[ID:%03d] groupSyncRead getdata failed', DXL_IDs(i));
            return;
        end
        % Get position and convert to signed 32-bit integer
        current_positions(i) = typecast(uint32(groupSyncReadGetData(groupread_num, DXL_IDs(i), ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)), 'int32');
                
    end

    % Add current positions to the recorded positions array
    recorded_positions = [recorded_positions; current_positions];
    if size(positions, 1) < 20
     
     
        % If we have fewer than 50 positions, just append the new one
        positions = [positions; current_positions];
    else
        
        % If there are already 50 positions, remove the oldest and append the new one
        positions = [positions(2:end, :); current_positions];
        %display(positions)
    end
    D_TL = (double(positions) - Zero_Position) / 4095 * pi * 8;
    D_PAgl=360*D_TL/(10*pi);

    for ii=1:1
        Deform_arm(1,(1:3)+(ii-1)*3)=TL2Deform_seg( D_TL(end,(1:3)+(ii-1)*3),R_arm,T_angle((1:3)+(ii-1)*3) );
    end
    [SM_ele,SM_sec]=Config_Arm_draw(Deform_arm,L0_seg,N_ele,1);
    %DataFrame =table(D_TL,'VariableNames',{'tendon'});
    %writematrix(DataFrame,"tendons.csv","WriteMode","append");
    %writematrix(D_TL,'tendons.csv','WriteMode','append');
    % Pause briefly to control the sampling rate
    last_D_TL=tail(D_TL, 1)
    
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
    
    current = round(result);
    tendon1 = [tendon1,last_D_TL(1)];
    tendon2 = [tendon2,last_D_TL(2)];
    tendon3 = [tendon3,last_D_TL(3)];
    tendon4 = [tendon4,last_D_TL(4)];
    tendon5 = [tendon5,last_D_TL(5)];
    tendon6 = [tendon6,last_D_TL(6)];
    tendon7 = [tendon7,last_D_TL(7)];
    tendon8 = [tendon8,last_D_TL(8)];
    tendon9 = [tendon9,last_D_TL(9)];

    pos1 = [pos1,SM_sec(1:1,4:4,2)];
    pos2 = [pos2,SM_sec(2:2,4:4,2)];
    pos3 = [pos3,SM_sec(3:3,4:4,2)];
    
    
    pause(0.1);  % Adjust this as needed for your timing requirements
end

DataFrame =table([current],[SM_sec(1:1,4:4,2)],[SM_sec(2:2,4:4,2)], ...
    [SM_sec(3:3,4:4,2)],[last_D_TL(1)],[last_D_TL(2)],[last_D_TL(3)], ...
    [var(pos1)],[var(pos2)],[var(pos3)],[var(tendon1)],[var(tendon2)], ...
    [var(tendon3)],[var(tendon4)],[var(tendon5)],[var(tendon6)], ...
    [var(tendon7)],[var(tendon8)],[var(tendon9)],'VariableNames', ...
    {'current','x','y','z','M1','M2','M3','vx','vy','vz','vM1','vM2', ...
    'vM3','vM4','vM5','vM6','vM7','vM8','vM9'})
writetable(DataFrame,"static_results_multiple4.csv","WriteMode","append")
tendon1=[];
tendon2=[];
tendon3=[];
tendon4=[];
tendon5=[];
tendon6=[];
tendon7=[];
tendon8=[];
tendon9=[];
pos1=[];
pos2=[];
pos3=[];


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










