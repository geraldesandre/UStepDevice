close all;
clear all;
clc;

%% Global parameters

aurora_present = 1;

sensor_angle_inside_needle = 40.0;

% Needle initial orientation
needle_V0 = [0 0 1];
needle_N0 = [-sind(sensor_angle_inside_needle) cosd(sensor_angle_inside_needle) 0];

n_preparation_step = 12;
preparation_step_size = 10;
preparation_insertion_speed = 2.5;
preparation_rotation_speed = 0.1;

% Insertion trajectory
n_step = 22;
n_step_no_rotation = 0;
step_size = 8;
insertion_speed = 1.0;
rotation_speed = 1.0;
% minimum_insertion = 4.0;

duty_cycle = 0.75;

%% Configure the TCP/IP client for communicating with the Raspberry Pi

ustep_device = UStepDeviceHandler(n_step);

%% Configure the Aurora sensor object

aurora_device = AuroraDriver('/dev/ttyUSB1');
if(aurora_present)    
    aurora_device.openSerialPort();
    aurora_device.init();
    aurora_device.detectAndAssignPortHandles();
    aurora_device.initPortHandleAll();
    aurora_device.enablePortHandleDynamicAll();
    aurora_device.startTracking();
end

%% Set file name for storing the results

fprintf('Duty Cycle Experiment - DC = %.2f\n', duty_cycle);
output_file_name = input('Type the name of the file for saving the results\n','s');

experiment_start = tic;

%% Adjust the needle starting position

if(aurora_present)
    fprintf('Place the needle inside the device \nMake sure to align the needle tip to the end of the device\n');
    input('When you are done, hit ENTER to close the front gripper\n');
    ustep_device.closeFrontGripper();
    
    % Moving the needle forward until it gets detected by the Aurora system
    fprintf('Adjusting the needle initial orientation\n');
    
    % Moving the needle backward
    for i_preparation_step = 1:n_preparation_step
        ustep_device.moveForward(preparation_step_size, preparation_insertion_speed);
    end
    
    % Adjusting the needle orientation
    aurora_device.updateSensorDataAll();
    needle_quaternion = quatinv(aurora_device.port_handles(1,1).rot);
    correction_angle = measureNeedleCorrectionAngle(needle_quaternion, needle_N0);
    
    fprintf('Needle found! Correction angle = %.2f degrees \n', correction_angle);
    ustep_device.rotateNeedleDegrees(correction_angle, preparation_rotation_speed);
    
    aurora_device.BEEP('1');
    answer = input('\nDoes this angle seem correct? (y/n)\n','s');
    if(~(strcmp(answer, 'y') || strcmp(answer, 'Y') || strcmp(answer, 'yes') || strcmp(answer, 'Yes') || strcmp(answer, 'YES')))
        
        correction_angle = 0;
        while 1
            angle = input('Type the correction angle, in CW direction, to adjust the needle orientation:\n');
            
            if(angle == 0)
                fprintf('Needle orientation adjusted.\n');
                fprintf('Next time, you should set the variable "sensor_angle_inside_needle" to %.2f\n', sensor_angle_inside_needle+correction_angle);
                break;
            else
                correction_angle = correction_angle + angle;
                ustep_device.rotateNeedleDegrees(angle, preparation_rotation_speed);
            end
        end
        
    end
    
    % Moving the needle backward
    for i_preparation_step = 1:n_preparation_step
        ustep_device.moveBackward(preparation_step_size, preparation_insertion_speed);
    end
end

ustep_device.closeFrontGripper();
ustep_device.openBackGripper();

preparation_time = toc(experiment_start);
insertion_start = tic;

%% Perform forward steps

aurora_device.BEEP('2');
fprintf('\nPreparing to start the experiment. Place the gelatin\n');
input('Hit ENTER when you are ready\n');

for i_step = 1:n_step_no_rotation
%     fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, mS = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, W = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, rotation_speed, duty_cycle);
    
    % Measure needle pose before moving
    ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
    ustep_device.moveForward(step_size, insertion_speed);
%     ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, minimum_insertion, 0.0);
    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, rotation_speed, 0.0);
end

for i_step = n_step_no_rotation+1:n_step
%     fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, mS = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    fprintf('\nPerforming step %d/%d: S = %.2f, V = %.2f, W = %.2f, DC = %.2f\n', i_step, n_step, step_size, insertion_speed, rotation_speed, duty_cycle);
    
    % Measure needle pose before moving
    ustep_device.savePoseForward(aurora_device, i_step);
    
    % Move needle
%     ustep_device.moveDC(step_size, insertion_speed, minimum_insertion, duty_cycle);
%     ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, minimum_insertion, duty_cycle);
    ustep_device.moveDC(step_size, insertion_speed, rotation_speed, duty_cycle);
    ustep_device.saveCommandsDC(i_step, step_size, insertion_speed, rotation_speed, duty_cycle);
end

% Measure the final needle pose
ustep_device.savePoseForward(aurora_device, n_step+1);

insertion_time = toc(insertion_start);
removing_start = tic;

%% Perform backward steps

aurora_device.BEEP('3');
fprintf('\nNeedle insertion complete!\n');
input('Hit ENTER to start retreating the needle\n');

% Measure the final needle pose
ustep_device.savePoseBackward(aurora_device, n_step+1);

for i_step = n_step:-1:1
    fprintf('\nPerforming backward step %d/%d\n', i_step, n_step);
    
    % Move needle
    ustep_device.moveBackward(step_size, insertion_speed);
    
    % Measure needle pose before retreating
    ustep_device.savePoseBackward(aurora_device, i_step);

end

%% Save the results and close the program

aurora_device.BEEP('4');
removing_time = toc(removing_start);
experiment_time = toc(experiment_start);

% Close the aurora system
if(aurora_present)
    aurora_device.stopTracking();
    delete(aurora_device);
end

% Save experiment results
save(sprintf('%s.mat',output_file_name));