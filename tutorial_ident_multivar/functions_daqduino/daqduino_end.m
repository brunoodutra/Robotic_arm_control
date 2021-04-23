% ----------------------------------------------------------------------
% DAQ-Duino: Matlab side code. Make sure Arduino side is up and running.
% Author: Prof. Antonio Silveira (asilveira@ufpa.br)
%   Laboratory of Control and Systems, UFPA (www.ufpa.br)
%   Group of Control and Systems, UDESC (www.udesc.br)
% ----------------------------------------------------------------------
%
% DAQDUINO_END  Ends the MATLAB serial connection to the Arduino board
%               in use. It also sets the Arduino PWM channel in use to 0V.
%


function []=daqduino_end,
% Disconnect the serial link and put the PWM to zero
    global s;
    fprintf(s,'%s', '0.0' ); % Sends u(k)=0
    fclose(s);
disp('DaqDuino ended.');
disp('');
disp('---------------------------------------------------');
disp('DAQ-Duino, 2013-2015.');
disp('Laboratory of Control and Systems (LACOS, ufpa.br).');
disp('Group of Control and Systems (GCS, udesc.br).');
disp('Author: Prof. Antonio Silveira (asilveira@ufpa.br).');
disp('---------------------------------------------------');