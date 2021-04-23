% ----------------------------------------------------------------------
% DAQ-Duino: Matlab side code. Make sure Arduino side is up and running.
% Author: Prof. Antonio Silveira (asilveira@ufpa.br)
%   Laboratory of Control and Systems, UFPA (www.ufpa.br)
%   Group of Control and Systems, UDESC (www.udesc.br)
% ----------------------------------------------------------------------
%
% Description: this is a very simple code to handle data acquisition essays
%              using Matlab and Arduino. The Arduino side code is required
%		       in order to receive I/O requests from this Matlab side.
%              I/O range is 0V to 5V.
%
% Functions available:
%   DAQDUINO_START
%   DAQDUINO_END
%   DAQDUINO_READ
%   DAQDUINO_WRITE
%
% Use "help [name_function]" to get help!
%
% -----------------------------------------
% DAQDUINO_START(COMM) , Inform the comm port name String
%                        in use by Arduino, e.g. 'COM4'.

function []=daqduino_start(commport,vel),
    global s;
    s = serial(commport,'BaudRate',vel); % Check you Arduino COMM port
%     s = serial(commport,'BaudRate',9600); % Check you Arduino COMM port
    fopen(s);
    pause(3); % Wait 1 second to stabilize the link
    fprintf(s,'%s', '0.0' ); % sets the PWM in use (Arduino side) to zero.
                             % Also, Arduino will send the current state of
                             % y(k) to the Matlab buffer.
    
disp(['DaqDuino started! Connection is open on port ' +commport]);
disp('Available functions: daqduino_end(), daqduino_read(),');
disp('daqduino_write(u(k),Ts).');
disp('');
disp('---------------------------------------------------');
disp('DAQ-Duino, 2013-2019.');
disp('Laboratory of Control and Systems (LACOS, ufpa.br).');
disp('Author:Bruno Gomes Dutra (brunodutra@ufpa.br).');
disp('---------------------------------------------------');