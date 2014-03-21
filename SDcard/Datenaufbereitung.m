%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dieses Script macht aus der .csv - Datei brauchbare Daten und        %
% speichert diese in eine mat-Datei ab. Ausserdem wird hier geplottet  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Writer:        Walter von Matt                                    %
%    Email:         walti.vonmatt@bluewin.ch                           %
%                   walter.vonmatt@stud.hslu.ch                        %
%    Organisation:  HSLU                                               %
%    Date:          Oktober 2013                                       %
%    Version:       V4.1                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1)  Erklärungen der Variablen und Normeinheiten
% 2)  Variablendeklarationen (Abtastzeit definieren)
% 3)  Daten werden eingelesen und in Spaltenvektoren gespeichert. Die   %
%     ladende Datei heisst 'data.csv'.                                  %
% 4)  Der Datensatz wird skalierent
% 5)  Der Datensatz wird interpoliert
% 6)  Alle "NAN" Werte werden durch 0 ersetzt
% 7)  Steuereingänge mit Hilfe der Mischmatrix machen
% 8)  Korrektur der translatorischen Winkel durch die
%     Potentiometerstellung
% 9)  Aus den Beschleunigunge die Geschwindikeiten berechnen (Integral)
% 10) Aus den Kreiselgeschwindigkeiten die Drehwinkel berechnen (Integral)
% 11) Absolute Vorwärtsgeschwindigkeit berechnen
% 12) kompletter Datensatz als .mat - Datei abspeichern
% 13) Berechnungen für Flugsimulator FlightGear anstellen
% 14) Simulator starten und mit Hilfe der Simulinkschnittstelle die
%     gemessenen Flugdaten verifizieren
% 15) die besten Daten zwischen XSens und Px4 Board auswählen
% 16) Anregung: laterales Leinenziehen   -> Daten abspeichern fürs CIFER
% 17) Anregung: longitudnaler Motorschub -> Daten abspeichern fürs CIFER
% 18) Diverse Plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear all;
close all;
format compact;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 1)   Erklärungen
%  ---------------------------------------------------------------------
%  Variable ¦ Einheit  ¦  Bezeichnung      ¦ Erläuterung
%  ---------------------------------------------------------------------
%  Lat      ¦ [deg]    ¦  Latitude         ¦ Breitengrad
%  Lon      ¦ [deg]    ¦  Longitude        ¦ Längegrad
%  Alt      ¦ [mm]     ¦  Altitude         ¦ Höhenlage
%  VelN     ¦ [m/s]    ¦  North velocity   ¦ Geschwindigkeit in Norden
%  VelE     ¦ [m/s]    ¦  East velocity    ¦ Geschwindigkeit in Osten
%  VelD     ¦ [m/s]    ¦  Down velocity    ¦ Geschwindigkeit nacht unten
%  Gyro     ¦ [rad/s]  ¦  Angular volocity ¦ Kreiselfrequenz (rotativ)
%  Acc      ¦ [m/s^2]  ¦  Aceleration      ¦ Beschleunigung in X-Y-Z
%  Mag      ¦ [gauss]  ¦  Magnitude        ¦ Lage vom Magnetometer
%  Roll     ¦ [rad]    ¦  Rollwinkel       ¦ 1 rad = 360 / 2pi = 57°
%  (Phi)    ¦ [rad]    ¦  Rollwinkel       ¦ 1 rad = 360 / 2pi = 57°
%  Pitch    ¦ [rad]    ¦  Pitchwinkel      ¦ 1 rad = 360 / 2pi = 57°
%  (Theta)  ¦ [rad]    ¦  Pitchwinkel      ¦ 1 rad = 360 / 2pi = 57°
%  Yaw      ¦ [rad]    ¦  Yawwinkel        ¦ 1 rad = 360 / 2pi = 57°
%  (Psi)    ¦ [rad]    ¦  Yawwinkel        ¦ 1 rad = 360 / 2pi = 57°
%  p        ¦ [rad/s]  ¦  Roll rate        ¦ rot. Geschwindigkeit um phi
%  q        ¦ [rad/s]  ¦  Pitch rate       ¦ rot. Geschwindigkeit um theta
%  r        ¦ [rad/s]  ¦  Yaw rate         ¦ rot. Geschwindigkeit um psi
%  lat      ¦ [%]      ¦  Differentiell    ¦ Differentielle Anregung
%  long     ¦ [%]      ¦  Proportionall    ¦ Parallele Anregung

%  -----------------------------------------------
%  Kanal    ¦ PWM - Kanal ¦  Funktion            ¦
%  -----------------------------------------------
%  1        ¦ RC_Ch0      ¦  Heck Servo          ¦
%  4        ¦ RC_Ch3      ¦  Bremsleine links    ¦
%  5        ¦ RC_Ch4      ¦  Bremsleine rechts   ¦
%  7        ¦ RC_Ch6      ¦  Motor links         ¦
%  8        ¦ RC_Ch7      ¦  Motor rechts        ¦

%% 2)   Abtastzeit definieren
% Abtastfrequenz vom Px4 Board : 50Hz
% Abtastfrequenz vom Xsens     : 50Hz
Ts = 1 / 100;
figure(1);

% Anfang und Schluss wegschneiden
% Länge der Daten: Excel öffnen und Zeilen * Ts rechnen.
StartZeit = 50240;
EndZeit   = 91640;

%% 3)   Einlesen der Daten (Nur das erste Mal nötig)

% % Variablen initialisieren
% filename = 'flug2.csv';
% delimiter = ';';
% startRow = 2;
% %formatSpec = '%f%f%f%f%f%f%f%f%s%s%s%s%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';
% formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';
% % File öffnen
% fileID = fopen(filename,'r');
%
% % Kollonenen lesen
% %dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,0,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
% %dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,NaN, 'ReturnOnError', false);
% dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'EmptyValue' ,0,'HeaderLines' ,startRow-1, 'ReturnOnError', false);
%
% % Textfile schliessen
% fclose(fileID);

% ************************************************************************
% SETTINGS
% ************************************************************************

% Set the path to your sysvector.bin file here
filePath = 'log001.bin';

% Set the minimum and maximum times to plot here [in seconds]
mintime=0;        %The minimum time/timestamp to display, as set by the user [0 for first element / start]
maxtime=0;        %The maximum time/timestamp to display, as set by the user [0 for last element / end]

%conversion factors
fconv_timestamp=1E-6; % [microseconds] to [seconds]

% ************************************************************************
% Import the PX4 logs
% ************************************************************************
% ************************************************************************
% RETRIEVE SYSTEM VECTOR
% *************************************************************************
% //All measurements in NED frame

% Convert to CSV
%arg1 = 'log-fx61-20130721-2.bin';
arg1 = filePath;
delim = ',';
time_field = 'TIME';
data_file = 'data.csv';
csv_null = '';

if not(exist(data_file, 'file'))
    s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -t"%s" -d"%s" -n"%s"', arg1, data_file, time_field, delim, csv_null) );
end

if exist(data_file, 'file')
    
    %data = csvread(data_file);
    sysvector = tdfread(data_file, ',');
    
    % shot the flight time
    time_us = sysvector.TIME_StartTime(end) - sysvector.TIME_StartTime(2);
    time_s = uint64(time_us*1e-6);
    time_m = uint64(time_s/60);
    time_s = time_s - time_m * 60;
    
    disp([sprintf('Flight log duration: %d:%d (minutes:seconds)', time_m, time_s) char(10)]);
    
    disp(['logfile conversion finished.' char(10)]);
else
    disp(['file: ' data_file ' does not exist' char(10)]);
end

%Translate min and max plot times to indices
time=double(sysvector.TIME_StartTime) .*fconv_timestamp;
mintime_log=time(1);        %The minimum time/timestamp found in the log
maxtime_log=time(end);      %The maximum time/timestamp found in the log
CurTime=mintime_log;        %The current time at which to draw the aircraft position

% ************************************************************************
%  FINDMINMAXINDICES
%  ************************************************************************
    for i=1:size(sysvector.TIME_StartTime,1)
        if time(i)>=mintime; idxmin=i; break; end
    end
    for i=1:size(sysvector.TIME_StartTime,1)
        if maxtime==0; idxmax=size(sysvector.TIME_StartTime,1); break; end
        if time(i)>=maxtime; idxmax=i; break; end
    end
    mintime=time(idxmin); 
    imintime = idxmin;
    maxtime=time(idxmax);
    imaxtime = idxmax;

% Datenarray erstellen anhand der Headers
% und sogleich Anfang und Ende wegschneiden
TIME_StartTime = sysvector.TIME_StartTime(imintime:imaxtime);
% ATT_Time = 0;
% ATT_Roll = dataArray{:, 3}(StartZeit:EndZeit,1);
% ATT_Pitch = dataArray{:, 4}(StartZeit:EndZeit,1);
% ATT_Yaw = dataArray{:, 5}(StartZeit:EndZeit,1);
% ATT_RollRate = dataArray{:, 6}(StartZeit:EndZeit,1);
% ATT_PitchRate = dataArray{:, 7}(StartZeit:EndZeit,1);
% ATT_YawRate = dataArray{:, 8}(StartZeit:EndZeit,1);
% ATSP_Time = dataArray{:, 9}(StartZeit:EndZeit,1);
% ATSP_RollSP = dataArray{:, 10}(StartZeit:EndZeit,1);
% ATSP_PitchSP = dataArray{:, 11}(StartZeit:EndZeit,1);
% ATSP_YawSP = dataArray{:, 12}(StartZeit:EndZeit,1);
% ATSP_ThrustSP = dataArray{:, 13}(StartZeit:EndZeit,1);
% IMU_Time = dataArray{:, 14}(StartZeit:EndZeit,1);
IMU_AccX = sysvector.IMU_AccX(imintime:imaxtime);
IMU_AccY = sysvector.IMU_AccY(imintime:imaxtime);
IMU_AccZ = sysvector.IMU_AccZ(imintime:imaxtime);
% IMU_GyroX = dataArray{:, 18}(StartZeit:EndZeit,1);
% IMU_GyroY = dataArray{:, 19}(StartZeit:EndZeit,1);
% IMU_GyroZ = dataArray{:, 20}(StartZeit:EndZeit,1);
% IMU_MagX = dataArray{:, 21}(StartZeit:EndZeit,1);
% IMU_MagY = dataArray{:, 22}(StartZeit:EndZeit,1);
% IMU_MagZ = dataArray{:, 23}(StartZeit:EndZeit,1);
% SENS_BaroPres = dataArray{:, 24}(StartZeit:EndZeit,1);
% SENS_BaroAlt = dataArray{:, 25}(StartZeit:EndZeit,1);
% SENS_BaroTemp = dataArray{:, 26}(StartZeit:EndZeit,1);
% SENS_DiffPres = dataArray{:, 27}(StartZeit:EndZeit,1);
% LPOS_Time = dataArray{:, 28}(StartZeit:EndZeit,1);
% LPOS_X = dataArray{:, 29}(StartZeit:EndZeit,1);
% LPOS_Y = dataArray{:, 30}(StartZeit:EndZeit,1);
% LPOS_Z = dataArray{:, 31}(StartZeit:EndZeit,1);
% LPOS_VX = dataArray{:, 32}(StartZeit:EndZeit,1);
% LPOS_VY = dataArray{:, 33}(StartZeit:EndZeit,1);
% LPOS_VZ = dataArray{:, 34}(StartZeit:EndZeit,1);
% LPOS_RefLat = dataArray{:, 35}(StartZeit:EndZeit,1);
% LPOS_RefLon = dataArray{:, 36}(StartZeit:EndZeit,1);
% LPOS_RefAlt = dataArray{:, 37}(StartZeit:EndZeit,1);
% LPOS_XYFlags = dataArray{:, 38}(StartZeit:EndZeit,1);
% LPOS_ZFlags = dataArray{:, 39}(StartZeit:EndZeit,1);
% LPOS_Landed = dataArray{:, 40}(StartZeit:EndZeit,1);
% LPSP_X = dataArray{:, 41}(StartZeit:EndZeit,1);
% LPSP_Y = dataArray{:, 42}(StartZeit:EndZeit,1);
% LPSP_Z = dataArray{:, 43}(StartZeit:EndZeit,1);
% LPSP_Yaw = dataArray{:, 44}(StartZeit:EndZeit,1);
% GPS_GPS_t = dataArray{:, 45}(StartZeit:EndZeit,1);
% GPS_FixTyp = dataArray{:, 46}(StartZeit:EndZeit,1);
% GPS_EPH = dataArray{:, 47}(StartZeit:EndZeit,1);
% GPS_EPV = dataArray{:, 48}(StartZeit:EndZeit,1);
% GPS_POS_t = dataArray{:, 49}(StartZeit:EndZeit,1);
% GPS_Lat = dataArray{:, 50}(StartZeit:EndZeit,1);
% GPS_Lon = dataArray{:, 51}(StartZeit:EndZeit,1);
% GPS_Alt = dataArray{:, 52}(StartZeit:EndZeit,1);
% GPS_VEL_t = dataArray{:, 53}(StartZeit:EndZeit,1);
% GPS_VelN = dataArray{:, 54}(StartZeit:EndZeit,1);
% GPS_VelE = dataArray{:, 55}(StartZeit:EndZeit,1);
% GPS_VelD = dataArray{:, 56}(StartZeit:EndZeit,1);
% GPS_Cog = dataArray{:, 57}(StartZeit:EndZeit,1);
% ATTC_Time = dataArray{:, 58}(StartZeit:EndZeit,1);
% ATTC_Roll = dataArray{:, 59}(StartZeit:EndZeit,1);
% ATTC_Pitch = dataArray{:, 60}(StartZeit:EndZeit,1);
% ATTC_Yaw = dataArray{:, 61}(StartZeit:EndZeit,1);
% ATTC_Thrust = dataArray{:, 62}(StartZeit:EndZeit,1);
% STAT_Time = dataArray{:, 63}(StartZeit:EndZeit,1);
% STAT_MainState = dataArray{:, 64}(StartZeit:EndZeit,1);
% STAT_NavState = dataArray{:, 65}(StartZeit:EndZeit,1);
% STAT_ArmState = dataArray{:, 66}(StartZeit:EndZeit,1);
% STAT_BatV = dataArray{:, 67}(StartZeit:EndZeit,1);
% STAT_BatC = dataArray{:, 68}(StartZeit:EndZeit,1);
% STAT_BatRem = dataArray{:, 69}(StartZeit:EndZeit,1);
% STAT_BatWarn = dataArray{:, 70}(StartZeit:EndZeit,1);
% STAT_Landed = dataArray{:, 71}(StartZeit:EndZeit,1);
% RC_Time = dataArray{:, 72}(StartZeit:EndZeit,1);
% RC_Ch0 = dataArray{:, 73}(StartZeit:EndZeit,1);
% RC_Ch1 = dataArray{:, 74}(StartZeit:EndZeit,1);
% RC_Ch2 = dataArray{:, 75}(StartZeit:EndZeit,1);
% RC_Ch3 = dataArray{:, 76}(StartZeit:EndZeit,1);
% RC_Ch4 = dataArray{:, 77}(StartZeit:EndZeit,1);
% RC_Ch5 = dataArray{:, 78}(StartZeit:EndZeit,1);
% RC_Ch6 = dataArray{:, 79}(StartZeit:EndZeit,1);
% RC_Ch7 = dataArray{:, 80}(StartZeit:EndZeit,1);
% OUT0_TimeOut0 = dataArray{:, 81}(StartZeit:EndZeit,1);
% OUT0_Out1 = dataArray{:, 82}(StartZeit:EndZeit,1);
% OUT0_Out2 = dataArray{:, 83}(StartZeit:EndZeit,1);
% OUT0_Out3 = dataArray{:, 84}(StartZeit:EndZeit,1);
% OUT0_Out4 = dataArray{:, 85}(StartZeit:EndZeit,1);
% OUT0_Out5 = dataArray{:, 86}(StartZeit:EndZeit,1);
% OUT0_Out6 = dataArray{:, 87}(StartZeit:EndZeit,1);
% OUT0_Out7 = dataArray{:, 88}(StartZeit:EndZeit,1);
% AIRS_Time = dataArray{:, 89}(StartZeit:EndZeit,1);
% AIRS_IndSpeed = dataArray{:, 90}(StartZeit:EndZeit,1);
% AIRS_TrueSpeed = dataArray{:, 91}(StartZeit:EndZeit,1);
% ARSP_Time = dataArray{:, 92}(StartZeit:EndZeit,1);
% ARSP_RollRateSP = dataArray{:, 93}(StartZeit:EndZeit,1);
% ARSP_PitchRateSP = dataArray{:, 94}(StartZeit:EndZeit,1);
% ARSP_YawRateSP = dataArray{:, 95}(StartZeit:EndZeit,1);
% FLOW_Time = dataArray{:, 96}(StartZeit:EndZeit,1);
% FLOW_RawX = dataArray{:, 97}(StartZeit:EndZeit,1);
% FLOW_RawY = dataArray{:, 98}(StartZeit:EndZeit,1);
% FLOW_CompX = dataArray{:, 99}(StartZeit:EndZeit,1);
% FLOW_CompY = dataArray{:, 100}(StartZeit:EndZeit,1);
% FLOW_Dist = dataArray{:, 101}(StartZeit:EndZeit,1);
% FLOW_Q = dataArray{:, 102}(StartZeit:EndZeit,1);
% FLOW_SensID = dataArray{:, 103}(StartZeit:EndZeit,1);
% GPOS_Time = dataArray{:, 104}(StartZeit:EndZeit,1);
% GPOS_Lat = dataArray{:, 105}(StartZeit:EndZeit,1);
% GPOS_Lon = dataArray{:, 106}(StartZeit:EndZeit,1);
% GPOS_Alt = dataArray{:, 107}(StartZeit:EndZeit,1);
% GPOS_VelN = dataArray{:, 108}(StartZeit:EndZeit,1);
% GPOS_VelE = dataArray{:, 109}(StartZeit:EndZeit,1);
% GPOS_VelD = dataArray{:, 110}(StartZeit:EndZeit,1);
% GPSP_AltRel = dataArray{:, 111}(StartZeit:EndZeit,1);
% GPSP_Lat = dataArray{:, 112}(StartZeit:EndZeit,1);
% GPSP_Lon = dataArray{:, 113}(StartZeit:EndZeit,1);
% GPSP_Alt = dataArray{:, 114}(StartZeit:EndZeit,1);
% GPSP_Yaw = dataArray{:, 115}(StartZeit:EndZeit,1);
% GPSP_LoiterR = dataArray{:, 116}(StartZeit:EndZeit,1);
% GPSP_LoiterDir = dataArray{:, 117}(StartZeit:EndZeit,1);
% GPSP_NavCmd = dataArray{:, 118}(StartZeit:EndZeit,1);
% GPSP_P1 = dataArray{:, 119}(StartZeit:EndZeit,1);
% GPSP_P2 = dataArray{:, 120}(StartZeit:EndZeit,1);
% GPSP_P3 = dataArray{:, 121}(StartZeit:EndZeit,1);
% GPSP_P4 = dataArray{:, 122}(StartZeit:EndZeit,1);
% ESC_Time = dataArray{:, 123}(StartZeit:EndZeit,1);
% ESC_Cnt = dataArray{:, 124}(StartZeit:EndZeit,1);
% ESC_NumESC = dataArray{:, 125}(StartZeit:EndZeit,1);
% ESC_Conn = dataArray{:, 126}(StartZeit:EndZeit,1);
% ESC_N = dataArray{:, 127}(StartZeit:EndZeit,1);
% ESC_Ver = dataArray{:, 128}(StartZeit:EndZeit,1);
% ESC_Adr = dataArray{:, 129}(StartZeit:EndZeit,1);
% ESC_Volt = dataArray{:, 130}(StartZeit:EndZeit,1);
% ESC_Amp = dataArray{:, 131}(StartZeit:EndZeit,1);
% ESC_RPM = dataArray{:, 132}(StartZeit:EndZeit,1);
% ESC_Temp = dataArray{:, 133}(StartZeit:EndZeit,1);
% ESC_SetP = dataArray{:, 134}(StartZeit:EndZeit,1);
% ESC_SetPRAW = dataArray{:, 135}(StartZeit:EndZeit,1);
% GVSP_VX = dataArray{:, 136}(StartZeit:EndZeit,1);
% GVSP_VY = dataArray{:, 137}(StartZeit:EndZeit,1);
% GVSP_VZ = dataArray{:, 138}(StartZeit:EndZeit,1);
% XIMU_Time = dataArray{:, 139}(StartZeit:EndZeit,1);
% XIMU_AccX = dataArray{:, 140}(StartZeit:EndZeit,1);
% XIMU_AccY = dataArray{:, 141}(StartZeit:EndZeit,1);
% XIMU_AccZ = dataArray{:, 142}(StartZeit:EndZeit,1);
% XIMU_GyroX = dataArray{:, 143}(StartZeit:EndZeit,1);
% XIMU_GyroY = dataArray{:, 144}(StartZeit:EndZeit,1);
% XIMU_GyroZ = dataArray{:, 145}(StartZeit:EndZeit,1);
% XIMU_MagX = dataArray{:, 146}(StartZeit:EndZeit,1);
% XIMU_MagY = dataArray{:, 147}(StartZeit:EndZeit,1);
% XIMU_MagZ = dataArray{:, 148}(StartZeit:EndZeit,1);
% XSEN_BaroPres = dataArray{:, 149}(StartZeit:EndZeit,1);
% XSEN_BaroAlt = dataArray{:, 150}(StartZeit:EndZeit,1);
% XSEN_BaroTemp = dataArray{:, 151}(StartZeit:EndZeit,1);
% XSEN_DiffPres = dataArray{:, 152}(StartZeit:EndZeit,1);
% XGPS_GPS_t = dataArray{:, 153}(StartZeit:EndZeit,1);
% XGPS_FixTyp = dataArray{:, 154}(StartZeit:EndZeit,1);
% XGPS_EPH = dataArray{:, 155}(StartZeit:EndZeit,1);
% XGPS_EPV = dataArray{:, 156}(StartZeit:EndZeit,1);
% XGPS_POS_t = dataArray{:, 157}(StartZeit:EndZeit,1);
% XGPS_Lat = dataArray{:, 158}(StartZeit:EndZeit,1);
% XGPS_Lon = dataArray{:, 159}(StartZeit:EndZeit,1);
% XGPS_Alt = dataArray{:, 160}(StartZeit:EndZeit,1);
% XGPS_VEL_t = dataArray{:, 161}(StartZeit:EndZeit,1);
% XGPS_VelN = dataArray{:, 162}(StartZeit:EndZeit,1);
% XGPS_VelE = dataArray{:, 163}(StartZeit:EndZeit,1);
% XGPS_VelD = dataArray{:, 164}(StartZeit:EndZeit,1);
% XGPS_Cog = dataArray{:, 165}(StartZeit:EndZeit,1);
% XATT_Time = dataArray{:, 166}(StartZeit:EndZeit,1);
% XATT_Roll = dataArray{:, 167}(StartZeit:EndZeit,1);
% XATT_Pitch = dataArray{:, 168}(StartZeit:EndZeit,1);
% XATT_Yaw = dataArray{:, 169}(StartZeit:EndZeit,1);
% XATT_RollRate = dataArray{:, 170}(StartZeit:EndZeit,1);
% XATT_PitchRate = dataArray{:, 171}(StartZeit:EndZeit,1);
% XATT_YawRate = dataArray{:, 172}(StartZeit:EndZeit,1);
% XGPO_Time = dataArray{:, 173}(StartZeit:EndZeit,1);
% XGPO_Lat = dataArray{:, 174}(StartZeit:EndZeit,1);
% XGPO_Lon = dataArray{:, 175}(StartZeit:EndZeit,1);
% XGPO_Alt = dataArray{:, 176}(StartZeit:EndZeit,1);
% XGPO_VelN = dataArray{:, 177}(StartZeit:EndZeit,1);
% XGPO_VelE = dataArray{:, 178}(StartZeit:EndZeit,1);
% XGPO_VelD = dataArray{:, 179}(StartZeit:EndZeit,1);
% RANG_Time = dataArray{:, 180}(StartZeit:EndZeit,1);
% RANG_Ang_l = dataArray{:, 181}(StartZeit:EndZeit,1);
% RANG_Ang_r = dataArray{:, 182}(StartZeit:EndZeit,1);
% FWRV_c96a81565ef959731a97 = dataArray{:, 183}(StartZeit:EndZeit,1);

% Temporäre Variablen löschen
% clearvars filename delimiter startRow formatSpec fileID dataArray ans;

%% ------------------------------------------------------------------------
%% 4)   Datensatz skalieren

% Heck Servo
RC_Ch0_scaled = RC_Ch0;

if (1==1)
    RC_Ch3_scaled = RC_Ch4 - 0.096;
    RC_Ch4_scaled = RC_Ch4 - 0.096;
    RC_Ch3_Range    = 0.6;
    RC_Ch4_Range    = 0.6;
    RC_Ch3_scaled = RC_Ch3_scaled  / RC_Ch3_Range * 2 ;
    RC_Ch4_scaled = RC_Ch4_scaled  / RC_Ch4_Range * 2 * (-1);
end

% Motoren auf 1 skalieren (-1 = 0% Speed)
% Bereich der minimalen und maximalen Auslenkung der Motoren
RC_Ch6_Range    = max(RC_Ch6)-min(RC_Ch6);  % 1.334
RC_Ch7_Range    = max(RC_Ch7)-min(RC_Ch7);  % 1.332
% Nun müssen gewisse Offsets einbeechnet werden.
RC_Ch6_scaled   = RC_Ch6 - (max(RC_Ch6)-(RC_Ch6_Range/2));
RC_Ch7_scaled   = RC_Ch7 - (max(RC_Ch7)-(RC_Ch7_Range/2));
% Motor links
RC_Ch6_scaled = RC_Ch6_scaled  / RC_Ch6_Range * 2 ;
% Motor rechts
RC_Ch7_scaled = RC_Ch7_scaled  / RC_Ch7_Range * 2 ;

% Höhenlagen korrigieren (Allmend = 437müM)
XGPS_Alt_scaled     = XGPS_Alt     + (437-min(XGPS_Alt));
XGPO_Alt_scaled     = XGPO_Alt     + (437-XGPO_Alt(1));
GPOS_Alt_scaled     = GPOS_Alt     + (437-min(GPOS_Alt));
SENS_BaroAlt_scaled = SENS_BaroAlt + (437-min(SENS_BaroAlt));
XSEN_BaroAlt_scaled = XSEN_BaroAlt + (437-min(XSEN_BaroAlt));

% Barodaten von mbar Pa umrechnen
SENS_BaroPres = SENS_BaroPres*100;
XSEN_BaroPres = XSEN_BaroPres*100;

% Yaw - Winkel Anpassung, weil der Winkel von 2pi auf 0 springt
ATT_Yaw_scaled = unwrap(ATT_Yaw);
XATT_Yaw_scaled = unwrap(XATT_Yaw);

% Roll - Winkel Anpassung, weil der Winkel von 2pi auf 0 springt
ATT_Roll_scaled = unwrap(ATT_Roll);
XATT_Roll_scaled = unwrap(XATT_Roll);

% Yaw Winkel auf Startwert von 0 setzen
ATT_Yaw_scaled  = ATT_Yaw_scaled  - 0.11;
XATT_Yaw_scaled = XATT_Yaw_scaled - 0.085;

%% 5)   Datensatz interpolieren

% Zeitvektor vom Px4 Board wird in us geloggt. --> Sekunden machen
Time_Vektor = TIME_StartTime*10^-6;
% Vektor so skalieren, dass der erste Wert bei 0 startet
Time = Time_Vektor - min(Time_Vektor);
% Zeitvektor mit 0.02s Schritte
Zeit = (0:Ts:max(Time))';

% -------------------------------------------------------------------------

% Px4 - Roll / Pitch / Yaw
data_Px4_ATT_Roll      = interp1(Time, ATT_Roll_scaled, Zeit, 'linear');
data_Px4_ATT_Pitch     = interp1(Time, ATT_Pitch, Zeit, 'linear');
data_Px4_ATT_Yaw       = interp1(Time, ATT_Yaw_scaled, Zeit, 'linear');
data_Px4_ATT_RollRate  = interp1(Time, ATT_RollRate, Zeit, 'linear');
data_Px4_ATT_PitchRate = interp1(Time, ATT_PitchRate, Zeit, 'linear');
data_Px4_ATT_YawRate   = interp1(Time, ATT_YawRate, Zeit, 'linear');

% -------------------------------------------------------------------------

% Px4 - Beschleunigungen (Acc)
data_Px4_AccX = interp1(Time, IMU_AccX, Zeit, 'linear');
data_Px4_AccY = interp1(Time, IMU_AccY, Zeit, 'linear');
data_Px4_AccZ = interp1(Time, IMU_AccZ, Zeit, 'linear');

% Px4 - Kreiselwerte (Gyro)
data_Px4_GyroX = interp1(Time, IMU_GyroX, Zeit, 'linear');
data_Px4_GyroY = interp1(Time, IMU_GyroY, Zeit, 'linear');
data_Px4_GyroZ = interp1(Time, IMU_GyroZ, Zeit, 'linear');

% Px4 - Lagedaten (von Magnetometer)
data_Px4_MagX = interp1(Time, IMU_MagX, Zeit, 'linear');
data_Px4_MagY = interp1(Time, IMU_MagY, Zeit, 'linear');
data_Px4_MagZ = interp1(Time, IMU_MagZ, Zeit, 'linear');

% Px4 - Baro - Daten
data_Px4_BaroPres = interp1(Time, SENS_BaroPres, Zeit, 'linear');
data_Px4_BaroAlt  = interp1(Time, SENS_BaroAlt_scaled, Zeit, 'linear');
data_Px4_BaroTemp = interp1(Time, SENS_BaroTemp, Zeit, 'linear');

% -------------------------------------------------------------------------

% Einänge bzw. Dutycycle in [%] interpolieren
data_In_Servo_Heck   = interp1(Time, RC_Ch0_scaled, Zeit, 'linear');
data_In_Servo_Links  = interp1(Time, RC_Ch3_scaled, Zeit, 'linear');
data_In_Servo_Rechts = interp1(Time, RC_Ch4_scaled, Zeit, 'linear');
data_In_Motor_Links  = interp1(Time, RC_Ch6_scaled, Zeit, 'linear');
data_In_Motor_Rechts = interp1(Time, RC_Ch7_scaled, Zeit, 'linear');

% -------------------------------------------------------------------------

% Px4 - Lagedaten (von GPS)
data_Px4_GPOS_Lat = interp1(Time, GPOS_Lat, Zeit, 'linear');
data_Px4_GPOS_Lon = interp1(Time, GPOS_Lon, Zeit, 'linear');
data_Px4_GPOS_Alt = interp1(Time, GPOS_Alt_scaled, Zeit, 'linear');

% Px4 - GPOS_VelD/E/N Geschwindigkeiten
data_Px4_GPOS_VelN = interp1(Time, GPOS_VelN, Zeit, 'linear');
data_Px4_GPOS_VelE = interp1(Time, GPOS_VelE, Zeit, 'linear');
data_Px4_GPOS_VelD = interp1(Time, GPOS_VelD, Zeit, 'linear');

% -------------------------------------------------------------------------

% XSENS - XIMU - Beschleunigungen (Acc)
data_Xsens_AccX = interp1(Time, XIMU_AccX, Zeit, 'linear');
data_Xsens_AccY = interp1(Time, XIMU_AccY, Zeit, 'linear');
data_Xsens_AccZ = interp1(Time, XIMU_AccZ, Zeit, 'linear');

% XSENS - XIMU - Kreiselbeschleunigungen (Gyro)
data_Xsens_GyroX = interp1(Time, XIMU_GyroX, Zeit, 'linear');
data_Xsens_GyroY = interp1(Time, XIMU_GyroY, Zeit, 'linear');
data_Xsens_GyroZ = interp1(Time, XIMU_GyroZ, Zeit, 'linear');

% XSENS - XIMU - Lagedaten (von Magnetometer)
data_Xsens_MagX = interp1(Time, XIMU_MagX, Zeit, 'linear');
data_Xsens_MagY = interp1(Time, XIMU_MagY, Zeit, 'linear');
data_Xsens_MagZ = interp1(Time, XIMU_MagZ, Zeit, 'linear');

% Xsens - Baro - Daten
data_Xsens_BaroPres = interp1(Time, XSEN_BaroPres, Zeit, 'linear');
data_Xsens_BaroAlt  = interp1(Time, XSEN_BaroAlt_scaled, Zeit, 'linear');
data_Xsens_BaroTemp = interp1(Time, XSEN_BaroTemp, Zeit, 'linear');

% -------------------------------------------------------------------------

% XSENS - Lagedaten (von GPS)
data_Xsens_XGPS_Lat = interp1(Time, XGPS_Lat, Zeit, 'linear');
data_Xsens_XGPS_Lon = interp1(Time, XGPS_Lon, Zeit, 'linear');
data_Xsens_XGPS_Alt = interp1(Time, XGPS_Alt_scaled, Zeit, 'linear');

% XSENS - GPS VeIN VeIE VeID
data_Xsens_XGPS_VelN = interp1(Time, XGPS_VelN, Zeit, 'linear');
data_Xsens_XGPS_VelE = interp1(Time, XGPS_VelE, Zeit, 'linear');
data_Xsens_XGPS_VelD = interp1(Time, XGPS_VelD, Zeit, 'linear');

% XSENS - Roll / Pitch / Yaw
data_Xsens_XATT_Roll  = interp1(Time, XATT_Roll_scaled,  Zeit, 'linear');
data_Xsens_XATT_Pitch = interp1(Time, XATT_Pitch, Zeit, 'linear');
data_Xsens_XATT_Yaw   = interp1(Time, XATT_Yaw_scaled,   Zeit, 'linear');

% XSENS - RollRate / PitchRate / YawRate
data_Xsens_XATT_RollRate  = interp1(Time, XATT_RollRate,  Zeit, 'linear');
data_Xsens_XATT_PitchRate = interp1(Time, XATT_PitchRate, Zeit, 'linear');
data_Xsens_XATT_YawRate   = interp1(Time, XATT_YawRate,   Zeit, 'linear');

% -------------------------------------------------------------------------

% XSENS - Positionsdaten XGPO_Lat
data_Xsens_XGPO_Lat = interp1(Time, XGPO_Lat, Zeit, 'linear');
data_Xsens_XGPO_Lon = interp1(Time, XGPO_Lon, Zeit, 'linear');
data_Xsens_XGPO_Alt = interp1(Time, XGPO_Alt_scaled, Zeit, 'linear');

% XSENS - Geschwindigkeiten XGPO_VeIN....
data_Xsens_XGPO_VelN = interp1(Time, XGPO_VelN, Zeit, 'linear');
data_Xsens_XGPO_VelE = interp1(Time, XGPO_VelE, Zeit, 'linear');
data_Xsens_XGPO_VelD = interp1(Time, XGPO_VelD, Zeit, 'linear');

% -------------------------------------------------------------------------

% Poti - Relativer Winkel zwischen Schirm und Paraglider
data_Poti_Links  = interp1(Time, RANG_Ang_l, Zeit, 'linear');
data_Poti_Rechts = interp1(Time, RANG_Ang_r, Zeit, 'linear');

%% 6)   Alle "NAN" Werte mit 0 ersetzen.
data_Xsens_XGPO_VelN(find(isnan(data_Xsens_XGPO_VelN))) = 0;
data_Xsens_XGPO_VelE(find(isnan(data_Xsens_XGPO_VelE))) = 0;
data_Xsens_XGPO_VelD(find(isnan(data_Xsens_XGPO_VelD))) = 0;
data_Xsens_XGPS_Lon(find(isnan(data_Xsens_XGPS_Lon))) = 0;
data_Xsens_XGPS_Lat(find(isnan(data_Xsens_XGPS_Lat))) = 0;
data_Xsens_XGPS_Alt(find(isnan(data_Xsens_XGPS_Alt))) = 0;
data_Xsens_XGPO_Lon(find(isnan(data_Xsens_XGPO_Lon))) = 0;
data_Xsens_XGPO_Lat(find(isnan(data_Xsens_XGPO_Lat))) = 0;
data_Xsens_XGPO_Alt(find(isnan(data_Xsens_XGPO_Alt))) = 0;
data_Xsens_XATT_Yaw(find(isnan(data_Xsens_XATT_Yaw))) = 0;
data_Xsens_XATT_Roll(find(isnan(data_Xsens_XATT_Roll))) = 0;
data_Xsens_XATT_Pitch(find(isnan(data_Xsens_XATT_Pitch))) = 0;
data_Xsens_MagZ(find(isnan(data_Xsens_MagZ))) = 0;
data_Xsens_MagY(find(isnan(data_Xsens_MagY))) = 0;
data_Xsens_MagX(find(isnan(data_Xsens_MagX))) = 0;
data_Xsens_GyroZ(find(isnan(data_Xsens_GyroZ))) = 0;
data_Xsens_GyroY(find(isnan(data_Xsens_GyroY))) = 0;
data_Xsens_GyroX(find(isnan(data_Xsens_GyroX))) = 0;
data_Xsens_AccZ(find(isnan(data_Xsens_AccZ))) = 0;
data_Xsens_AccY(find(isnan(data_Xsens_AccY))) = 0;
data_Xsens_AccX(find(isnan(data_Xsens_AccX))) = 0;
data_Px4_MagZ(find(isnan(data_Px4_MagZ))) = 0;
data_Px4_MagY(find(isnan(data_Px4_MagY))) = 0;
data_Px4_MagX(find(isnan(data_Px4_MagX))) = 0;
data_Px4_GyroZ(find(isnan(data_Px4_GyroZ))) = 0;
data_Px4_GyroY(find(isnan(data_Px4_GyroY))) = 0;
data_Px4_GyroX(find(isnan(data_Px4_GyroX))) = 0;
data_Px4_BaroTemp(find(isnan(data_Px4_BaroTemp))) = 0;
data_Px4_BaroAlt(find(isnan(data_Px4_BaroAlt))) = 0;
data_Px4_AccZ(find(isnan(data_Px4_AccZ))) = 0;
data_Px4_AccY(find(isnan(data_Px4_AccY))) = 0;
data_Px4_AccX(find(isnan(data_Px4_AccX))) = 0;
data_Px4_ATT_YawRate(find(isnan(data_Px4_ATT_YawRate))) = 0;
data_Px4_ATT_Yaw(find(isnan(data_Px4_ATT_Yaw))) = 0;
data_Px4_ATT_RollRate(find(isnan(data_Px4_ATT_RollRate))) = 0;
data_Px4_ATT_Roll(find(isnan(data_Px4_ATT_Roll))) = 0;
data_Px4_ATT_PitchRate(find(isnan(data_Px4_ATT_PitchRate))) = 0;
data_Px4_ATT_Pitch(find(isnan(data_Px4_ATT_Pitch))) = 0;
data_In_Servo_Rechts(find(isnan(data_In_Servo_Rechts))) = 0;
data_In_Servo_Links(find(isnan(data_In_Servo_Links))) = 0;
data_In_Servo_Heck(find(isnan(data_In_Servo_Heck))) = 0;
data_In_Motor_Rechts(find(isnan(data_In_Motor_Rechts))) = 0;
data_In_Motor_Links(find(isnan(data_In_Motor_Links))) = 0;

%% ------------------------------------------------------------------------
%% 7)   Aus PWM Eingänge die Steuereingänge mit Mixer umrechnen

% M * a = s    // a = Aktuatoren, M = Mixmatrix, s = Steuermatrix

%         1/2  ¦  -1/2
%  M =    - - - - - - -
%         1/2  ¦   1/2
M = [1/2 -1/2; 1/2 1/2];

% Steuereingänge der Servos berechnen
a = [data_In_Servo_Links data_In_Servo_Rechts];
s = M *a';
S_Servo_Differentiel = s(1,:)';
S_Servo_Parallel = s(2,:)';

% Steuereingänge der Motoren berechnen
a = [data_In_Motor_Links data_In_Motor_Rechts];
s = M *a';
S_Motor_Differentiel = s(1,:)';
S_Motor_Parallel = s(2,:)';

% Steuerstellung Potis
a = [data_Poti_Links data_Poti_Rechts];
s = M *a';
S_Poti_Differentiel = s(1,:)';
S_Poti_Parallel = s(2,:)';

%% 8)   Mit Potentiometerstellung die Drehwinkel korrigeren

% Pitch korrigeren mit Hilfe der longitudinalen Potistellung
data_Xsens_XATT_Pitch_PotiCorrected = data_Xsens_XATT_Pitch - S_Poti_Parallel - S_Poti_Differentiel;
%Yaw korrigeren mit Hilfe der lateralen Potistellung
data_Xsens_XATT_Yaw_PotiCorrected = data_Xsens_XATT_Yaw - S_Poti_Differentiel;

if (1==2)
    %% 9)   Aus AccX,AccY,AccZ --> Vx,Vy,Vz berechnen
    % Das Level vom Px4 - AccZ ist bei -10.
    data_Px4_AccZ = (data_Px4_AccZ + 10)*8;
    data_Px4_AccY = data_Px4_AccY*8;
    
    data_Xsens_vx_calculated = (cumsum(data_Xsens_AccX)*Ts);
    data_Xsens_vy_calculated = (cumsum(data_Xsens_AccY)*Ts);
    data_Xsens_vz_calculated = (cumsum(data_Xsens_AccZ)*Ts);
    data_Px4_vx_calculated   = (cumsum(data_Px4_AccX)*Ts);
    data_Px4_vy_calculated   = (cumsum(data_Px4_AccY)*Ts);
    data_Px4_vz_calculated   = (cumsum(data_Px4_AccZ)*Ts);
    
    %% 10) Aus GyroX... --> Pitch... berechnen
    data_Xsens_Roll_calculated  = (cumsum(data_Xsens_GyroX)*Ts);
    data_Xsens_Pitch_calculated = (cumsum(data_Xsens_GyroY)*Ts);
    data_Xsens_Yaw_calculated   = (cumsum(data_Xsens_GyroZ)*Ts);
    data_Px4_Roll_calculated    = (cumsum(data_Px4_GyroX)*Ts);
    data_Px4_Pitch_calculated   = (cumsum(data_Px4_GyroY)*Ts)/100; % weil px4 daten unbrauchbar!!
    data_Px4_Yaw_calculated     = (cumsum(data_Px4_GyroZ)*Ts);
    
    % Mit "detrend()" der Wegdrift minimieren
    data            = data_Xsens_Pitch_calculated(25000:27500,1)-34;
    date_tempOrig   = data_Xsens_XATT_Pitch(25000:27500,1);
    detrend_sdata   = detrend(data);
    trend           = data - detrend_sdata;
    tt              = (Ts:Ts:length(trend)*Ts)';
    
    figure(1)
    subplot(1,1,1);
    plot(tt, [data, date_tempOrig, trend, detrend_sdata])
    legend('Berechneter Pitchwinkel aus der Pitchrate', 'Effektiv gemessener Pitchwinkel', 'Detrend - Linie', 'Trend (korrigierter Verlauf)'); grid on;; title('Vergleich der gemessenen Winkel und den integrierten Winkelraten');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
end

%% 11) Absolute Fahrgeschwindigkeit berechnen
data_V_absolut = sqrt(data_Xsens_XGPS_VelN.^2 + data_Xsens_XGPS_VelE.^2);
% Maximale Geschwindigkeit liegt bei 17m/s = 61km/h
max(data_V_absolut) * 3.6;

%% 12) Datensatz in .mat - Datei abspeichern
save('data.mat');

%% ------------------------------------------------------------------------
%% 13) FlightGear berechnungen
if (1==2)
    % Vektorlänge bestimmen
    VektorLaenge = length(Zeit);
    Nullvektor   = (1:length(Zeit))';
    Nullvektor(1:length(Zeit)) = 0;
    
    % Vektorenstücke aus dem ganzen Vektor herausschneiden
    % Diferenz Bremsleine: 12000-15000
    % Wähle aus, welcher Modus zu fliegen im Simulator
    choice = 5;
    
    % Startsequenz
    if (choice==1);
        Startvalue  = 10;
        EndValue    = 10000;
        
        % Differentielles Leinenziehen
    elseif (choice==2)
        Startvalue  = 15200;
        EndValue    = 39380;
        
        % Motorschub
    elseif (choice==3)
        Startvalue  = 25000;
        EndValue    = 30000;
        
        % Verifizierungs turn
    elseif (choice==4)
        Startvalue  = 32200;
        EndValue    = 39380;
        
        % Landen
    elseif (choice==5)
        Startvalue  = 1;
        EndValue    = 53000;
        
    end
    
    % Vektoren füllen und zuschneiden (Xsens oder Px4 --> Egal)
    FlightGear_Lon       = data_Xsens_XGPS_Lon(Startvalue:EndValue,1);
    FlightGear_Lat       = data_Xsens_XGPS_Lat(Startvalue:EndValue,1);
    FlightGear_Alt       = data_Xsens_XGPS_Alt(Startvalue:EndValue,1);
    FlightGear_RollRate  = data_Px4_ATT_RollRate(Startvalue:EndValue,1);
    FlightGear_PitchRate = data_Px4_ATT_PitchRate(Startvalue:EndValue,1);
    FlightGear_YawRate   = data_Px4_ATT_YawRate(Startvalue:EndValue,1);
    FlightGear_Roll      = data_Px4_ATT_Roll(Startvalue:EndValue,1);
    FlightGear_Pitch     = data_Px4_ATT_Pitch(Startvalue:EndValue,1);
    FlightGear_Yaw       = data_Px4_ATT_Yaw(Startvalue:EndValue,1);
    
    % Da zuviele Punkte vorhanden sind, ist der Flight Gear zu langsam.
    Tab = 50;  % Abtastpunkte, weil 100Hz
    
    FlightGear_Lon        = FlightGear_Lon(1:Tab:length(FlightGear_Lon));
    FlightGear_Lat        = FlightGear_Lat(1:Tab:length(FlightGear_Lat));
    FlightGear_Alt        = FlightGear_Alt(1:Tab:length(FlightGear_Alt));
    FlightGear_RollRate   = FlightGear_RollRate(1:Tab:length(FlightGear_RollRate));
    FlightGear_PitchRate  = FlightGear_PitchRate(1:Tab:length(FlightGear_PitchRate));
    FlightGear_YawRate    = FlightGear_YawRate(1:Tab:length(FlightGear_YawRate));
    FlightGear_Roll       = FlightGear_Roll(1:Tab:length(FlightGear_Roll));
    FlightGear_Pitch      = FlightGear_Pitch(1:Tab:length(FlightGear_Pitch));
    FlightGear_Yaw        = FlightGear_Yaw(1:Tab:length(FlightGear_Yaw));
    
    FlightGear_Zeit = (1:1:length(FlightGear_Lon))';
    
    % Vektoren so anpassen, dass sie auf dem KSFO - Flughafen passen
    % Lat = 37.629, Lon = -122.394 --> Shift/v = CaseView
    
    FlightGear_Lat = FlightGear_Lat - 9.4022;
    FlightGear_Lon = FlightGear_Lon - 130.6951;
    FlightGear_Alt = FlightGear_Alt - 429;
    FlightGear_Yaw = FlightGear_Yaw + 0;
    
    % Vektoren füllen
    l     = [FlightGear_Zeit FlightGear_Lon];
    u     = [FlightGear_Zeit FlightGear_Lat];
    h     = [FlightGear_Zeit FlightGear_Alt];
    p     = [FlightGear_Zeit FlightGear_RollRate];
    q     = [FlightGear_Zeit FlightGear_PitchRate];
    r     = [FlightGear_Zeit FlightGear_YawRate];
    phi   = [FlightGear_Zeit FlightGear_Roll];
    theta = [FlightGear_Zeit FlightGear_Pitch];
    psi   = [FlightGear_Zeit FlightGear_Yaw];
end

%% 14) Simulinkmodell FlightGear Simulator starten
if (1==2);
    % Run Simulink Modell
    run 'FlightGear.mdl'
    
    % Run FlightGear
    dos('runfg &');
end;

%% ------------------------------------------------------------------------
%% 15) Daten zwischen Xsens und Px4 auswählen und beste Sequenzen schneiden

% Steuereingänge Servo Differentiell und Parallel
CIFER_S_Servo_Differentiell  = S_Servo_Differentiel;
CIFER_S_Servo_Parallel       = S_Servo_Parallel;

% Steuereingänge Motor Differentiell und Parallel
CIFER_S_Motor_Differentiell  = S_Motor_Differentiel;
CIFER_S_Motor_Parallel       = S_Motor_Parallel;

% % Pitch Winkel (Theta), Xsensdaten sind zu variable, Px4 Daten sind stabiler
% CIFER_data_Pitch        = data_Px4_ATT_Pitch;
% % Roll Winkel (Phi), Px4 und Xsens genau gleich gut
% CIFER_data_Roll         = data_Xsens_XATT_Roll;
% % Yaw Winkel (Psi), Px4 und Xsens fast gleich gut
% CIFER_data_Yaw          = data_Xsens_XATT_Yaw;
%
% % Pitch Winkel Geschwindigkeit q (GyroY = Pitchrate)
% CIFER_data_PitchRate      = data_Px4_ATT_PitchRate;
% % Roll Winkel Geschwindigkeit p (GyroX = Rollrate)
% CIFER_data_RollRate       = data_Px4_ATT_RollRate;
% % Yaw Winkel Geschwindigkeit r (GyroZ = Yawrate)
% CIFER_data_YawRate        = data_Px4_ATT_YawRate;

%TEST
% -------------------------------------------------------------------------
% Pitch Winkel (Theta), Xsensdaten sind zu variable, Px4 Daten sind stabiler
CIFER_data_Pitch        = data_Xsens_XATT_Pitch;
% Roll Winkel (Phi), Px4 und Xsens genau gleich gut
CIFER_data_Roll         = data_Xsens_XATT_Roll;
% Yaw Winkel (Psi), Px4 und Xsens fast gleich gut
CIFER_data_Yaw          = data_Xsens_XATT_Yaw;
% Pitch Winkel Geschwindigkeit q (GyroY = Pitchrate)
CIFER_data_PitchRate      = data_Xsens_GyroY;
% Roll Winkel Geschwindigkeit p (GyroX = Rollrate)
CIFER_data_RollRate       = data_Xsens_GyroX;
% Yaw Winkel Geschwindigkeit r (GyroZ = Yawrate=
CIFER_data_YawRate        = data_Xsens_GyroZ;
% -------------------------------------------------------------------------

% Höhe, Px4 Barodaten sow Xsens XGPS datan sind sehr gut, XGPO schlecht!!
CIFER_data_Hoehe        = data_Xsens_XGPS_Alt;

% Steig und Sinkrate
CIFER_data_vz        = data_Xsens_XGPS_VelD;

% Absolute Geschwindigkeit in North, East, Down
CIFER_data_Vabs         = data_V_absolut;

% Alles Ploten um die Daten zu verifizierne, befor sie ins CIFER gehen
subplot(4,1,1);
plot(Zeit, [CIFER_S_Servo_Differentiell, CIFER_S_Servo_Parallel,CIFER_S_Motor_Differentiell, CIFER_S_Motor_Parallel])
legend('Servo Differentiell', 'Servo Parallel', 'Motor Differentiell', 'Motor Parallel'); grid on;; title('Steuereingänge');
xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
subplot(4,1,2);
plot(Zeit, [CIFER_data_Pitch, CIFER_data_Roll,CIFER_data_Yaw])
legend('Pitch', 'Roll', 'Yaw'); grid on;; title('Pitch/Roll/Yaw');
xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
subplot(4,1,3);
plot(Zeit, [CIFER_data_RollRate, CIFER_data_PitchRate,CIFER_data_YawRate])
legend('RollRate', 'PitchRate', 'YawRate'); grid on;; title('Gyro');
xlabel('Zeitvektor [s]'); ylabel('Winkelgeschwindigket [rad/s]');
subplot(4,2,7);
plot(Zeit, CIFER_data_Hoehe)
legend('Hoehe'); grid on; title('Höhe');
xlabel('Zeitvektor [s]'); ylabel('Höhe [müM');
subplot(4,2,8);
plot(Zeit, CIFER_data_vz)
legend('vz'); grid on; title('Geschwindigkeit nach z');
xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');

% figure(2);
% subplot(2,1,1);
% plot(Zeit, [CIFER_data_Servo_Links, CIFER_data_Servo_Rechts,CIFER_data_Motor_Links, CIFER_data_Motor_Rechts])
% legend('Servo Links', 'Servo Rechts', 'Motor Links', 'Motor Rechts'); grid on;; title('Steuereingänge');
% xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
% subplot(2,1,2);
% plot(Zeit, [CIFER_data_RollRate, CIFER_data_PitchRate,CIFER_data_YawRate])
% legend('RollRate', 'PitchRate', 'YawRate'); grid on;; title('Gyro');
% xlabel('Zeitvektor [s]'); ylabel('Winkelgeschwindigket [rad/s]');

%% 16) CIFER - Differentielles Leinenziehen
if (1==2)
    
    % U1 -> Differentielles Leinenziehen
    % Y1 -> Yaw
    % Y2 -> Yawrate
    % Y3 -> Roll
    % Y4 -> Rollrate
    % Y5 -> Pitch
    % Y6 -> Pitchrate
    
    % Erster Datensatz des differentiellen Leinenziehen
    % -------------------------------------------------
    StartZeit = 11000;
    EndZeit   = 18000;
    U1                = CIFER_S_Servo_Differentiell(StartZeit:EndZeit,1);
    Y1                = unwrap(CIFER_data_Yaw(StartZeit:EndZeit,1));
    Y2                = CIFER_data_YawRate(StartZeit:EndZeit,1);
    Y3                = CIFER_data_Roll(StartZeit:EndZeit,1);
    Y4                = CIFER_data_RollRate(StartZeit:EndZeit,1);
    Y5                = CIFER_data_Pitch(StartZeit:EndZeit,1);
    Y6                = CIFER_data_PitchRate(StartZeit:EndZeit,1);
    CIFER_data_Zeit   = (Ts:Ts:length(U1)*Ts)';
    
    % Speichere Datei
    matrix = [CIFER_data_Zeit U1 Y1 Y2 Y3 Y4 Y5 Y6];
    save realData_DiffServo_1.txt matrix -ascii;
    
    % Alles plotten
    figure(1);
    subplot(2,1,1);
    plot(CIFER_data_Zeit, [U1, Y1, Y2, Y3, Y4, Y5, Y6])
    legend('U1 - Servo','Y1 - Yaw','Y2 - Yawrate','Y3 - Roll','Y4 - Rollrate','Y5 - Pitch','Y6 - Pitchrate'); grid on;; title('1 Differentielles Leinenziehen vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    subplot(2,1,2);
    plot(CIFER_data_Zeit, [U1, Y3])
    legend('U1 - Servo','Y3 - Roll'); grid on;; title('1 IN / OUT');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    
    
    
    figure(5)
    subplot(2,1,1)
    plot(CIFER_data_Zeit, U1)
    legend('U1 - Servo'); grid on;; title('3 Differentielles Leinenziehen vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    subplot(2,1,2)
    plot(CIFER_data_Zeit, Y4)
    legend('Y3','Y4'); grid on;; title('3 Differentielles Leinenziehen vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    %%
    % Zweiter Datensatz des differentiellen Leinenziehen
    % --------------------------------------------------
    StartZeit = 23000;
    EndZeit   = 34000;
    U1                = CIFER_S_Servo_Differentiell(StartZeit:EndZeit,1);
    Y1                = unwrap(CIFER_data_Yaw(StartZeit:EndZeit,1));
    Y2                = CIFER_data_YawRate(StartZeit:EndZeit,1);
    Y3                = CIFER_data_Roll(StartZeit:EndZeit,1);
    Y4                = CIFER_data_RollRate(StartZeit:EndZeit,1);
    Y5                = CIFER_data_Pitch(StartZeit:EndZeit,1);
    Y6                = CIFER_data_PitchRate(StartZeit:EndZeit,1);
    CIFER_data_Zeit   = (Ts:Ts:length(U1)*Ts)';
    
    % Speichere Datei
    matrix = [CIFER_data_Zeit U1 Y1 Y2 Y3 Y4 Y5 Y6];
    save realData_DiffServo_2.txt matrix -ascii;
    
    % Alles plotten
    subplot(2,1,1);
    plot(CIFER_data_Zeit, [U1, Y1, Y2, Y3, Y4, Y5, Y6])
    legend('U1 - Servo','Y1 - Yaw','Y2 - Yawrate','Y3 - Roll','Y4 - Rollrate','Y5 - Pitch','Y6 - Pitchrate'); grid on;; title('2 Differentielles Leinenziehen vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    subplot(2,1,2);
    plot(CIFER_data_Zeit, [U1, Y3])
    legend('U1 - Servo','Y3 - Roll'); grid on;; title('2 IN / OUT');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    
    % Dritter Datensatz des differentiellen Leinenziehen
    % --------------------------------------------------
    StartZeit = 42000;
    EndZeit   = 52500;
    U1                = CIFER_S_Servo_Differentiell(StartZeit:EndZeit,1);
    Y1                = unwrap(CIFER_data_Yaw(StartZeit:EndZeit,1));
    Y2                = CIFER_data_YawRate(StartZeit:EndZeit,1);
    Y3                = CIFER_data_Roll(StartZeit:EndZeit,1);
    Y4                = CIFER_data_RollRate(StartZeit:EndZeit,1);
    Y5                = CIFER_data_Pitch(StartZeit:EndZeit,1);
    Y6                = CIFER_data_PitchRate(StartZeit:EndZeit,1);
    CIFER_data_Zeit   = (Ts:Ts:length(U1)*Ts)';
    
    % Speichere Datei
    matrix = [CIFER_data_Zeit U1 Y1 Y2 Y3 Y4 Y5 Y6];
    save realData_DiffServo_3.txt matrix -ascii;
    
    % Alles plotten
    subplot(2,1,1);
    plot(CIFER_data_Zeit, [U1, Y1, Y2, Y3, Y4, Y5, Y6])
    legend('U1 - Servo','Y1 - Yaw','Y2 - Yawrate','Y3 - Roll','Y4 - Rollrate','Y5 - Pitch','Y6 - Pitchrate'); grid on;; title('3 Differentielles Leinenziehen vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    subplot(2,1,2);
    plot(CIFER_data_Zeit, [U1, Y3])
    legend('U1 - Servo','Y3 - Roll'); grid on;; title('3 IN / OUT');
    xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    
    
    
    %% 17) CIFER - Paralleler Schub
    if (1==2)
        % U1 -> Paralleler Schub
        % Y1 -> Yaw
        % Y2 -> Yawrate
        % Y3 -> Roll
        % Y4 -> Rollrate
        % Y5 -> Pitch
        % Y6 -> Pitchrate
        
        % Erster Datensatz mit parallelem Motorschub
        % ------------------------------------------
        StartZeit = 33700;
        EndZeit   = 42500;
        U1                = CIFER_S_Motor_Parallel(StartZeit:EndZeit,1);
        Y1                = CIFER_data_Yaw(StartZeit:EndZeit,1);
        Y2                = CIFER_data_YawRate(StartZeit:EndZeit,1);
        Y3                = CIFER_data_Roll(StartZeit:EndZeit,1);
        Y4                = CIFER_data_RollRate(StartZeit:EndZeit,1);
        Y5                = CIFER_data_Pitch(StartZeit:EndZeit,1);
        Y6                = CIFER_data_PitchRate(StartZeit:EndZeit,1);
        CIFER_data_Zeit   = (Ts:Ts:length(U1)*Ts)';
        
        % Speichere Datei
        matrix = [CIFER_data_Zeit U1 Y5 Y6];
        save realData_Schub_1.txt matrix -ascii;
        
        % Alles plotten
        figure(1);
        subplot(2,1,1);
        plot(CIFER_data_Zeit, [U1, Y1, Y2, Y3, Y4, Y5, Y6])
        legend('U1 - Motor','Y1 - ','Y2 - Yawrate','Y3 - Roll','Y4 - Rollrate','Y5 - Pitch','Y6 - Pitchrate'); grid on;; title('1 Paralleler Schub vs. Roll/Yaw/Pitch, Rollrate/Yawrate/Pitchrate');
        xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
        subplot(2,1,2);
        plot(CIFER_data_Zeit, [U1, Y5])
        legend('U1 - Motor','Y1 - Pitch'); grid on;; title('1 IN / OUT');
        xlabel('Zeitvektor [s]'); ylabel('[rad], [rad/s]');
    end
end

%% ------------------------------------------------------------------------
if (1==2);
    
    %% PLOT - Motoren / Servos
    figure(1);
    subplot(2,1,1);
    plot(Zeit, [data_In_Motor_Links, data_In_Motor_Rechts,])
    legend('Motor links', 'Motor rechts'); grid on;; title('Motorenschub');
    xlabel('Zeitvektor [s]'); ylabel('Motorschub [%]');
    subplot(2,1,2);
    plot(Zeit, [data_In_Servo_Links, data_In_Servo_Rechts])
    legend('Servo links', 'Servo rechts'); grid on;; title('Servos Bremsleinen');
    xlabel('Zeitvektor [s]'); ylabel('Servostellung [%]');
    
    %% PLOT - Höhe darstellen
    subplot(1,1,1);
    Time = TIME_StartTime * 10^-6;
    Time = Time - min(Time);
    plot(Zeit, [data_Px4_BaroAlt, data_Px4_GPOS_Alt, data_Xsens_BaroAlt, data_Xsens_XGPO_Alt,data_Xsens_XGPS_Alt]);
    h_legend = legend('PX4 Barodaten','PX4 GPOS Höhe', 'Xsens Barodaten', 'Xsens GPOS Höhe', 'Xsens GPS Höhe');
    set(h_legend,'FontSize',16);
    grid on;
    title('Verlgeich der Höhendaten (Höhe Allmend = 437müM)','fontSize',16);
    xlabel('Zeit [s]','fontSize',16); ylabel('Höhe [müM]','fontSize',16);
    set(gca,'FontSize',16)
    axis([-Inf max(Zeit) -Inf Inf])
    
    %% PLOT - Potentiometer
    subplot(1,1,1);
    p_Zeit = Zeit(1:length(RANG_Ang_l));
    n = 180/pi;
    plot(p_Zeit, [RANG_Ang_l*n, RANG_Ang_r*n]);
    h_legend = legend('relativer Winkel links','relativer Winkel rechts');
    set(h_legend,'FontSize',16);
    grid on;
    title('Relativer Winkel zwischen Last und Gleitschirm','fontSize',16);
    xlabel('Zeit [s]','fontSize',16); ylabel('Winkel [°]','fontSize',16);
    set(gca,'FontSize',16)
    axis([-Inf max(p_Zeit) -Inf Inf])
    
    
    %% PLOT - Vergleich Barodaten
    subplot(1,3,1);
    plot(Zeit, [data_Px4_BaroPres, data_Xsens_BaroPres])
    legend('Px4', 'Xsens'); grid on;; title('Barodaten Druck');
    xlabel('Zeitvektor [s]'); ylabel('Druck [Pa]');
    subplot(1,3,2);
    plot(Zeit, [data_Px4_BaroAlt, data_Xsens_BaroAlt])
    legend('Px4', 'Xsens'); grid on;; title('Barodaten Altitude');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [müM]');
    subplot(1,3,3);
    plot(Zeit, [data_Px4_BaroTemp, data_Xsens_BaroTemp])
    legend('Px4', 'Xsens'); grid on;; title('Barodaten Temperatur (Temperatur im Barometer)');
    xlabel('Zeitvektor [s]'); ylabel('Temperatur [°C]');
    
    %% PLOT - Vergleich Roll/Pitch/Yaw
    subplot(3,1,1);
    plot(Zeit, [data_Px4_ATT_Roll, data_Xsens_XATT_Roll])
    legend('Px4 Roll', 'Xsens Roll'); grid on;; title('Rollwinkel');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(3,1,2);
    plot(Zeit, [data_Px4_ATT_Pitch, data_Xsens_XATT_Pitch])
    legend('Px4 Pitch', 'Xsens Pitch'); grid on;; title('Pitchwinkel');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(3,1,3);
    plot(Zeit, [data_Px4_ATT_Yaw, data_Xsens_XATT_Yaw])
    legend('Px4 Yaw', 'Xsens Yaw'); grid on;; title('Yawwinkel');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - Vergleich Beschleunigungen - Px4/Xsens
    subplot(3,1,1);
    plot(Zeit, [data_Px4_AccX, data_Xsens_AccX])
    legend('Xsens AccX', 'Px4 AccX'); grid on;; title('Beschleunigung AccX');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    subplot(3,1,2);
    plot(Zeit, [data_Px4_AccY, data_Xsens_AccY])
    legend('Xsens AccY', 'Px4 AccY'); grid on;; title('Beschleunigung AccY');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    subplot(3,1,3);
    plot(Zeit, [data_Px4_AccZ, data_Xsens_AccZ])
    legend('Xsens AccZ', 'Px4 AccZ'); grid on;; title('Beschleunigung AccZ');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    
    %% PLOT - Vergeleich Gyrodaten - Px4/Xsens
    subplot(3,1,1);
    plot(Zeit, [data_Px4_GyroX, data_Xsens_GyroX])
    legend('Px4 GyroX', 'Xsens GyroX'); grid on;; title('Kreiselfrequenz GyroX');
    xlabel('Zeitvektor [s]'); ylabel('Kreisfrequenz [rad/s]');
    subplot(3,1,2);
    plot(Zeit, [data_Px4_GyroY, data_Xsens_GyroY])
    legend('Px4 GyroY', 'Xsens GyroY'); grid on;; title('Kreiselfrequenz GyroY');
    xlabel('Zeitvektor [s]'); ylabel('Kreisfrequenz [rad/s]');
    subplot(3,1,3);
    plot(Zeit, [data_Px4_GyroZ, data_Xsens_GyroZ])
    legend('Px4 GyroZ', 'Xsens GyroZ'); grid on;; title('Kreiselfrequenz GyroZ');
    xlabel('Zeitvektor [s]'); ylabel('Kreisfrequenz [rad/s]');
    
    %% PLOT - Vergleich - Lon/Lat/Alt
    subplot(1,3,1);
    plot(Zeit, [data_Xsens_XGPO_Lat, data_Xsens_XGPS_Lat, data_Px4_GPOS_Lat])
    legend('XGPO Lat', 'XGPS Lat', 'GPOS Lat'); grid on;; title('latitude');
    xlabel('Zeitvektor [s]'); ylabel('Breitengrad [°]');
    subplot(1,3,2);
    plot(Zeit, [data_Xsens_XGPO_Lon, data_Xsens_XGPS_Lon, data_Px4_GPOS_Lon])
    legend('XGPO Lon', 'XGPS Lon', 'GPOS Lon'); grid on;; title('Longitude');
    xlabel('Zeitvektor [s]'); ylabel('Längengrad [°]');
    subplot(1,3,3);
    plot(Zeit, [data_Xsens_XGPO_Alt, data_Xsens_XGPS_Alt, data_Px4_GPOS_Alt])
    legend('XGPO Alt', 'XGPS Alt', 'GPOS Alt'); grid on;; title('Altitude');
    xlabel('Zeitvektor [s]'); ylabel('Höhe [m]');
    
    %% PLOT - Vergleich Geschwindigkeiten
    subplot(3,1,1);
    plot(Zeit, [data_Xsens_XGPO_VelN, data_Xsens_XGPS_VelN, data_Px4_GPOS_VelN])
    legend('XGPO VelN', 'XGPS VelN', 'GPOS VelN'); grid on;; title('Geschwindigkeit in Norden VelN');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    subplot(3,1,2);
    plot(Zeit, [data_Xsens_XGPO_VelE, data_Xsens_XGPS_VelE, data_Px4_GPOS_VelE])
    legend('XGPO VelE', 'XGPS VelE', 'GPOS VelE'); grid on;; title('Geschwindigkeit in Osten VelE');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    subplot(3,1,3);
    plot(Zeit, [data_Xsens_XGPO_VelD, data_Xsens_XGPS_VelD, data_Px4_GPOS_VelD])
    legend('XGPO VelD', 'XGPS VelD', 'GPOS VelD'); grid on;; title('Geschwindigkeit nach unten VelD');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    
    %% PLOT - Gierwinkel skalieren
    subplot(3,1,1);
    plot(TIME_StartTime/1000000, [ATT_Yaw_scaled, ATT_Yaw])
    legend('Gierwinkel skaliert', 'Gierwinkel original'); grid on;; title('Gierwinkel Px4 skalieren');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(3,1,2);
    plot(TIME_StartTime/1000000, [XATT_Yaw_scaled, XATT_Yaw])
    legend('Gierwinkel skaliert', 'Gierwinkel original'); grid on;; title('Gierwinkel Xsens skalieren');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(3,1,3);
    plot(Zeit, [data_Px4_ATT_Yaw, data_Xsens_XATT_Yaw])
    legend('Gierwinkel Px4', 'Gierwinkel Xsens'); grid on;; title('Vergleich Gierwinkel Xsens und Px4');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - Aus AccX,AccY,AccZ --> Vx,Vy,Vz berechnen
    figure(1)
    subplot(2,3,1);
    plot(Zeit, [data_Xsens_AccX, data_Px4_AccX])
    legend('Xsens', 'Px4'); grid on; title('AccX');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    subplot(2,3,2);
    plot(Zeit, [data_Xsens_AccY, data_Px4_AccY])
    legend('Xsens', 'Px4'); grid on; title('AccY');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    subplot(2,3,3);
    plot(Zeit, [data_Xsens_AccZ, data_Px4_AccZ])
    legend('Xsens', 'Px4'); grid on; title('AccZ');
    xlabel('Zeitvektor [s]'); ylabel('Beschleunigung [m/s^2]');
    subplot(2,3,4);
    plot(Zeit, [data_Xsens_vx_calculated, data_Px4_vx_calculated])
    legend('Xsens', 'Px4'); grid on; title('Vx');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    subplot(2,3,5);
    plot(Zeit, [data_Xsens_vy_calculated, data_Px4_vy_calculated])
    legend('Xsens', 'Px4'); grid on; title('Vy');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    subplot(2,3,6);
    plot(Zeit, [data_Xsens_vz_calculated, data_Px4_vz_calculated])
    legend('Xsens', 'Px4'); grid on; title('Vz');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    
    %% PLOT - Aus GyroX... --> Pitch... berechnen
    figure(1)
    subplot(4,1,1);
    plot(Zeit, [data_In_Motor_Rechts, data_In_Motor_Rechts, data_In_Servo_Links, data_In_Servo_Rechts])
    legend('Motor Links', 'Motor Rechts', 'Servo Links', 'Servo Rechts'); grid on; title('GyroX');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(4,1,2);
    plot(Zeit, [data_Xsens_Roll_calculated, data_Px4_Roll_calculated, data_Xsens_XATT_Roll])
    legend('Roll aus Xsens berechnet', 'Roll aus Px4 berechnet', 'Roll aus Ysens gemessen'); grid on; title('Rollwinkel aus Gyro X Daten');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(4,1,3);
    plot(Zeit, [data_Xsens_Pitch_calculated, data_Px4_Pitch_calculated, data_Xsens_XATT_Pitch])
    legend('Pitch aus Xsens berechnet', 'Pitch aus Px4 berechnet', 'Pitch aus Xsens gemessen'); grid on; title('Pitchwinkel aus Gyro Y Daten');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(4,1,4);
    plot(Zeit, [data_Xsens_Yaw_calculated, data_Px4_Yaw_calculated, data_Xsens_XATT_Yaw])
    legend('Gier aus Xsens berechnet', 'Gier aus Px4 berechnet', 'Gier aus Xsens gemessen'); grid on; title('Gierwinkel aus Gyro Z Daten');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - Absolute  Fahrgeschwindigkeit
    figure(1);
    subplot(1,1,1);
    plot(Zeit, [data_V_absolut, data_Xsens_XGPS_VelN,data_Xsens_XGPS_VelE])
    legend('Absolute Geschwindigkeit', 'XGPS VelN', 'XGPS VelE'); grid on;; title('Geschwindigkeiten');
    xlabel('Zeitvektor [s]'); ylabel('Geschwindigkeit [m/s]');
    
    %% PLOT - Eingänge mit den Ausgängen vergleichen
    subplot(2,2,1);
    plot(Zeit, [data_In_Motor_Links, data_In_Motor_Rechts])
    legend('Motor links', 'Motor rechts'); grid on;; title('Motorenschub');
    xlabel('Zeitvektor [s]'); ylabel('Motorschub [%]');
    subplot(2,2,2);
    plot(Zeit, [data_In_Servo_Links, data_In_Servo_Rechts])
    legend('Servo links', 'Servo rechts'); grid on;; title('BremsleinenServos');
    xlabel('Zeitvektor [s]'); ylabel('Servostellung [%]');
    subplot(2,2,3);
    plot(Zeit, [data_Xsens_XATT_Pitch, data_Px4_ATT_Pitch])
    legend('Xsens Pitch', 'Px4 Pitch'); grid on;; title('Pitch');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(2,2,4);
    plot(Zeit, [data_Xsens_XATT_Roll, data_Px4_ATT_Roll])
    legend('Xsens Roll', 'Px4 Roll'); grid on;; title('Roll');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - Steuereingänge
    
    subplot(3,2,1);
    plot(Zeit, [data_In_Servo_Links, data_In_Servo_Rechts])
    legend('A1 Servo links', 'A2 Servo rechts'); grid on;; title('Aktuatoren (effektives PWM am Servo)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(3,2,2);
    plot(Zeit, [S_Servo_Differentiel, S_Servo_Parallel])
    legend('S1 Differentiel', 'S2 Parallel'); grid on;; title('Steuereingang Servo (Ansteuerung durch Fernbedienung)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(3,2,3);
    plot(Zeit, [data_In_Motor_Links, data_In_Motor_Rechts])
    legend('A1 Motor links', 'A2 Motor rechts'); grid on;; title('Aktuatoren (effektives PWM am Motor)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(3,2,4);
    plot(Zeit, [S_Motor_Differentiel, S_Motor_Parallel])
    legend('S1 Differentiel', 'S2 Parallel'); grid on;; title('Steuereingang Motor (Ansteuerung durch Fernbedienung)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(3,2,5);
    plot(Zeit, [data_Poti_Links, data_Poti_Rechts])
    legend('A1 Poti links', 'A2 Poti rechts'); grid on;; title('Potistellung');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    subplot(3,2,6);
    plot(Zeit, [S_Poti_Differentiel, S_Poti_Parallel])
    legend('S1 Differentiel', 'S2 Parallel'); grid on;; title('Steuerstellung (umgerechnet in laterale und longitudinale Position)');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - Durch Potis korrigierte Winkel
    
    subplot(2,2,1);
    plot(Zeit, [S_Poti_Parallel, S_Poti_Differentiel])
    legend('S2 Parallel', 'S2 Differentiell'); grid on;; title('Potistellung');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(2,2,2);
    plot(Zeit, [data_Xsens_XATT_Pitch_PotiCorrected, data_Xsens_XATT_Pitch])
    legend('Korrigierte Daten', 'Orginaldaten'); grid on;; title('Korrigierter Pitchwinkel');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    subplot(2,2,3);
    plot(Zeit, S_Poti_Differentiel)
    legend('S2 Parallel'); grid on;; title('Potistellung');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(2,2,4);
    plot(Zeit, [data_Xsens_XATT_Yaw_PotiCorrected, data_Xsens_XATT_Yaw])
    legend('Korrigierte Daten', 'Orginaldaten'); grid on;; title('Korrigierter Yawinkelr');
    xlabel('Zeitvektor [s]'); ylabel('Winkel [rad]');
    
    %% PLOT - IN / OUT in einem grossen Plot
    subplot(2,1,1);
    plot(Zeit, [S_Motor_Differentiel, S_Motor_Parallel, data_Xsens_XATT_Pitch, data_Px4_ATT_Pitch])
    legend('Motor links', 'Motor rechts', 'Xsens Pitch', 'Px4 Pitch'); grid on;; title('Motorenschub');
    xlabel('Zeitvektor [s]'); ylabel('Eingänge [%], Winkel [rad]');
    subplot(2,1,2);
    plot(Zeit, [S_Servo_Differentiel, S_Servo_Parallel, data_Xsens_XATT_Roll, data_Px4_ATT_Roll])
    legend('Servo links', 'Servo rechts','Xsens Roll', 'Px4 Roll'); grid on;; title('Servos Bremsleinen');
    xlabel('Zeitvektor [s]'); ylabel('Eingänge [%], Winkel [rad]');
    
    %% PLOT - Test
    subplot(2,1,1);
    plot(Zeit, [data_In_Servo_Links, data_In_Servo_Rechts])
    legend('A1 Servo links', 'A2 Servo rechts'); grid on;; title('Aktuatoren (effektives PWM am Servo)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    subplot(2,1,2);
    plot(Zeit, [S_Servo_Differentiel, S_Servo_Parallel])
    legend('S1 Differentiel', 'S2 Parallel'); grid on;; title('Steuereingang Servo (Ansteuerung durch Fernbedienung)');
    xlabel('Zeitvektor [s]'); ylabel('PWM [%]');
    
    
    
    %% Exponentielle Kurve
    exp_wmin = 2*pi*1/2;
    exp_wmax = 2*pi*1/8;
    
    t_exp  = (1:0.01:90);
    t_boe  = (exp_wmax+(sqrt(exp_wmin) - exp_wmax)/length(t_exp):(sqrt(exp_wmin) - exp_wmax)/length(t_exp):sqrt(exp_wmin));
    s_exp  = t_boe.^2;
    %s_lin  = ((max(s_exp)/length(t_exp)):(max(s_exp)/length(t_exp)):max(s_exp));
    s_lin  = (exp_wmax+(exp_wmin - exp_wmax)/length(t_exp):(exp_wmin - exp_wmax)/length(t_exp):exp_wmin);
    
    hold on;
    plot(t_exp, s_lin, 'g')
    plot(t_exp, s_exp)
    grid on; xlabel('Zeitvektor [s]'); ylabel('Kreisfrequenz [rad/s]');
    
    %%
    subplot(1,1,1);
    plot(TIME_StartTime/1000000, [RC_Ch3_scaled, RC_Ch4_scaled])
    h_legend = legend('Bremsleinenservo links', 'Bremsleinenservo rechts');
    set(h_legend,'FontSize',16);
    grid on;
    title('Ausschnitt einer Sweep-Anregung der Bremsleinenservos','fontSize',16);
    xlabel('Zeit [s]','fontSize',16);
    ylabel('PWM [ -1...1 ]','fontSize',16);
    set(gca,'FontSize',16)
    
    %% PLOT Logging Frequenz
    fail10 = 0;
    fail20 = 0;
    fail30 = 0;
    fail50 = 0;
    fail100 = 0;
    Frequenz = 100;
    Abtast = 1/Frequenz*1000;
    for i = 1:length(TIME_StartTime)-1
        TIME_diff(i) = (TIME_StartTime(i+1) - TIME_StartTime(i))/1000;
        
        if TIME_diff(i) > (Abtast * 2)
            fail100 = fail100+1;
            fail50 = fail50+1;
            fail30 = fail30+1;
            fail20 = fail20+1;
            fail10 = fail10+1;
        elseif TIME_diff(i) > (Abtast * 1.5)
            fail50 = fail50+1;
            fail30 = fail30+1;
            fail20 = fail20+1;
            fail10 = fail10+1;
        elseif TIME_diff(i) > (Abtast * 1.3)
            fail30 = fail30+1;
            fail20 = fail20+1;
            fail10 = fail10+1;
        elseif TIME_diff(i) > (Abtast * 1.2)
            fail20 = fail20+1;
            fail10 = fail10+1;
        elseif TIME_diff(i) > (Abtast * 1.1)
            fail10 = fail10+1;
        end
    end
    fail100 = 100/length(TIME_StartTime)*fail100;
    fail50 = 100/length(TIME_StartTime)*fail50;
    fail30 = 100/length(TIME_StartTime)*fail30;
    fail20 = 100/length(TIME_StartTime)*fail20;
    fail10 = 100/length(TIME_StartTime)*fail10;
    plot(StartZeit:EndZeit-1,TIME_diff,'+')
    str = sprintf('Zeitdifferenz zwischen zwei Logging-Einträgen (Soll = %dms)',Abtast);
    title(str,'fontSize',16);
    xlabel('Logging-Eintrag [n]','fontSize',16)
    ylabel('Zeitdifferenz [ms]','fontSize',16)
    set(gca,'FontSize',16)
    grid on
    axis([-Inf EndZeit Abtast max(TIME_diff)])
    
    %% PLOT Logging Frequenz Xsens GPS POS
    Datensatz = XGPS_POS_t;
    for i = 1:length(Datensatz)-1
        TIME_diff(i) = (Datensatz(i+1) - Datensatz(i))/1000;
    end
    plot(1:length(Datensatz)-1,TIME_diff,'+')
    str = sprintf('Zeitdifferenz zwischen zwei neuen GPS Einträgen der Xsens Einheit');
    title(str,'fontSize',16);
    xlabel('Logging-Eintrag [n]','fontSize',16)
    ylabel('Zeitdifferenz [ms]','fontSize',16)
    set(gca,'FontSize',16)
    grid on
    axis([-Inf length(TIME_diff) 180 320])
    
end