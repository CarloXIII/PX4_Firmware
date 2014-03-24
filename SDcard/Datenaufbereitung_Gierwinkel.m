%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dieses Script importiert die vorhandenen Daten der .bin - Datei und  %
% speichert diese in eine mat-Datei ab. Ausserdem wird hier geplottet  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    Writer:        Walter von Matt                                    %
%                   Lukas Köpfli                                       %
%    Organisation:  HSLU                                               %
%    Date:          März 2014                                          %
%    Version:       V4.1                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 1)  Erklärungen der Variablen und Normeinheiten
% 2)  Variablendeklarationen (Abtastzeit definieren)
% 3)  Daten werden eingelesen und in Spaltenvektoren gespeichert. Die
%     ladende Datei heisst 'data.csv'.
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

%  ---------------------------------------------------------------
%  Kanal    ¦ Variable    ¦  Funktion            ¦  Zugehörigkeit ¦
%  ---------------------------------------------------------------
%  1        ¦ RC_Ch0      ¦  Roll/Querruder      ¦  PX4
%  2        ¦ RC_Ch1      ¦  Nick/Höhenruder     ¦  PX4
%  3        ¦ RC_Ch2      ¦  Schub/Gas           ¦  PX4
%  4        ¦ RC_Ch3      ¦  Gier/Seitenruder    ¦  PX4
%  5        ¦ RC_Ch4      ¦  Main Mode Switch    ¦  PX4
%  6        ¦ RC_Ch5      ¦  Assist Switch       ¦  PX4
%  7        ¦ RC_Ch6      ¦  Heck Servo          ¦  Paraglider
%  8        ¦ RC_Ch7      ¦  Bremsleine links    ¦  Paraglider
%  9        ¦ RC_Ch8      ¦  Bremsleine rechts   ¦  Paraglider
% 10        ¦ RC_Ch9      ¦  Motor links         ¦  Paraglider
% 11        ¦ RC_Ch10     ¦  Motor rechts        ¦  Paraglider
% 12        ¦ RC_Ch11     ¦  Empfangsweiche      ¦  Paraglider

%  -----------------------------------
%  Variable    ¦  Funktion            ¦
%  -----------------------------------
%  OUT0_Out0   ¦  Heck Servo
%  OUT0_Out1   ¦  Bremsleine links
%  OUT0_Out2   ¦  Bremsleine rechts
%  OUT0_Out3   ¦  Motor links
%  OUT0_Out4   ¦  Motor rechts

%% 2)   Abtastzeit definieren
% Abtastfrequenz vom Px4 Board : 50Hz
% Abtastfrequenz vom Xsens     : 50Hz
Ts = 1 / 50;

% Anfang und Schluss wegschneiden
% Länge der Daten: Excel öffnen und Zeilen * Ts rechnen.
StartZeit = 50240;
EndZeit   = 91640;

%% 3)   Einlesen der Daten (Nur das erste Mal nötig)

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
ATT_Time = sysvector.ATT_Time(imintime:imaxtime);
ATT_Roll = sysvector.ATT_Roll(imintime:imaxtime);
ATT_Pitch = sysvector.ATT_Pitch(imintime:imaxtime);
ATT_Yaw = sysvector.ATT_Yaw(imintime:imaxtime);
ATT_RollRate = sysvector.ATT_RollRate(imintime:imaxtime);
ATT_PitchRate = sysvector.ATT_PitchRate(imintime:imaxtime);
ATT_YawRate = sysvector.ATT_YawRate(imintime:imaxtime);
ATT_GX = sysvector.ATT_GX(imintime:imaxtime);
ATT_GY = sysvector.ATT_GY(imintime:imaxtime);
ATT_GZ = sysvector.ATT_GZ(imintime:imaxtime);
ATSP_Time = sysvector.ATSP_Time(imintime:imaxtime);
ATSP_RollSP = sysvector.ATSP_RollSP(imintime:imaxtime);
ATSP_PitchSP = sysvector.ATSP_PitchSP(imintime:imaxtime);
ATSP_YawSP = sysvector.ATSP_YawSP(imintime:imaxtime);
ATSP_ThrustSP = sysvector.ATSP_ThrustSP(imintime:imaxtime);
IMU_Time = sysvector.IMU_Time(imintime:imaxtime);
IMU_AccX = sysvector.IMU_AccX(imintime:imaxtime);
IMU_AccY = sysvector.IMU_AccY(imintime:imaxtime);
IMU_AccZ = sysvector.IMU_AccZ(imintime:imaxtime);
IMU_GyroX = sysvector.IMU_GyroX(imintime:imaxtime);
IMU_GyroY = sysvector.IMU_GyroY(imintime:imaxtime);
IMU_GyroZ = sysvector.IMU_GyroZ(imintime:imaxtime);
IMU_MagX = sysvector.IMU_MagX(imintime:imaxtime);
IMU_MagY = sysvector.IMU_MagY(imintime:imaxtime);
IMU_MagZ = sysvector.IMU_MagZ(imintime:imaxtime);
% SENS_BaroPres = sysvector.SENS_BaroPres(imintime:imaxtime);
% SENS_BaroAlt = sysvector.SENS_BaroAlt(imintime:imaxtime);
% SENS_BaroTemp = sysvector.SENS_BaroTemp(imintime:imaxtime);
% SENS_DiffPres = sysvector.SENS_DiffPres(imintime:imaxtime);
% LPOS_Time = sysvector.LPOS_Time(imintime:imaxtime);
% LPOS_X = sysvector.LPOS_X(imintime:imaxtime);
% LPOS_Y = sysvector.LPOS_Y(imintime:imaxtime);
% LPOS_Z = sysvector.LPOS_Z(imintime:imaxtime);
% LPOS_VX = sysvector.LPOS_VX(imintime:imaxtime);
% LPOS_VY = sysvector.LPOS_VY(imintime:imaxtime);
% LPOS_VZ = sysvector.LPOS_VZ(imintime:imaxtime);
% LPOS_RefLat = sysvector.LPOS_RefLat(imintime:imaxtime);
% LPOS_RefLon = sysvector.LPOS_RefLon(imintime:imaxtime);
% LPOS_RefAlt = sysvector.LPOS_RefAlt(imintime:imaxtime);
% LPOS_XYFlags = sysvector.LPOS_XYFlags(imintime:imaxtime);
% LPOS_ZFlags = sysvector.LPOS_ZFlags(imintime:imaxtime);
% LPOS_Landed = sysvector.LPOS_Landed(imintime:imaxtime);
% LPSP_X = sysvector.LPSP_X(imintime:imaxtime);
% LPSP_Y = sysvector.LPSP_Y(imintime:imaxtime);
% LPSP_Z = sysvector.LPSP_Z(imintime:imaxtime);
% LPSP_Yaw = sysvector.LPSP_Yaw(imintime:imaxtime);
% GPS_GPS_t = sysvector.GPS_GPS_t(imintime:imaxtime);
% GPS_FixType = sysvector.GPS_FixType(imintime:imaxtime);
% GPS_EPH = sysvector.GPS_EPH(imintime:imaxtime);
% GPS_EPV = sysvector.GPS_EPV(imintime:imaxtime);
% GPS_POS_t = sysvector.GPS_POS_t(imintime:imaxtime);
% GPS_Lat = sysvector.GPS_Lat(imintime:imaxtime);
% GPS_Lon = sysvector.GPS_Lon(imintime:imaxtime);
% GPS_Alt = sysvector.GPS_Alt(imintime:imaxtime);
% GPS_VEL_t = sysvector.GPS_VEL_t(imintime:imaxtime);
% GPS_VelN = sysvector.GPS_VelN(imintime:imaxtime);
% GPS_VelE = sysvector.GPS_VelE(imintime:imaxtime);
% GPS_VelD = sysvector.GPS_VelD(imintime:imaxtime);
% GPS_Cog = sysvector.GPS_Cog(imintime:imaxtime);
ATTC_Time = sysvector.ATTC_Time(imintime:imaxtime);
ATTC_Roll = sysvector.ATTC_Roll(imintime:imaxtime);
ATTC_Pitch = sysvector.ATTC_Pitch(imintime:imaxtime);
ATTC_Yaw = sysvector.ATTC_Yaw(imintime:imaxtime);
ATTC_Thrust = sysvector.ATTC_Thrust(imintime:imaxtime);
STAT_Time = sysvector.STAT_Time(imintime:imaxtime);
STAT_MainState = sysvector.STAT_MainState(imintime:imaxtime);
STAT_ArmState = sysvector.STAT_ArmState(imintime:imaxtime);
STAT_BatRem = sysvector.STAT_BatRem(imintime:imaxtime);
STAT_BatWarn = sysvector.STAT_BatWarn(imintime:imaxtime);
STAT_Landed = sysvector.STAT_Landed(imintime:imaxtime);
RC_Time = sysvector.RC_Time(imintime:imaxtime);
RC_Ch0 = sysvector.RC_Ch0(imintime:imaxtime);
RC_Ch1 = sysvector.RC_Ch1(imintime:imaxtime);
RC_Ch2 = sysvector.RC_Ch2(imintime:imaxtime);
RC_Ch3 = sysvector.RC_Ch3(imintime:imaxtime);
RC_Ch4 = sysvector.RC_Ch4(imintime:imaxtime);
RC_Ch5 = sysvector.RC_Ch5(imintime:imaxtime);
RC_Ch6 = sysvector.RC_Ch6(imintime:imaxtime);
RC_Ch7 = sysvector.RC_Ch7(imintime:imaxtime);
RC_Ch8 = sysvector.RC_Ch8(imintime:imaxtime);
RC_Ch9 = sysvector.RC_Ch9(imintime:imaxtime);
RC_Ch10 = sysvector.RC_Ch10(imintime:imaxtime);
RC_Ch11 = sysvector.RC_Ch11(imintime:imaxtime);
RC_Count = sysvector.RC_Count(imintime:imaxtime);
OUT0_Time = sysvector.OUT0_Time(imintime:imaxtime);
OUT0_Out0 = sysvector.OUT0_Out0(imintime:imaxtime);
OUT0_Out1 = sysvector.OUT0_Out1(imintime:imaxtime);
OUT0_Out2 = sysvector.OUT0_Out2(imintime:imaxtime);
OUT0_Out3 = sysvector.OUT0_Out3(imintime:imaxtime);
OUT0_Out4 = sysvector.OUT0_Out4(imintime:imaxtime);
OUT0_Out5 = sysvector.OUT0_Out5(imintime:imaxtime);
OUT0_Out6 = sysvector.OUT0_Out6(imintime:imaxtime);
OUT0_Out7 = sysvector.OUT0_Out7(imintime:imaxtime);
% AIRS_Time = sysvector.AIRS_Time(imintime:imaxtime);
% AIRS_IndSpeed = sysvector.AIRS_IndSpeed(imintime:imaxtime);
% AIRS_TrueSpeed = sysvector.AIRS_TrueSpeed(imintime:imaxtime);
% ARSP_Time = sysvector.ARSP_Time(imintime:imaxtime);
% ARSP_RollRateSP = sysvector.ARSP_RollRateSP(imintime:imaxtime);
% ARSP_PitchRateSP = sysvector.ARSP_PitchRateSP(imintime:imaxtime);
% ARSP_YawRateSP = sysvector.ARSP_YawRateSP(imintime:imaxtime);
% FLOW_Time = sysvector.FLOW_Time(imintime:imaxtime);
% FLOW_RawX = sysvector.FLOW_RawX(imintime:imaxtime);
% FLOW_RawY = sysvector.FLOW_RawY(imintime:imaxtime);
% FLOW_CompX = sysvector.FLOW_CompX(imintime:imaxtime);
% FLOW_CompY = sysvector.FLOW_CompY(imintime:imaxtime);
% FLOW_Dist = sysvector.FLOW_Dist(imintime:imaxtime);
% FLOW_Q = sysvector.FLOW_Q(imintime:imaxtime);
% FLOW_SensID = sysvector.FLOW_SensID(imintime:imaxtime);
% GPOS_Time = sysvector.GPOS_Time(imintime:imaxtime);
% GPOS_Lat = sysvector.GPOS_Lat(imintime:imaxtime);
% GPOS_Lon = sysvector.GPOS_Lon(imintime:imaxtime);
% GPOS_Alt = sysvector.GPOS_Alt(imintime:imaxtime);
% GPOS_VelN = sysvector.GPOS_VelN(imintime:imaxtime);
% GPOS_VelE = sysvector.GPOS_VelE(imintime:imaxtime);
% GPOS_VelD = sysvector.GPOS_VelD(imintime:imaxtime);
% GPOS_BaroAlt = sysvector.GPOS_BaroAlt(imintime:imaxtime);
% GPOS_Flags = sysvector.GPOS_Flags(imintime:imaxtime);
% GPSP_NavState = sysvector.GPSP_NavState(imintime:imaxtime);
% GPSP_Lat = sysvector.GPSP_Lat(imintime:imaxtime);
% GPSP_Lon = sysvector.GPSP_Lon(imintime:imaxtime);
% GPSP_Alt = sysvector.GPSP_Alt(imintime:imaxtime);
% GPSP_Yaw = sysvector.GPSP_Yaw(imintime:imaxtime);
% GPSP_Type = sysvector.GPSP_Type(imintime:imaxtime);
% GPSP_LoitR = sysvector.GPSP_LoitR(imintime:imaxtime);
% GPSP_LoitDir = sysvector.GPSP_LoitDir(imintime:imaxtime);
% GPSP_PitMin = sysvector.GPSP_PitMin(imintime:imaxtime);
% ESC_Time = sysvector.ESC_Time(imintime:imaxtime);
% ESC_Cnt = sysvector.ESC_Cnt(imintime:imaxtime);
% ESC_NumESC = sysvector.ESC_NumESC(imintime:imaxtime);
% ESC_Conn = sysvector.ESC_Conn(imintime:imaxtime);
% ESC_N = sysvector.ESC_N(imintime:imaxtime);
% ESC_Ver = sysvector.ESC_Ver(imintime:imaxtime);
% ESC_Adr = sysvector.ESC_Adr(imintime:imaxtime);
% ESC_Volt = sysvector.ESC_Volt(imintime:imaxtime);
% ESC_Amp = sysvector.ESC_Amp(imintime:imaxtime);
% ESC_RPM = sysvector.ESC_RPM(imintime:imaxtime);
% ESC_Temp = sysvector.ESC_Temp(imintime:imaxtime);
% ESC_SetP = sysvector.ESC_SetP(imintime:imaxtime);
% ESC_SetPRAW = sysvector.ESC_SetPRAW(imintime:imaxtime);
% GVSP_VX = sysvector.GVSP_VX(imintime:imaxtime);
% GVSP_VY = sysvector.GVSP_VY(imintime:imaxtime);
% GVSP_VZ = sysvector.GVSP_VZ(imintime:imaxtime);
% BATT_Time = sysvector.BATT_Time(imintime:imaxtime);
% BATT_V = sysvector.BATT_V(imintime:imaxtime);
% BATT_VFilt = sysvector.BATT_VFilt(imintime:imaxtime);
% BATT_C = sysvector.BATT_C(imintime:imaxtime);
% BATT_Discharged = sysvector.BATT_Discharged(imintime:imaxtime);
XIMU_Time = sysvector.XIMU_Time(imintime:imaxtime);
XIMU_AccX = sysvector.XIMU_AccX(imintime:imaxtime);
XIMU_AccY = sysvector.XIMU_AccY(imintime:imaxtime);
XIMU_AccZ = sysvector.XIMU_AccZ(imintime:imaxtime);
XIMU_GyroX = sysvector.XIMU_GyroX(imintime:imaxtime);
XIMU_GyroY = sysvector.XIMU_GyroY(imintime:imaxtime);
XIMU_GyroZ = sysvector.XIMU_GyroZ(imintime:imaxtime);
XIMU_MagX = sysvector.XIMU_MagX(imintime:imaxtime);
XIMU_MagY = sysvector.XIMU_MagY(imintime:imaxtime);
XIMU_MagZ = sysvector.XIMU_MagZ(imintime:imaxtime);
% XSEN_BaroPres = sysvector.XIMU_MagZ(imintime:imaxtime);
% XSEN_BaroAlt = sysvector.XSEN_BaroAlt(imintime:imaxtime);
% XSEN_BaroTemp = sysvector.XSEN_BaroTemp(imintime:imaxtime);
% XSEN_DiffPres = sysvector.XSEN_DiffPres(imintime:imaxtime);
% XGPS_GPS_t = sysvector.XGPS_GPS_t (imintime:imaxtime);
% XGPS_FixTyp = sysvector.XGPS_FixTyp(imintime:imaxtime);
% XGPS_EPH = sysvector.XGPS_EPH(imintime:imaxtime);
% XGPS_EPV = sysvector.XGPS_EPV(imintime:imaxtime);
% XGPS_POS_t = sysvector.XGPS_POS_t(imintime:imaxtime);
% XGPS_Lat = sysvector.XGPS_Lat(imintime:imaxtime);
% XGPS_Lon = sysvector.XGPS_Lon(imintime:imaxtime);
% XGPS_Alt = sysvector.XGPS_Alt(imintime:imaxtime);
% XGPS_VEL_t = sysvector.XGPS_VEL_t(imintime:imaxtime);
% XGPS_VelN = sysvector.XGPS_VelN(imintime:imaxtime);
% XGPS_VelE = sysvector.XGPS_VelE(imintime:imaxtime);
% XGPS_VelD = sysvector.XGPS_VelD(imintime:imaxtime);
% XGPS_Cog = sysvector.XGPS_Cog(imintime:imaxtime);
XATT_Time = sysvector.XATT_Time(imintime:imaxtime);
XATT_Roll = sysvector.XATT_Roll(imintime:imaxtime);
XATT_Pitch = sysvector.XATT_Pitch(imintime:imaxtime);
XATT_Yaw = sysvector.XATT_Yaw(imintime:imaxtime);
XATT_RollRate = sysvector.XATT_RollRate(imintime:imaxtime);
XATT_PitchRate = sysvector.XATT_PitchRate(imintime:imaxtime);
XATT_YawRate = sysvector.XATT_YawRate(imintime:imaxtime);
% XGPO_Time = sysvector.XGPO_Time(imintime:imaxtime);
% XGPO_Lat = sysvector.XGPO_Lat(imintime:imaxtime);
% XGPO_Lon = sysvector.XGPO_Lon(imintime:imaxtime);
% XGPO_Alt = sysvector.XGPO_Alt(imintime:imaxtime);
% XGPO_VelN = sysvector.XGPO_VelN(imintime:imaxtime);
% XGPO_VelE = sysvector.XGPO_VelE(imintime:imaxtime);
% XGPO_VelD = sysvector.XGPO_VelD(imintime:imaxtime);
RANG_Time = sysvector.RANG_Time(imintime:imaxtime);
RANG_Ang_l = sysvector.RANG_Ang_l(imintime:imaxtime);
RANG_Ang_r = sysvector.RANG_Ang_r(imintime:imaxtime);
TIME_StartTime = sysvector.TIME_StartTime(imintime:imaxtime);
VER_Arch = sysvector.VER_Arch(imintime:imaxtime);
VER_FwGit = sysvector.VER_FwGit(imintime:imaxtime);
PARM_Name = sysvector.PARM_Name(imintime:imaxtime);
PARM_Value = sysvector.PARM_Value(imintime:imaxtime);

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

% -------------------------------------------------------------------------

% Einänge bzw. Dutycycle in [%] interpolieren
data_In_Servo_Heck   = interp1(Time, RC_Ch0_scaled, Zeit, 'linear');
data_In_Servo_Links  = interp1(Time, RC_Ch3_scaled, Zeit, 'linear');
data_In_Servo_Rechts = interp1(Time, RC_Ch4_scaled, Zeit, 'linear');
data_In_Motor_Links  = interp1(Time, RC_Ch6_scaled, Zeit, 'linear');
data_In_Motor_Rechts = interp1(Time, RC_Ch7_scaled, Zeit, 'linear');

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

% -------------------------------------------------------------------------

% XSENS - Roll / Pitch / Yaw
data_Xsens_XATT_Roll  = interp1(Time, XATT_Roll_scaled,  Zeit, 'linear');
data_Xsens_XATT_Pitch = interp1(Time, XATT_Pitch, Zeit, 'linear');
data_Xsens_XATT_Yaw   = interp1(Time, XATT_Yaw_scaled,   Zeit, 'linear');

% XSENS - RollRate / PitchRate / YawRate
data_Xsens_XATT_RollRate  = interp1(Time, XATT_RollRate,  Zeit, 'linear');
data_Xsens_XATT_PitchRate = interp1(Time, XATT_PitchRate, Zeit, 'linear');
data_Xsens_XATT_YawRate   = interp1(Time, XATT_YawRate,   Zeit, 'linear');

% -------------------------------------------------------------------------

% Poti - Relativer Winkel zwischen Schirm und Paraglider
data_Poti_Links  = interp1(Time, RANG_Ang_l, Zeit, 'linear');
data_Poti_Rechts = interp1(Time, RANG_Ang_r, Zeit, 'linear');

%% 6)   Alle "NAN" Werte mit 0 ersetzen.
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
%% 12) Datensatz in .mat - Datei abspeichern
save('data.mat');

%% ------------------------------------------------------------------------

%% PLOT Rohdaten

%% PLOT - Relevante RC-Signale

figure(1);
subplot(2,1,1);
plot(Time,[RC_Ch2, RC_Ch3]);
legend('RC-Input Schub', 'RC-Input Gier'); grid on; title('RC-Rohdaten');
xlabel('Zeitvektor [s]'); ylabel('RC Input [-1...1]');
axis([-Inf Inf -1 1])
subplot(2,1,2);
plot(Time,[RC_Ch9, RC_Ch10]);
legend('RC-Input Motor links', 'RC-Input Motor rechts'); grid on; title('RC-Rohdaten');
xlabel('Zeitvektor [s]'); ylabel('RC Input [-1...1]');
axis([-Inf Inf -1 1])



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