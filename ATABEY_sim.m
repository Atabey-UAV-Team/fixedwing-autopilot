clc, clear, close all

simTime = 60;

% SABİTLER
x0 = [0;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0;];

u = [0;
    0;
    0;
    0;];

% KONTROL SINIRLARI
u1min = 0*pi/180;       % Aileron min/max derecesi
u1max = -1*u1min;

u2min = 0*pi/180;       % Stabilizer min/max derecesi
u2max = -1*u2min;

u3min = 0*pi/180;       % Rudder min/max derecesi
u3max = -1*u3min;

u4min = 0;              % Motor çalışma aralığı
u4max = 1;

%% SİMÜLASYON
sim("ATABEY_sistem_modeli.slx")

%% PLOTLAR
time = simTime;  
X = out.SimulatedOutputs;
U = out.SimulatedInputs;

figure('Name','Sistem Durumları','NumberTitle','off')
numStates = size(X,2);
for i = 1:numStates
    subplot(numStates,1,i)
    plot(time, X(:,i), 'LineWidth', 1.5)
    grid on
    xlabel('Simülasyon Zamanı: [s]')
    ylabel(['x_' num2str(i)])
    title(['Durum: x_' num2str(i)])
end

figure('Name','Kontrol Girdileri','NumberTitle','off')
numInputs = size(U,2);
for i = 1:numInputs
    subplot(numInputs,1,i)
    plot(time, U(:,i), 'LineWidth', 1.5)
    hold on
    yline(eval(['u' num2str(i) 'min']), '--r', 'Min')  % min limit
    yline(eval(['u' num2str(i) 'max']), '--g', 'Max')  % max limit
    grid on
    xlabel('Time [s]')
    ylabel(['u_' num2str(i)])
    title(['Kontrol Girdisi: u_' num2str(i)])
    legend('Girdi','Min Limit','Max Limit')
end