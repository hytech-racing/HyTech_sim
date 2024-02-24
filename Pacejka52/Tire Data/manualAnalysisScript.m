load("B1965run15.mat")
P = P*0.145038;

figure(2)
subplot(6,1,1)
plot(ET,FX)
ylabel('Fx (N)')
subplot(6,1,2)
plot(ET,FY)
ylabel('Fy (N)')
subplot(6,1,3)
plot(ET,FZ)
ylabel('Fz (N)')
subplot(6,1,4)
plot(ET,SA)
ylabel('SA (deg)')
subplot(6,1,5)
plot(ET,P)
ylabel('Pressure (PSI)')
subplot(6,1,6)
plot(ET,IA)
ylabel('Camber Angle')
disp('Select data limits')
[x,~] = ginput(2);
mask = (ET < x(1)) | (ET > x(2));
ET_trim = ET(~mask);
FX_trim = FX(~mask);
FY_trim = FY(~mask);
FZ_trim = -FZ(~mask);
SA_trim = -SA(~mask);
NFY_trim = NFY(~mask);
P_trim = P(~mask);
IA_trim = IA(~mask);
TSTC_trim = TSTC(~mask);
MZ_trim = MZ(~mask);
MX_trim = MX(~mask);

pause(0.5)

% xdft = fft(FZ_trim);
%    % sampling interval -- assuming equal sampling
%    DT = ET_trim(2)-ET_trim(1);
%    % sampling frequency
%    Fs = 1/DT;
%    DF = Fs/length(ET_trim);
%    freq = 0:DF:Fs/2;
%    xdft = xdft(1:length(xdft)/2+1);
%    figure
%    plot(freq,abs(xdft))
%% Camber Analysis
load("B1965run15.mat")
P = P*0.145038;

figure
hold on
IAvec = [0,2,4];
FZvec = [-850];
pVec = [10];
Legend = {};
for i = 1:length(IAvec)
    iaHigh = IAvec(i) + 1;
    iaLow = IAvec(i) - 1;
    for j = 1:length(FZvec)
        fzHigh = FZvec(j) + 50;
        fzLow = FZvec(j) - 50;
        for k = 1:length(pVec)
            pHigh = pVec(k) + 1;
            pLow = pVec(k) - 1;
            FYfilt = FY((fzHigh > FZ & fzLow < FZ) & (iaHigh > IA & iaLow < IA) & pHigh > P & pLow < P);
            SAfilt = SA((fzHigh > FZ & fzLow < FZ) & (iaHigh > IA & iaLow < IA) & pHigh > P & pLow < P);
            plot(SAfilt,FYfilt)
            Legend = [Legend,['FZ = ',num2str(FZvec(j)),' IA = ',num2str(IAvec(i)),' P = ',num2str(pVec(k))]];
        end
    end
end
legend(Legend)
