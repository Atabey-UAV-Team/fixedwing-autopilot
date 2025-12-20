function [XDOT, GeodeticCoordinates] = ATABEY_dynamics(X, U)

%% DURUM VE KONTROL TANIMLAMALARI
x1 = X(1);  % u
x2 = X(2);  % v
x3 = X(3);  % w
x4 = X(4);  % p
x5 = X(5);  % q
x6 = X(6);  % r
x7 = X(7);  % phi
x8 = X(8);  % theta
x9 = X(9);  % psi
x13 = X(13); % Motor açısal hız 

u1 = U(1);  % delta_Aileron (rad)
u2 = U(2);  % delta_Stabilizer (rad)
u3 = U(3);  % delta_Rudder (rad)
u4 = U(4);  % delta_Motor [0-1]

%% SABİTLER
% Araç Sabitleri
mass = 14;          % Toplam kütle (kg)

cbar = 0;           % Ortalama Aerodinamik Kord (m)
lt = 0;             % Kuyruğun Aerodinamik Kordu - Araç Gövdesi arası mesafe (m) 
wingSurface = 0;    % Kanat alanı (m^2)
tailSurface = 0;    % Kuyruk alanı (m^2)

Xcg = 0;            % Moment referansına göre (Fm), ağırlık merkezinin X konumu (m)
Ycg = 0;            % Moment referansına göre (Fm), ağırlık merkezinin Y konumu (m)
Zcg = 0;            % Moment referansına göre (Fm), ağırlık merkezinin Z konumu (m)

Xac = 0;            % Moment referansına göre (Fm), aerodinamik merkezin X konumu (m)
Yac = 0;            % Moment referansına göre (Fm), aerodinamik merkezin Y konumu (m)
Zac = 0;            % Moment referansına göre (Fm), aerodinamik merkezin Z konumu (m)

Xapt = 0;           % Moment referansına göre (Fm), motor kuvvetinin X konumu (m)
Yapt = 0;           % Moment referansına göre (Fm), motor kuvvetinin Y konumu (m)
Zapt = 0;           % Moment referansına göre (Fm), motor kuvvetinin Z konumu (m)

% Çevresel Sabitler
gravity = 9.81;             % Yer çekimi (m/s^2)
airDensity = 1.225;         % Hava yoğunluğu (kg/m^3)
depsda = 0.25;              % Alpha'ya göre downwash değişimi (rad/rad)

alpha_L0 = 0*pi/180 ;       % 0 lift iken hücum açısı (rad)
n_CL = 5.5;                 % Lift grafiğinin lineer bölgedeki eğimi
alpha_3 = 0.0;              % alpha^3 katsayısı
alpha_2 = 0.0;              % alpha^2 katsayısı
alpha_1 = 0.0;              % alpha^1 katsayısı
alpha_0 = 0.0;              % alpha^0 katsayısı
alphaSwitch = 0*pi/180;     % Lift grafiğinin lineer bölgeden nonlineer 
                            % bölgeye geçişindeki alfa değeri (rad)

% Kontrol Sınırları -> Simulasyon Koduna Gömülü
% Simulink Saturation bloğu ile kısıtlama yapıldı

% Motor Parametreleri
motorMaxOmega = 0;          % Azami motor hızı, deneysel (rad/s)
rotorDiameter = 0;          % Pervane çapı (m)
motor_CT0 = 0;              % Deneysel Katsayılar
motor_CT1 = 0;
motor_CT2 = 0;

%% DEĞİŞKEN DEĞERLER
airspeed = max(sqrt(x1^2 + x2^2 + x3^2), 0.1);    % Airspeed
% Kalkış anında kararsızlık koruması

alpha = atan2(x3,x1);                             % α
beta = atan2(x2, sqrt(x1^2 + x3^2 + eps));        % β

dynamicPressure = 0.5*airDensity*airspeed^2;      % Dinamik basınç

wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%% AERODİNAMİK KUVVET KATSAYILARI
% CL_wb
if alpha < alphaSwitch
    CL_wb = n_CL*(alpha - alpha_L0);                       
    % Lineer bölge
else
    CL_wb = alpha_3*alpha^3 + alpha_2*alpha^2 + alpha_1*alpha + alpha_0;    
    % Nonlineer bölge
end

% CL_t
epsilon = depsda*(alpha-alpha_L0);
alpha_t = alpha-epsilon+u2+1.3*x5*lt/airspeed;     % ?
CL_t = 3.1*(tailSurface/wingSurface)*alpha_t;


totalLift = CL_wb + CL_t;                      % Toplam Lift, CL

totalDrag = 0.13 + 0.07*(5.5*alpha+0.654)^2;   % Toplam Drag, CD

sideForce = -1.6*beta + 0.24*u3;               % Yanal Kuvvet, CY, ?, CY = CYβ​​*β + CYδr*​​​δr

%% F_s'E GÖRE BOYUTSAL KUVVETLER
FA_s = [-totalDrag*dynamicPressure*wingSurface;
         sideForce*dynamicPressure*wingSurface;
        -totalLift*dynamicPressure*wingSurface];

FA_b = roty(alpha)*FA_s;

%% AC'YE GÖRE AERODİNAMİK MOMENT KATSAYILARI
eta11 = 0; % ???
eta21 = 0;
eta31 = 0;
eta = [eta11;eta21;eta31];

dCMdx = zeros(3,3);
dCMdu = zeros(3,3);

CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

% AC'YE GÖRE AERODİNAMİK MOMENTLER
Mac_b = CMac_b*dynamicPressure*wingSurface*cbar;

%% CG'YE GÖRE AERODİNAMİK MOMENTLER
rcg_b = [Xcg;Ycg;Zcg;];
rac_b = [Xac;Yac;Zac];
MAcg_b = Mac_b + cross(FA_b,rcg_b - rac_b);

%% İTKİ KUVVETLERİ VE MOMENTLERİ
% MOTOR HESAPLARI
motorOmega = x13;
motorTau = 0.05;                                   % Motor zaman sabiti, deneysel

motorOmegaCmd = motorMaxOmega*u4;                  % Anlık motor çevrimi girdisi
motorOmegaDot = (motorOmegaCmd - x13)/motorTau;    % Motor çevrimi girdisi türevi
x13dot = motorOmegaDot;

revolutions_Motor = motorOmega/(2*pi);
J = airspeed/(revolutions_Motor*rotorDiameter + eps);
CT = motor_CT0 + motor_CT1*J + motor_CT2*J^2;

FMo = airDensity * revolutions_Motor^2 * rotorDiameter^4 * CT;    
FE_b = [FMo;0;0];   % Motor Fb ile aynı hizada varsayımı - (DEĞİŞTİR)

mew = [Xcg - Xapt; Yapt - Ycg; Zcg - Zapt];
MEcg_b = cross(mew,FE_b);

%% YERÇEKİMİ
g_b = [-gravity*sin(x8);gravity*cos(x8)*sin(x7);gravity*cos(x8)*cos(x7);];
Fg_b = mass*g_b;

%% DURUM TÜREVLERİ
InertiaMatrix = mass * [zeros(3,3)];     % Eylemsizlik matrisi               
invInertia = inv(InertiaMatrix);         % Ib oluşturulunca hard code'la

F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/mass) * F_b - cross(wbe_b,V_b);

Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invInertia*(Mcg_b - cross(wbe_b,InertiaMatrix*wbe_b));   
% invIb hard code'landığında uyarı gidecek

% EULER HESAPLARI
% Quaternion kullanımı?
% Kararsızlık koruması
x8_sat = min(max(x8, -(89*pi/180)), 89*pi/180);
H_phi = [ 1  sin(x7)*tan(x8_sat)   cos(x7)*tan(x8_sat);
          0  cos(x7)             -sin(x7);
          0  sin(x7)/cos(x8_sat)  cos(x7)/cos(x8_sat) ];
    
x7to9dot = H_phi*wbe_b;

%% NAVİGASYON DENKLEMLERİ
Cvb = (rotx(x7)*roty(x8)*rotz(x9))';

x10to12dot = Cvb*V_b;

%% JEODEZİK KOORDİNATLAR
% WGS-84 sabitleri
wgs84_a  = 6378137.0;
wgs84_f  = 1/298.257223563;
wgs84_e2 = wgs84_f * (2 - wgs84_f);

% ECEF'e Göre Pozisyonlar
Xecef = X(10);
Yecef = X(11);
Zecef = X(12);

p_geodetic = sqrt(Xecef^2 + Yecef^2);

% Longitude (λ)
geodetic_lon = atan2(Yecef, Xecef);

% Bowring yöntemi
minor_semi_axis  = wgs84_a*sqrt(1 - wgs84_e2);
ep_geodetic2 = (wgs84_a^2 - minor_semi_axis^2)/minor_semi_axis^2;

theta_geodetic = atan2(Zecef*wgs84_a, p_geodetic*minor_semi_axis);

geodetic_lat = atan2( ...
    Zecef + ep_geodetic2*minor_semi_axis*sin(theta_geodetic)^3, ...
    p_geodetic - wgs84_e2*wgs84_a*cos(theta_geodetic)^3 );

% Radius of curvature in the prime vertical
N_geodetic = wgs84_a / sqrt(1 - wgs84_e2*sin(geodetic_lat)^2);

% Height (h)
geodetic_h = p_geodetic/cos(geodetic_lat) - N_geodetic;

% Çıktı
GeodeticCoordinates = [geodetic_lon, geodetic_lat, geodetic_h, X(7), X(8), X(9)];

%% BİRİNCİ DERECEDEN SONUÇ
XDOT = [x1to3dot; x4to6dot; x7to9dot; x10to12dot; x13dot];

end