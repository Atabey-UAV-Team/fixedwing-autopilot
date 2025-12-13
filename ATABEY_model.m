function XDOT = ATABEY_model(X, U)

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
x10 = X(10); % Motor açısal hız

u1 = U(1);  % delta_Aileron (rad)
u2 = U(2);  % delta_Stabilizer (rad)
u3 = U(3);  % delta_Rudder (rad)
u4 = U(4);  % delta_Motor [0-1]

%% SABİTLER
% Araç Sabitleri
m= 14;               % Toplam kütle (kg)

cbar = 0;           % Ortalama Aerodinamik Kord (m)
lt = 0;             % Kuyruğun Aerodinamik Kordu - Araç Gövdesi arası mesafe (m) 
S = 0;              % Kanat alanı (m^2)
St = 0;             % Kuyruk alanı (m^2)

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
g = 9.81;                   % Yer çekimi (m/s^2)
rho = 1.225;                % Hava yoğunluğu (kg/m^3)
depsda = 0.25;              % Alpha'ya göre downwash değişimi (rad/rad)

alpha_L0 = 0*pi/180 ;       % 0 lift iken hücum açısı (rad)
n = 5.5;                    % Lift grafiğinin lineer bölgedeki eğimi
a3 = 0.0;                   % alpha^3 katsayısı
a2 = 0.0;                   % alpha^2 katsayısı
a1 = 0.0;                   % alpha^1 katsayısı
a0 = 0.0;                   % alpha^0 katsayısı
alphaSwitch = 0*pi/180;     % Lift grafiğinin lineer bölgeden nonlineer 
                            % bölgeye geçişindeki alfa değeri (rad)

% Kontrol Sınırları -> Simulasyon Koduna Gömülü
% Simulink Saturation bloğu ile kısıtlama yapıldı

% Motor Parametreleri
omega_max = 0;          % Azami motor hızı, deneysel (rad/s)
D   = 0;                % Pervane çapı (m)
CT0 = 0;                % Deneysel Katsayılar
CT1 = 0;
CT2 = 0;

%% DEĞİŞKEN DEĞERLER
Va = sqrt(x1^2 + x2^2 + x3^2);  % Airspeed
Va_eff = max(Va, 0.1);          % Kalkış anında kararsızlık koruması

alpha = atan2(x3,x1);           % α
beta = asin(x2 / Va_eff);       % β

Q = 0.5*rho*Va^2;               % Dinamik basınç

wbe_b = [x4;x5;x6];
V_b = [x1;x2;x3];

%% AERODİNAMİK KUVVET KATSAYILARI
% CL_wb
if alpha < alphaSwitch
    CL_wb = n*(alpha - alpha_L0);                       % Lineer bölge
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;    % Nonlineer bölge
end

% CL_t
epsilon = depsda*(alpha-alpha_L0);
alpha_t = alpha-epsilon+u2+1.3*x5*lt/Va;     % ?
CL_t = 3.1*(St/s)*alpha_t;


CL = CL_wb + CL_t;                      % Toplam Lift

CD = 0.13 + 0.07*(5.5*alpha+0.654)^2;   % Toplam Drag

CY = -1.6*beta + 0.24*u3;               % Yanal Kuvvet, ?, CY = CYβ​​*β + CYδr*​​​δr

%% F_s'E GÖRE BOYUTSAL KUVVETLER
FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];

RyAlpha = [ cos(alpha) 0 sin(alpha);
                0      1     0;
           -sin(alpha) 0 cos(alpha)];
FA_b = RyAlpha*FA_s;

%% AC'YE GÖRE AERODİNAMİK MOMENT KATSAYILARI
eta11 = 0; % ???
eta21 = 0;
eta31 = 0;
eta = [eta11;eta21;eta31];

dCMdx = zeros(3,3);
dCMdu = zeros(3,3);

CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

% AC'YE GÖRE AERODİNAMİK MOMENTLER
MAC_b = CMac_b*Q*S*cbar;

%% CG'YE GÖRE AERODİNAMİK MOMENTLER
rcg_b = [Xcg;Ycg;Zcg;];
rac_b = [Xac;Yac;Zac];
MAcg_b = MAac_b + cross(FA_b,rcg_b - rac_b);

%% İTKİ KUVVETLERİ VE MOMENTLERİ
% MOTOR HESAPLARI
tau_m = 0.05;                           % Motor zaman sabiti, deneysel
omega_cmd = omega_max*u4;               % Anlık motor çevrimi girdisi
omega_dot = (omega_cmd - x10)/tau_m;    % Motor çevrimi girdisi türevi

omega = x10;
n = omega/(2*pi);
J = Va_eff/(n*D + eps);
CT = CT0 + CT1*J + CT2*J^2;

FMo = rho * n^2 * D^4 * CT;    
FE_b = [FMo;0;0];   % Motor Fb ile aynı hizada varsayımı - (DEĞİŞTİR)

mew = [Xcg - Xapt; Yapt - Ycg; Zcg - Zapt];
MEcg_b = cross(mew,FE_b);

%% YERÇEKİMİ
g_b = [-g*sin(x8);g*cos(x8)*sin(x7);g*cos(x8)*cos(x7);];
Fg_b = m*g_b;

%% DURUM TÜREVLERİ
Ib = m * [zeros(3,3)];        % Eylemsizlik matrisi               
invIb = inv(Ib);              % Ib oluşturulunca hard code'la

F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m) * F_b - cross(wbe_b,V_b);

Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b,Ib*wbe_b));   
% invIb hard code'landığında uyarı gidecek

% EULER HESAPLARI
% Quaternion kullanımı?
theta_lim = 89*pi/180;         % Kararsızlık koruması
x8_sat = min(max(x8, -theta_lim), theta_lim);
H_phi = [ 1  sin(x7)*tan(x8_sat)   cos(x7)*tan(x8_sat);
          0  cos(x7)             -sin(x7);
          0  sin(x7)/cos(x8_sat)  cos(x7)/cos(x8_sat) ];
    
x7to9dot = H_phi*wbe_b;

%% BİRİNCİ DERECEDEN SONUÇ
XDOT = [x1to3dot; x4to6dot; x7to9dot; omega_dot];

end