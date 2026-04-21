function a = drag_accel(r_vec, v_vec, Cd, A, m, Re)
% drag_accel  Atmospheric drag acceleration [m/s²] in ECI.
%
% Uses a multi-layer exponential atmosphere (COESA 1976 / Harris-Priester
% style) that is valid from sea-level to ~1000 km.  The original
% single-scale-height model (H=8500 m) gave ~10^-20 kg/m³ at ISS altitude
% instead of the correct ~4×10^-12 kg/m³.

alt  = norm(r_vec) - Re;       % geometric altitude (m)
rho  = layer_density(alt);     % kg/m³
vmag = norm(v_vec);
if vmag < 1e-10
    a = zeros(3,1);
    return;
end
a = -0.5 * rho * Cd * (A/m) * vmag * v_vec;
end

% ── Multi-layer exponential atmosphere ───────────────────────────────────
function rho = layer_density(alt_m)
% Each row: [base_alt_km,  rho_base_kg/m3,  scale_height_km]
% Derived from COESA 1976 standard atmosphere tables.
alt = alt_m / 1000;   % convert to km

if     alt <   0;  rho = 1.225;
elseif alt <  25;  rho = 1.225e+0  * exp(-alt        /  8.44);
elseif alt <  30;  rho = 3.899e-2  * exp(-(alt-  25) /  6.49);
elseif alt <  40;  rho = 1.774e-2  * exp(-(alt-  30) /  6.75);
elseif alt <  50;  rho = 3.972e-3  * exp(-(alt-  40) /  7.07);
elseif alt <  60;  rho = 1.057e-3  * exp(-(alt-  50) /  7.47);
elseif alt <  70;  rho = 3.206e-4  * exp(-(alt-  60) /  7.83);
elseif alt <  80;  rho = 8.770e-5  * exp(-(alt-  70) /  8.78);
elseif alt <  90;  rho = 1.905e-5  * exp(-(alt-  80) / 11.40);
elseif alt < 100;  rho = 3.396e-6  * exp(-(alt-  90) / 13.10);
elseif alt < 110;  rho = 5.604e-7  * exp(-(alt- 100) / 16.10);
elseif alt < 130;  rho = 9.708e-8  * exp(-(alt- 110) / 22.60);
elseif alt < 160;  rho = 2.222e-8  * exp(-(alt- 130) / 34.00);
elseif alt < 200;  rho = 3.832e-9  * exp(-(alt- 160) / 51.00);
elseif alt < 300;  rho = 2.541e-10 * exp(-(alt- 200) / 38.70);  % 200 km: 2.5e-10
elseif alt < 400;  rho = 1.916e-11 * exp(-(alt- 300) / 61.10);  % 300 km: 1.9e-11
elseif alt < 500;  rho = 3.725e-12 * exp(-(alt- 400) / 52.80);  % 400 km: 3.7e-12  ← ISS
elseif alt < 700;  rho = 5.608e-13 * exp(-(alt- 500) / 58.20);  % 500 km: 5.6e-13
elseif alt < 900;  rho = 3.070e-14 * exp(-(alt- 700) / 74.00);
else;              rho = 1.136e-15 * exp(-(alt- 900) / 91.00);
end
end
