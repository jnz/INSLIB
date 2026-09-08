%% ------------------------------------------------------------------
%  Generate a synthetic static IMU recording in the INSLIB replay
%  format, with a KNOWN Angle/Velocity Random Walk and Rate Random
%  Walk, to verify that python/allan_variance.py recovers them.
%
%  Format (see datasets/replay_format.py):
%      t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]
%  t_us is int64 microseconds, body frame FRD (x fwd, y right, z down),
%  so a level static accelerometer reads about -9.81 on z.
% ------------------------------------------------------------------
close all; clear; clc;

rng(27);                  % reproducible
fs      = 50;             % sample rate [Hz]
t_total = 8*3600;         % 8 h -- the +1/2 branch needs hours to develop
outfile = 'imu.csv';

% --- Injected noise -------------------------------------------------
% ARW/VRW is an AMPLITUDE spectral density: [unit]/sqrt(Hz), which is
% the same as [unit]*sqrt(s). RRW is the random walk's driving strength
% in [unit]/sqrt(s). The tool reports N and K in exactly these units
% (and gyr_psd/acc_psd = N^2, the POWER spectral density).
gyr_arw_dps = 0.005;      % deg/s/sqrt(Hz)  = deg/sqrt(s)
gyr_rrw_dps = 0.0003;     % (deg/s)/sqrt(s)
acc_vrw     = 1.0e-3;     % m/s^2/sqrt(Hz)
acc_rrw     = 6.0e-5;     % (m/s^2)/sqrt(s)

% A constant offset the way an uncalibrated MEMS part has one. The Allan
% variance is blind to it (it differences the signal), so it must not
% change the recovered N/K -- included precisely to demonstrate that.
gyr_fixed_bias_dps = [1.8, 1.1, 0.1];

DEG = pi/180;
G   = 9.80665;
n   = round(fs * t_total);

% --- One channel: white noise + random-walk bias --------------------
% white:  per-sample stddev = N*sqrt(fs)      -> ADEV(tau) = N/sqrt(tau)
% drift:  per-step  stddev  = K/sqrt(fs)      -> ADEV(tau) = K*sqrt(tau/3)
chan = @(arw, rrw) arw*sqrt(fs)*randn(n,1) + (rrw/sqrt(fs))*cumsum(randn(n,1));

gyr = zeros(n,3);
acc = zeros(n,3);
for k = 1:3
    gyr(:,k) = (chan(gyr_arw_dps, gyr_rrw_dps) + gyr_fixed_bias_dps(k)) * DEG;
    acc(:,k) =  chan(acc_vrw,     acc_rrw);
end
acc(:,3) = acc(:,3) - G;          % z down -> specific force is -g

% --- Write the CSV (fprintf, not writematrix: 1.4M rows) ------------
t_us = round((0:n-1)' / fs * 1e6);
M    = [t_us, gyr, acc];

fid = fopen(outfile, 'w');
if fid < 0, error('cannot open %s for writing', outfile); end
fprintf(fid, '# t_us, gyr_frd_xyz [rad/s], acc_frd_xyz [m/s^2]\n');
fprintf(fid, '%.0f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n', M');
fclose(fid);

% --- What the tool should report back --------------------------------
tau_cross = sqrt(3) * gyr_arw_dps / gyr_rrw_dps;   % branch crossover [s]
fprintf('wrote %s: %d samples, %.2f h @ %g Hz\n\n', outfile, n, t_total/3600, fs);
fprintf('EXPECTED from allan_variance.py:\n');
fprintf('  gyr ARW      N = %.4e rad/s/sqrt(Hz)\n', gyr_arw_dps*DEG);
fprintf('  gyr_psd    N^2 = %.4e (rad/s)^2/Hz\n',   (gyr_arw_dps*DEG)^2);
fprintf('  gyr_bias_rw  K = %.4e rad/s/sqrt(s)\n',  gyr_rrw_dps*DEG);
fprintf('  acc VRW      N = %.4e m/s^2/sqrt(Hz)\n', acc_vrw);
fprintf('  acc_psd    N^2 = %.4e (m/s^2)^2/Hz\n',   acc_vrw^2);
fprintf('  acc_bias_rw  K = %.4e m/s^2/sqrt(s)\n',  acc_rrw);
fprintf('\n  branch crossover tau* = sqrt(3)*N/K = %.0f s, fit cap = %.0f s\n', ...
        tau_cross, t_total/10);
fprintf('  -> the +1/2 branch spans ~%.0f s .. %.0f s, plenty to fit\n', ...
        tau_cross, t_total/10);