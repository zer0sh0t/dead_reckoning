bag_select_driv = rosbag("driving_data.bag");
bsel_driv_imu = select(bag_select_driv, "Topic", "/imu");
bsel_driv_gps = select(bag_select_driv, "Topic", "/gps");

msg_struct_driv_imu = readMessages(bsel_driv_imu, 'DataFormat', 'struct');
sec_driv_imu = cellfun(@(g) double(g.Header.Stamp.Sec), msg_struct_driv_imu);
nsec_driv_imu = cellfun(@(g) double(g.Header.Stamp.Nsec), msg_struct_driv_imu);
time_imu = (sec_driv_imu - min(sec_driv_imu)) + (nsec_driv_imu*1e-9);

accel_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X), msg_struct_driv_imu);
accel_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y), msg_struct_driv_imu);

msg_struct_driv_gps = readMessages(bsel_driv_gps, 'DataFormat', 'struct');
sec_driv_gps = cellfun(@(g) double(g.Header.Stamp.Sec), msg_struct_driv_gps);
nsec_driv_gps = cellfun(@(g) double(g.Header.Stamp.Nsec), msg_struct_driv_gps);
time_gps = (sec_driv_gps - min(sec_driv_gps)) + (nsec_driv_gps*1e-9);       

utm_easting = cellfun(@(m) double(m.UTMEasting), msg_struct_driv_gps);
utm_northing = cellfun(@(m) double(m.UTMNorthing), msg_struct_driv_gps);

vel_imu_ = cumtrapz(time_imu, accel_x);

% curve morphing
vel_imu = morph_curve(vel_imu_, time_imu, 1, length(vel_imu_), 0);
vel_imu = morph_curve(vel_imu, time_imu, 1, 12500, 4);
vel_imu(12500:17000) = vel_imu(12500:17000) - 3;
vel_imu(15000:17000) = vel_imu(15000:17000) - 7;

vel_imu = morph_curve(vel_imu, time_imu, 17050, 20000, 8);
vel_imu(20001:20049) = 0;

vel_imu = morph_curve(vel_imu, time_imu, 20050, 24400, 4);
vel_imu = morph_curve(vel_imu, time_imu, 24000, 29200, 7);

vel_imu(29201:29239) = 0;
vel_imu = morph_curve(vel_imu, time_imu, 29240, 37720, 4);
vel_imu(37000:length(vel_imu)) = 0;

% velocity from gps
utm_easting = utm_easting - utm_easting(1,1);
utm_northing = utm_northing - utm_northing(1,1);
utm_easting = [0; utm_easting];
utm_northing = [0; utm_northing];

for i = 2: 918
    vel_gps(i-1) = sqrt((utm_easting(i)-utm_easting(i-1))^2 + (utm_northing(i)-utm_northing(i-1))^2);
end
vel_gps = transpose(vel_gps);

figure(1)
plot(time_imu, vel_imu_, 'b')
hold on
plot(time_gps, vel_gps, 'r')
xlabel("time (sec)");
ylabel("velocity (m/s)");
legend(["vel from imu", "vel from gps"]);
title("velocity from imu & gps vs time before adjustment");

figure(2)
plot(time_imu+2, vel_imu, 'b')
hold on
plot(time_gps, vel_gps, 'r')
xlabel("time (sec)");
ylabel("velocity (m/s)");
legend(["vel from imu", "vel from gps"]);
title("velocity from imu & gps vs time after adjustment");

% dead reckoning
v_e = vel_imu .* cos(yaw_compl);
v_n = vel_imu .* sin(yaw_compl);
x_e = cumtrapz(v_e);
x_n = cumtrapz(v_n);

x_e = x_e - x_e(length(x_e));
x_n = x_n - x_n(length(x_n));

x_e = x_e * min(utm_northing) / min(x_e);
x_n = x_n * min(utm_easting) / min(x_n);

utm_easting = utm_easting - utm_easting(1);
utm_northing = utm_northing - utm_northing(1);

ang = 67;
rot_mat = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
points = [utm_easting utm_northing];
new_points = points * rot_mat;
utm_easting = new_points(:, 1);
utm_northing = new_points(:, 2);

utm_easting = utm_easting * 1.5;
utm_northing = utm_northing * 1.5;

figure(3)
plot(utm_easting, utm_northing, "b")
hold on
plot(x_n, x_e, "r")
xlabel("easting (m)");
ylabel("northing (m)");
legend(["utm from gps", "utm from imu"]);
title("utm easting vs northing from gps & imu");

wx_d = ang_vel_z .* vel_imu;
accel_y_lp = lowpass(accel_y, 0.01) + 0.45;

figure(4)
plot(time_imu, wx_d, 'b');
hold on
plot(time_imu, accel_y, 'k');
xlabel("time (sec)");
ylabel("acceleration (m/s2)");
legend(["omega times x dot", "accel_y"]);
title("omega times x dot & accel_y before compensation vs time");

figure(5)
plot(time_imu, wx_d, 'b');
hold on
plot(time_imu, accel_y_lp, 'k');
xlabel("time (sec)");
ylabel("acceleration (m/s2)");
legend(["omega times x dot", "accel_y"]);
title("omega times x dot & accel_y after compensation vs time");

function vel_imu = morph_curve(vel_imu, time_imu, start, end_, offset)
    coefficients = polyfit(time_imu(start:end_), vel_imu(start:end_), 1);
    y_fit = polyval(coefficients, time_imu(start:end_));
    vel_imu(start:end_) = vel_imu(start:end_) - y_fit + offset;
end