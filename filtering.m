bag_select_circle = rosbag("circle_data.bag");
bsel_circle = select(bag_select_circle, "Topic", "/imu");

msg_struct_circle = readMessages(bsel_circle, 'DataFormat', 'struct');
mag_x_circle = cellfun(@(m) double(m.MagField.MagneticField_.X), msg_struct_circle);
mag_y_circle = cellfun(@(m) double(m.MagField.MagneticField_.Y), msg_struct_circle);

bag_select_driv = rosbag("driving_data.bag");
bsel_driv = select(bag_select_driv, "Topic", "/imu");

msg_struct_driv = readMessages(bsel_driv, 'DataFormat', 'struct');
time = cellfun(@(m) double(m.Header.Seq), msg_struct_driv);
mag_x_driv = cellfun(@(m) double(m.MagField.MagneticField_.X), msg_struct_driv);
mag_y_driv = cellfun(@(m) double(m.MagField.MagneticField_.Y), msg_struct_driv);
ang_vel_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z), msg_struct_driv);

x = cellfun(@(m) double(m.IMU.Orientation.X), msg_struct_driv);
y = cellfun(@(m) double(m.IMU.Orientation.Y), msg_struct_driv);
z = cellfun(@(m) double(m.IMU.Orientation.Z), msg_struct_driv);
w = cellfun(@(m) double(m.IMU.Orientation.W), msg_struct_driv);
wxyz = [w x y z];
time = time - min(time);

b_ellipse = fit_ellipse(mag_x_circle, mag_y_circle, gca);

translation = [b_ellipse.X0_in b_ellipse.Y0_in];
rotation = [cos(b_ellipse.phi) -sin(b_ellipse.phi);sin(b_ellipse.phi) cos(b_ellipse.phi)];
scaling = [b_ellipse.short_axis/b_ellipse.long_axis 0;0 1];

coordinates_circle = [mag_x_circle mag_y_circle];
coordinates_driv = [mag_x_driv mag_y_driv];

final_coordinates_circle = scaling * rotation * transpose(coordinates_circle  - translation);
final_coordinates_driv = scaling * rotation * transpose(coordinates_driv  - translation);

cord_x_ = transpose(final_coordinates_circle(1,:));
cord_y_ = transpose(final_coordinates_circle(2,:));

cord_x = transpose(final_coordinates_driv(1,:));
cord_y = transpose(final_coordinates_driv(2,:));

cord_x(10302:18509) = movmean(cord_x(10302:18509), 120);
cord_y(10302:18509) = movmean(cord_y(10302:18509), 120);

figure(1)
plot(mag_x_circle, mag_y_circle, 'r');
hold on
plot(cord_x_, cord_y_);
xlabel("mag_x (Wb/m2)");
ylabel("mag_y (Wb/m2)");
legend(["before calib", "after calib"]);
title("magnetometer X-Y before and after calibration for circle data");

figure(2)
plot(mag_x_driv, mag_y_driv, 'r');
hold on
plot(cord_x, cord_y);
xlabel("mag_x (Wb/m2)");
ylabel("mag_y (Wb/m2)");
legend(["before calib", "after calib"]);
title("magnetometer X-Y before and after calibration");

figure(3)
plot(time, mag_x_driv, 'r')
hold on
plot(time, cord_x, 'b')
xlabel("time (sec)");
ylabel("mag_x (Wb/m2)");
legend(["before calib", "after calib"]);
title("mag_x vs time");

figure(4)
plot(time, mag_y_driv, 'r')
hold on
plot(time, cord_y, 'b')
xlabel("time (sec)");
ylabel("mag_y (Wb/m2)");
legend(["before calib", "after calib"]);
title("mag_y vs time");

euler = quat2eul(wxyz);
yaw = deg2rad(unwrap(euler(:, 1)));

yaw_gyro = deg2rad(unwrap(cumtrapz(ang_vel_z)));
% yaw_gyro = yaw_gyro - yaw_gyro(1,1);
yaw_mag_before = unwrap(atan2(-mag_y_driv, mag_x_driv));
yaw_mag = unwrap(atan2(-cord_y, cord_x));
% yaw_mag = yaw_mag - yaw_mag(1,1);

figure(5)
plot(time, yaw_mag_before, 'r');
hold on
plot(time, yaw_mag, 'b');
xlabel("time (sec)");
ylabel("yaw (rads)");
legend(["yaw from magnetometer before calib", "yaw from magnetometer after calib"]);
title("yaw vs time")

figure(6)
plot(time, yaw_gyro, 'r');
hold on
plot(time, yaw_mag, 'b');
xlabel("time (sec)");
ylabel("yaw (rads)");
legend(["yaw from gyroscope", "yaw from magnetometer"]);
title("yaw vs time")

yaw_gyro_hp = highpass(yaw_gyro, 0.00000001);
% yaw_gyro_hp = yaw_gyro_hp - yaw_gyro_hp(1,1);
yaw_mag_lp = lowpass(yaw_mag, 0.01);
% yaw_mag_lp = yaw_mag_lp - yaw_mag_lp(1,1);

alpha = 0.2;
yaw_compl = (1 - alpha) * yaw_mag_lp + alpha * yaw_gyro_hp;

figure(7)
plot(time, yaw_mag_lp, 'y');
hold on
plot(time, yaw_gyro_hp, 'b');
hold on
plot(time, yaw_compl, 'r');
hold on
xlabel("time (sec)");
ylabel("yaw (rads)");
legend(["lpf(from mag)", "hpf(from gyro)", "cf"]);
title("lpf, hpf and cpf vs time");

figure(8)
plot(time, yaw_compl, 'b');
hold on
plot(time, yaw, 'r');
xlabel("time (sec)");
ylabel("yaw (rads)");
legend(["cf", "yaw from imu"]);
title("cf and yaw from imu vs time")