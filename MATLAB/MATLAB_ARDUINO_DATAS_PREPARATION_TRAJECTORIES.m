% %Program for preparing the data before sending them to arduino. 
% 
% %--- preparing the data for the ellipse trajectory
% 
% %reading the datas from the xlsx file
% table = xlsread("fishbot_quat.xlsx");
% time = table(:, 1);
% a_data = table(:, 2);
% b_data = table(:, 3);
% c_data = table(:, 4);
% d_data = table(:, 5);
% 
% %convert the quaternion into angles between 0° and 180°
% angles_ellipse = zeros(16901, 3);
% for i = 1:1:16901
%     phi_i = atan(2 * (a_data(i)*b_data(i) + c_data(i)*d_data(i))/(a_data(i)*a_data(i) - b_data(i)*b_data(i) - c_data(i)*c_data(i) + d_data(i)*d_data(i))) * 180/pi;
%     theta_i = - asin(2*(b_data(i)*d_data(i) - a_data(i)*c_data(i))) * 180/pi;
%     psi_i = atan(2 *(a_data(i)*d_data(i) + b_data(i)*c_data(i))/(a_data(i)*a_data(i) + b_data(i)*b_data(i) - c_data(i)*c_data(i) - d_data(i)*d_data(i))) * 180/pi;
%     angles_ellipse(i, 1) = phi_i + 90;
%     angles_ellipse(i, 2) = theta_i + 90;
%     angles_ellipse(i, 3) = psi_i + 90;
% end

%--- preparing the data for the 8-figure trajectory

%reading the datas from the csv file

datas = csvread("trajectorydata/SampleTrajectory_ColumnATime_ColumnsBtoDTaitBryanAngles.csv");
%datas = csvread("trajectorydata/Gen_0_C_1_MaxAng_25_ThkAng_0_RotAng_0_RotInt_0_SpdCde_0_Spdupv_1_Kv_0_hdev_0_freq_0.19_TB.csv");

t = datas(:, 1);
phi_datas = datas(:, 2);
theta_datas = datas(:, 3);
psi_datas = datas(:, 4);

%convert the angles into 0° and 180°. In the csv file the data are already
%TaitBryans angles so we just need to add 90° becaue they are between -90°
%and 90° in the file. 
vectors = zeros(size(datas, 1), 4);
for i = 1:1:size(datas)
    vectors(i, 1) = t(i, 1);
    vectors(i, 2) = phi_datas(i, 1) + 90;
    vectors(i, 3) = theta_datas(i, 1) + 90;
    vectors(i, 4) = psi_datas(i, 1) + 90;
end