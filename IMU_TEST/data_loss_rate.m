clear all
close all
clc
% 실행 시 txt파일의 첫번째 행을 삭제 후 실행할 것
client = readmatrix("imu_data_log_Client.txt");
server = readmatrix("imu_data_log_Server.txt");

[row_c, col_c] = size(client)
[row_s, col_s] = size(server)
ref = server(row_s, 1)

fprintf("데이터 손실율: %.4f\n", (1-row_s/ref) * 100)