clc 
clear all
close all

%importing data
servodata = importdata('servodata2.txt'); %counts
angledata = importdata('angledata.txt');  %radians

%converting radians to degree

radangle = angledata*(180/pi);

atime = [0:(20.91/999):20.91]';
atib_1 = radangle(:,1);
afem_1 = radangle(:,2);
atib_2 = radangle(:,3);
afem_2 = radangle(:,4);
atib_3 = radangle(:,5);
afem_3 = radangle(:,6);
atib_4 = radangle(:,7);
afeb_4 = radangle(:,8);


time1  = servodata(:,1);
time = time1-time1(1);
tibia_1 = servodata(:,2);
femur_1 = servodata(:,3);
tibia_2 = servodata(:,4);
femur_2 = servodata(:,5);
tibia_3 = servodata(:,6);
femur_3 = servodata(:,7);
tibia_4 = servodata(:,8);
femur_4 = servodata(:,9);

figure(1)
plot(time, tibia_1,atime,atib_1)
