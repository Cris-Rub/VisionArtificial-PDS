clc;
clear all;
close all hidden;

p = arduino('COM3','UNO','Libraries','Servo');

servo_x = servo(p,'D6');
servo_y = servo(p,'D9');

angulo = 90;

writePosition(servo_x,90/180);
writePosition(servo_y,90/180);

v = videoinput("winvideo", 1, "MJPG_640x480");
v.FramesPerTrigger = Inf;
v.ReturnedColorspace = "rgb";

snapshot1 = getsnapshot(v);
X = im2double(snapshot1);
[m, n, o] = size(X);
figure(1);
imshow(snapshot1);
hold on;
plot(320, 240, 'ob');

