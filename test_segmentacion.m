clc;
clear all;
close all hidden;

%% CONEXION A ARDUINO
p = arduino('COM3','UNO','Libraries','Servo');

%% SERVOMOTORES
servo_x = servo(p,'D6');
servo_y = servo(p,'D9');

%% SET DE MOTORES A 90°
angulox = 90/180;
anguloy = 90/180;
writePosition(servo_x,angulox);
writePosition(servo_y,anguloy);

%% CONFIGURACIÓN CÁMARA
v = videoinput("winvideo", 1, "MJPG_640x480");
% v = videoinput("winvideo", 1, "MJPG_1440x1080");
v.FramesPerTrigger = Inf;
v.ReturnedColorspace = "rgb";
set(v, 'TriggerFrameDelay', 5);
% src = getselectedsource(v);
% src.Brightness = 255;
start(v);

%% OBTENCIÓN DE COLOR A SEGEMENTAR
recording1 = getdata(v, v.FramesAvailable);
implay(recording1);
pause('on')
pause(2)
snapshot1 = getsnapshot(v);
X = im2double(snapshot1);
[m, n, o] = size(X);
% imshow(X);
select = roipoly(X);
umbral = 30/255;

%% Referencias del color seleccionado
refR = sum(sum(X(:,:,1).*select))/sum(select(:));
refG = sum(sum(X(:,:,2).*select))/sum(select(:));
refB = sum(sum(X(:,:,3).*select))/sum(select(:));

while true
    recording1 = getdata(v, v.FramesAvailable);

%     %% Se le añade ruido a la imagen capturada
%     X = X + 0.2 * randn(m, n, o);
%     
%     %% Filtro pasa bajas (eliminar ruido)
%     r = 300;
%     sigma = 0.05;
%     for z = 1 : o
%         Xf(:,:,z) = fftshift(fft2(X(:,:,z)));
%         H = zeros(m, n);
%         for i = 1 : m
%             dy = (i-m/2)/(m/2);
%             for j = 1 : n
%                 dx = (j-n/2)/(n/2);
%                 dxy = sqrt(dx^2+dy^2);
%                 H(i, j) = exp(-(dxy^2)/(2*sigma^2));
%             end
%         end
%         Yf(:,:,z) = Xf(:,:,z).*H; %Filtrado
%         Y(:,:,z) = ifft2(ifftshift(Yf(:,:,z))); % Regreso al dominio espacial
%     end

    %% Busqueda de color RGB
    busquedaR = X(:,:,1) > refR - umbral & X(:,:,1) < refR + umbral; % Rangos mayores o menores 
    busquedaG = X(:,:,2) > refG - umbral & X(:,:,2) < refG + umbral; % Rangos mayores o menores
    busquedaB = X(:,:,3) > refB - umbral & X(:,:,3) < refB + umbral; % Rangos mayores o menores

    %% Resultado RGB
    busqueda = busquedaR.*busquedaG.*busquedaB;
    % busqueda = medfilt2(busqueda);

    %% Centroides y diferencias
    [posy, posx] = find(busqueda == 1);
    cy = sum(posy)/sum(busqueda(:));
    cx = sum(posx)/sum(busqueda(:));

    diferenciax = 320 - cx;
    diferenciay = 240 - cy;
    fprintf('Diferencia en x: %f \n Diferencia en y: %f \n', diferenciax, diferenciay);
    
    %% Fondo negro a la imagen segmentada
    for i = 1 : 3
        busqRGB(:,:,i) = X(:,:,i).*busqueda; % Fondo gris
    end

    %% Mostrar imagen segmentada
    figure(1);
    imshow(busqRGB);
    hold on
    plot(cx, cy, 'or');
    plot(320, 240, 'ob');
    hold off
%     figure(2);
%     imshow(Y);

    figure(3)
    imshow(X)
    
    %% Capturar nueva imagen
    snapshot1 = getsnapshot(v);
    X = im2double(snapshot1);
end
stop(v)

%% IMPORTANTE: USAR SIEMPRE AL FINALIZAR EL CÓDIGO
delete(v)
clear v