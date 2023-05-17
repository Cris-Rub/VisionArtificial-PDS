%% PROYECTO FINAL PROCESAMIENTO DIGITAL DE SEÑALES D01
%% INTEGRANTES DEL EQUIPO:
%% RUBIO SANCHEZ CRISTOPHER JESUS: 217639714
%% ARELLANO MADERA EDWIN OMAR: 217734504

clc;
clear all;
close all hidden;

%% CONEXIÓN A ARDUINO
p = arduino('COM3','UNO','Libraries','Servo');

%% CONEXIÓN A SERVOMOTORES
servo_x = servo(p,'D6');
servo_y = servo(p,'D9');

%% SET DE MOTORES A 90°
angulox = 90/180;
anguloy = 90/180;
writePosition(servo_x,angulox);
writePosition(servo_y,anguloy);

%% CONFIGURACIÓN CÁMARA
v = videoinput("winvideo", 1, "MJPG_640x480");
v.FramesPerTrigger = Inf;
v.ReturnedColorspace = "rgb";
set(v, 'TriggerFrameDelay', 5);
start(v);

%% OBTENCIÓN DE COLOR A SEGEMENTAR
recording1 = getdata(v, v.FramesAvailable);
implay(recording1);
pause('on')
pause(2)
snapshot1 = getsnapshot(v);
X = im2double(snapshot1);
[m, n, o] = size(X);
select = roipoly(X);
umbral = 30/255;

%% Referencias del color seleccionado
refR = sum(sum(X(:,:,1).*select))/sum(select(:));
refG = sum(sum(X(:,:,2).*select))/sum(select(:));
refB = sum(sum(X(:,:,3).*select))/sum(select(:));

%% Ganancias PID x
Kp_x = 0.5;
Ki_x = 0.0001;
Kd_x = 0.001;

%% Ganancias PID y
Kp_y = 0.35;
Ki_y = 0.0001;
Kd_y = 0.0004;

%% Errores para PID
errorAcumx = 0;
errorIntx = 0;
errorAcumy = 0;
errorInty = 0;

count = 0;

%% Vectores para plotear los errores
posicionVect_x = [];
posicionAntesVect_x = [];
posicionVect_y = [];
posicionAntesVect_y = [];
for frames=1:250
    tic;
    recording1 = getdata(v, v.FramesAvailable);
    X = X + 0.2 * randn(m, n, o); % Ruido a la imagen capturada

    %% Filtro pasa bajas (eliminar ruido)
    r = 300;
    sigma = 0.05;
    for z = 1 : o
        Xf(:,:,z) = fftshift(fft2(X(:,:,z)));
        H = zeros(m, n);
        for i = 1 : m
            dy = (i-m/2)/(m/2);
            for j = 1 : n
                dx = (j-n/2)/(n/2);
                dxy = sqrt(dx^2+dy^2);
                H(i, j) = exp(-(dxy^2)/(2*sigma^2));
            end
        end
        Yf(:,:,z) = Xf(:,:,z).*H; %Filtrado
        Y(:,:,z) = ifft2(ifftshift(Yf(:,:,z))); % Regreso al dominio espacial
    end

    %% Busqueda de color RGB
    busquedaR = Y(:,:,1) > refR - umbral & Y(:,:,1) < refR + umbral; % Rangos mayores o menores 
    busquedaG = Y(:,:,2) > refG - umbral & Y(:,:,2) < refG + umbral; % Rangos mayores o menores
    busquedaB = Y(:,:,3) > refB - umbral & Y(:,:,3) < refB + umbral; % Rangos mayores o menores

    %% Resultado RGB
    busqueda = busquedaR.*busquedaG.*busquedaB;

    %% Centroides y diferencias
    [posy, posx] = find(busqueda == 1);
    cy = sum(posy)/sum(busqueda(:));
    cx = sum(posx)/sum(busqueda(:));

    diferenciax = 320 - cx;
    diferenciay = 240 - cy;
    fprintf('Diferencia en x: %f \n Diferencia en y: %f \n', diferenciax, diferenciay);
    
    %% Fondo negro a la imagen segmentada
    for i = 1 : 3
        busqRGB(:,:,i) = Y(:,:,i).*busqueda;
    end

    %% Condiciones para setear angulos
    anguloAntesx = angulox;
    anguloAntesy = anguloy;
    
    if ~isnan(diferenciax) || ~isnan(diferenciay)
        count = 0;
        time_ex = toc;

        %% CALCULO COMPONENTES PID x
        Px = diferenciax * Kp_x;
        Ix = errorAcumx * time_ex * Ki_x;
        Dx = ((diferenciax - errorIntx)/time_ex) * Kd_x;

        errorAcumx = errorAcumx + diferenciax;
        errorIntx = diferenciax;
        controlx = Px + Ix + Dx;
        %% Movimiento en x
        if controlx > 10/180 || controlx < -10/180
            if diferenciax > 0
                angulox = (anguloAntesx + (controlx/16.1)/180) ; % IZQUIERDA
            else
                angulox = (anguloAntesx - (-controlx/16.1)/180) ; % DERECHA
            end
        end
        
        %% CALCULO COMPONENTES PID y
        Py = diferenciay * Kp_y;
        Iy = errorAcumy * time_ex * Ki_y;
        Dy = ((diferenciay - errorInty)/time_ex) * Kd_y;

        errorAcumy = errorAcumy + diferenciay;
        errorInty = diferenciay;
        controly = Py + Iy + Dy;
        %% Movimiento en y
        if controly > 10/180 || controly < -10/180
            if diferenciay > 0
                anguloy = (anguloAntesy - (controly/18)/180); % ARRIBA
            else
                anguloy = (anguloAntesy + (-controly/18)/180); % ABAJO
            end
        end
        
        %% Movimiento de motores
        writePosition(servo_x, angulox);
        writePosition(servo_y, anguloy);

    else
        count = count + 1;
        if count == 10
            angulox = 90/180;
            anguloy = 90/180;

            %% Movimiento de motores
            writePosition(servo_x, angulox);
            writePosition(servo_y, anguloy);
        end
    end

    %% Mostrar imagen segmentada
    figure(1);
    imshow(busqRGB);
    hold on
    plot(cx, cy, 'or');
    plot(320, 240, 'ob');
    hold off
    
%     figure(2)
%     imshow(X)
% 
%     figure(3)
%     imshow(Y)

    %% Capturar nueva imagen
    snapshot1 = getsnapshot(v);
    X = im2double(snapshot1);

    %% Vectores de error
    posicionVect_x(frames) = angulox;
    posicionAntesVect_x(frames) = anguloAntesx;
    posicionVect_y(frames) = anguloy;
    posicionAntesVect_y(frames) = anguloAntesy;
    % count_time = count_time + 1;
end
stop(v)

%% Graficas de error
figure(2)
hold on
plot(posicionVect_x, 'b');
plot(posicionAntesVect_x, 'r');
title('Grafica de error en eje x')
xlabel('Frames')
ylabel('Grados')
hold off

figure(3)
hold on
plot(posicionVect_y, 'b');
plot(posicionAntesVect_y, 'r');
title('Grafica de error en eje y')
xlabel('Frames')
ylabel('Grados')
hold off

%% IMPORTANTE: USAR SIEMPRE AL FINALIZAR EL CÓDIGO
delete(v)
clear v