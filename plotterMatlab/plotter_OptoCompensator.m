clear all;
close all;

filename = '/home/mlotz/msc2_ws_irp6/logs/reports.dat';
plotPath = '/home/mlotz/msc2_ws_irp6/plots/';
docPath = '/home/mlotz/msc2_ws_irp6/dummy/';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
T = readtable(filename);
fontsize = 25;



seq = table2array(T(2:end,1));
xz=table2array(T(2:end,4));
yz=table2array(T(2:end,5));
zz=table2array(T(2:end,6));

xc=table2array(T(2:end,9));
yc=table2array(T(2:end,10));
zc=table2array(T(2:end,11));


f(2) = figure(2);
hold on;
title('pomiary w osi X [N]');
xlabel('czas [s]');
ylabel('Fx_z, Fx_c');
plot(seq,xz);
plot(seq,xc);

f(3) = figure(3);
hold on;
title('pomiary w osi Y [N]');
xlabel('czas [s]');
ylabel('Fy_z, Fy_c');
plot(seq,yz);
plot(seq,yc);

f(4) = figure(4);
hold on;
title('pomiary w osi Z [N]');
xlabel('czas [s]');
ylabel('Fz_z, Fz_c');
plot(seq,zz);
plot(seq,zc);


%saveas(f(2),strcat(plotPath,'compensator_x.pdf'))
%saveas(f(3),strcat(plotPath,'compensator_y.pdf'))
%saveas(f(4),strcat(plotPath,'compensator_z.pdf'))


fclose('all');
%close all;



