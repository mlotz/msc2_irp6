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
xz1=table2array(T(2:end,4));
yz1=table2array(T(2:end,5));
zz1=table2array(T(2:end,6));

xc1=table2array(T(2:end,9));
yc1=table2array(T(2:end,10));
zc1=table2array(T(2:end,11));

xz2=table2array(T(2:end,14));
yz2=table2array(T(2:end,15));
zz2=table2array(T(2:end,16));

xc2=table2array(T(2:end,19));
yc2=table2array(T(2:end,20));
zc2=table2array(T(2:end,21));


f(2) = figure(2);
hold on;
title('1pomiary w osi X [N]');
xlabel('czas [s]');
ylabel('Fx_z, Fx_c');
plot(seq,xz1);
plot(seq,xc1);

f(3) = figure(3);
hold on;
title('1pomiary w osi Y [N]');
xlabel('czas [s]');
ylabel('Fy_z, Fy_c');
plot(seq,yz1);
plot(seq,yc1);

f(4) = figure(4);
hold on;
title('1pomiary w osi Z [N]');
xlabel('czas [s]');
ylabel('Fz_z, Fz_c');
plot(seq,zz1);
plot(seq,zc1);

f(5) = figure(5);
hold on;
title('2pomiary w osi X [N]');
xlabel('czas [s]');
ylabel('Fx_z, Fx_c');
plot(seq,xz2);
plot(seq,xc2);

f(6) = figure(6);
hold on;
title('2pomiary w osi Y [N]');
xlabel('czas [s]');
ylabel('Fy_z, Fy_c');
plot(seq,yz2);
plot(seq,yc2);

f(7) = figure(7);
hold on;
title('2pomiary w osi Z [N]');
xlabel('czas [s]');
ylabel('Fz_z, Fz_c');
plot(seq,zz2);
plot(seq,zc2);


saveas(f(2),strcat(plotPath,'compensator1_x.pdf'))
saveas(f(3),strcat(plotPath,'compensator1_y.pdf'))
saveas(f(4),strcat(plotPath,'compensator1_z.pdf'))
saveas(f(5),strcat(plotPath,'compensator2_x.pdf'))
saveas(f(6),strcat(plotPath,'compensator2_y.pdf'))
saveas(f(7),strcat(plotPath,'compensator2_z.pdf'))

fclose('all');
close all;



