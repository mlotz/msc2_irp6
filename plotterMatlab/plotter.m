clear all;
close all;

filename = '/home/mlotz/msc_ws_irp6/logs/reports.dat';
plotPath = '/home/mlotz/msc_ws_irp6/plots/';
docPath = '/home/mlotz/Praca_Magisterska/SVN/17-mlotz/ppmgr/images/';
delimiterIn = ' ';
headerlinesIn = 1;
A = importdata(filename,delimiterIn,headerlinesIn);
fontsize = 25;


xFN1 = A.data(:,1);
yFN1= A.data(:,2);
zFN1 = A.data(:,3);
t = str2double(A.textdata(2:end,1));
X = str2double(A.textdata(2:end,2));
Y = str2double(A.textdata(2:end,3));
% xF = str2double(A.textdata(2:end,17));
% yF = str2double(A.textdata(2:end,18));
% zF = str2double(A.textdata(2:end,19));
xFN2 = str2double(A.textdata(2:end,16));
yFN2 = str2double(A.textdata(2:end,17));
zFN2 = str2double(A.textdata(2:end,18));


% f1 = figure(1);
% hold on;
% title('Trajektoria XY');
% xlabel('X');
% ylabel('Y');
% plot(X,Y);
f(2) = figure(2);
hold on;
title('X w czasie');
xlabel('t');
ylabel('X');
plot(t,X);
f(3) = figure(3);
hold on;
title('Y w czasie');
xlabel('t');
ylabel('Y');
plot(t,Y);

%asd


f(4) = figure(4);
set(gca,'fontsize',fontsize)
hold on;
title('optForce1 - xFN1');
xlabel('czas[s]');
ylabel('xFN1[N]');
plot(t,xFN1);

f(5) = figure(5);
set(gca,'fontsize',fontsize)
hold on;
title('optForce1 - yFN1');
xlabel('czas[s]');
ylabel('yFN1[N]');
plot(t,yFN1);

f(6) = figure(6);
set(gca,'fontsize',fontsize)
hold on;
title('optForce1 - zFN1');
xlabel('t[s]');
ylabel('zFN1[N]');
plot(t,zFN1);

f(7) = figure(7);
hold on;
title('optForce2 - xFN2');
xlabel('t');
ylabel('xFN2');
plot(t,xFN2);

f(8) = figure(8);
hold on;
title('optForce2 - yFN2');
xlabel('t');
ylabel('yFN2');
plot(t,yFN2);

f(9) = figure(9);
hold on;
title('optForce2 - zFN2');
xlabel('t');
ylabel('zFN2');
plot(t,zFN2);

%  for i =2:9
%      set(f(i),'FontSize',20);
%  end

saveas(f(2),strcat(plotPath,'x.pdf'))
saveas(f(3),strcat(plotPath,'y.pdf'))
saveas(f(4),strcat(plotPath,'xfn1.pdf'))
saveas(f(5),strcat(plotPath,'yfn1.pdf'))
saveas(f(6),strcat(plotPath,'zfn1.pdf'))
saveas(f(7),strcat(plotPath,'xfn2.pdf'))
saveas(f(8),strcat(plotPath,'yfn2.pdf'))
saveas(f(9),strcat(plotPath,'zfn2.pdf'))

saveas(f(2),strcat(docPath,'x.pdf'))
saveas(f(3),strcat(docPath,'y.pdf'))
saveas(f(4),strcat(docPath,'xfn1.pdf'))
saveas(f(5),strcat(docPath,'yfn1.pdf'))
saveas(f(6),strcat(docPath,'zfn1.pdf'))
saveas(f(7),strcat(docPath,'xfn2.pdf'))
saveas(f(8),strcat(docPath,'yfn2.pdf'))
saveas(f(9),strcat(docPath,'zfn2.pdf'))

fclose('all');
close all;



