function [xs_kf,xs_vo,ts_kf,ts_vo] = GenerateXYPlot(filename,heading_xaxis)

drawArrow = @(x,y,varargin) quiver( x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );

[xs_vo, ts_vo, dataVO] = VO(filename);
[xs_kf, Ps_kf, ts_kf, dataGPS] = KF(filename,heading_xaxis);


% calculate heading for VO
arrow_vo = zeros(size(ts_vo,2),2);
for i = 1:size(ts_vo,2)
   [~,~,yaw] = QuaternionToEuler(xs_vo(1:4,i));
   arrow_vo(i,1) = cos(yaw);
   arrow_vo(i,2) = sin(yaw);
end

% calculate heading for KF
arrow_kf = zeros(size(ts_kf,2),2);
for i = 1:size(ts_kf,2)
   [~,~,yaw] = QuaternionToEuler(xs_kf(1:4,i));
   arrow_kf(i,1) = cos(yaw);
   arrow_kf(i,2) = sin(yaw);
end

hold on;
plot(xs_vo(9,:),xs_vo(8,:)); % position in xy plane. x-axis=>up, y-axis=>right
plot(xs_kf(9,:),xs_kf(8,:)); % position in xy plane. x-axis=>up, y-axis=>right
scatter(dataGPS(1:10:end,2),dataGPS(1:10:end,1),'+');

% draw arrow for VO
scale = 3;
for i = 1:30*5:size(ts_vo,2)
   arrowx = [xs_vo(8,i) xs_vo(8,i)+arrow_vo(i,1)*scale];
   arrowy = [xs_vo(9,i) xs_vo(9,i)+arrow_vo(i,2)*scale];
   drawArrow(arrowy,arrowx,'MaxHeadSize',3,'linewidth',1,'color','b');
end
% draw arrow for KF
for i = 1:128*5:size(ts_kf,2)
   arrowx = [xs_kf(8,i) xs_kf(8,i)+arrow_kf(i,1)*scale];
   arrowy = [xs_kf(9,i) xs_kf(9,i)+arrow_kf(i,2)*scale];
   drawArrow(arrowy,arrowx,'MaxHeadSize',3,'linewidth',1,'color','r');
end

hold off;


grid minor;
axis equal;
legend('VO','KF');

end