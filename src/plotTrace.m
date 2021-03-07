function plotTrace(quad,xh,uh,th,rh,loop)


    
    figure(1)

    clf; view(3);
    hold on; grid on;

    pos = xh(10:12,:);
    plot3(pos(1,:), pos(2,:), pos(3,:), 'g', 'linewidth', 1);
    %plot3(rh(1,:), rh(2,:), rh(3,:), 'k', 'linewidth', 1);

    for i = 1:5:size(th,2)
        quad.plot_point(xh(:,i),uh(:,i));
    end
    
    title('Successful recovery from inverted position')
    xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
    
    axis equal
    axis vis3d
        set(gcf,'color','w');
    
    
        if nargin > 5
            xlim([-20 20])
            ylim([-80 10])
            zlim([-120 30])
        else
            xlim([-0.4 4])
            ylim([-0.4 2])
            zlim([-0.3 2])
        end
    
    
end