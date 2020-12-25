function animate(quad,xh,uh,th,rh)


    for i = 1:size(th,2)
        figure(1)

        clf; view(3);
        hold on; grid on;

        pos = xh(10:12,:);
        plot3(pos(1,:), pos(2,:), pos(3,:), 'g', 'linewidth', 1);
        plot3(rh(1,:), rh(2,:), rh(3,:), 'k', 'linewidth', 1);

        quad.plot_point(xh(:,i),uh(:,i));

        axis equal
        axis vis3d
        
        xlim([-0.4 4])
        ylim([-0.4 2])
        zlim([-0.3 2])
        
        pause(0.01)
    
    end
end