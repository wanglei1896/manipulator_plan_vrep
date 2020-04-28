function calculate_shapeCentre_test()
    v1=[1 0 0 0 0 -1;
        0 1 0 0 -1 0;
        0 0 1 -1 0 0;];
    hold on
    plot3(v1(1,:),v1(2,:),v1(3,:))
    cen1=calculate_shapeCentre(v1);
    plot3(cen1(1),cen1(2),cen1(3),'o')
end

