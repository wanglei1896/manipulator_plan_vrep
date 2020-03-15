dotCloud_1_3 = calDotCloud_1_3(model,20);
dotCloud_4_6 = calDotCloud_4_6(model,20);

model.plot([0,0,0,0,0,0]);
hold on
plot3(dotCloud_1_3(1,:),dotCloud_1_3(2,:),dotCloud_1_3(3,:),'g.');
plot3(dotCloud_4_6(1,:),dotCloud_4_6(2,:),dotCloud_4_6(3,:),'r.');

function result = calDotCloud_1_3(manipulator,sample)
    result = zeros(3,sample,sample,sample);
    j1=linspace(-pi,pi,sample);
    j2=j1; j3=j1;
    for i=1:sample
        for j=1:sample
            for k=1:sample
                t=manipulator.A(1:3, [j1(i) j2(j) j3(k) 0 0 0]);
                result(:,i,j,k)=t.t;
            end
        end
    end
end
function result = calDotCloud_4_6(manipulator,sample)
    result = zeros(3,sample,sample);
    j4=linspace(-pi,pi,sample);
    j5=linspace(-pi,pi,sample);
    for i=1:sample
        for j=1:sample
            t=manipulator.A(1:6, [0 0 0 j4(i) j5(j) 0]);
            result(:,i,j)=t.t;
        end
    end
end