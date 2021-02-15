standard_coordination=G1_coordination;
test_coordination=G2_coordination;
k=0;
sum_distance=0;
N=1;i=1;j=1;
ADNN = zeros(100,1);
for k=-50:1:50
    for i=1:2000
        delta_d = distance(standard_coordination(i,1)+k,standard_coordination(i,2)+k,test_coordination(i,1),test_coordination(i,2));
        if delta_d < 10
            delta_d = 2;
            sum_distance = sum_distance + delta_d;
            N =N+1;
        end
    if standard_coordination(i,1) == 0
        break
    end  
    end
    sum_distance
    ADNN(j) = sum_distance/N;
    sum_distance=0;
    j=j+1;
end

translation_i=-10:1:10;

figure
plot(ADNN)

title('ADNN - translation_i图像')
xlabel('translation_i')
ylabel('ADNN')
min(ADNN)