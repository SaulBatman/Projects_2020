test_coordination=zeros(2000,2);
for i=1:2000
    if data_coordination(i,1) == 0
        break
    end
    test_coordination(i,1) = data_coordination(i,1) + 1;
    test_coordination(i,2) = data_coordination(i,2) + 1;
    
end
