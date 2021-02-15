position_G2=zeros(384,384);
G2_coordination=zeros(2000,2);
i=1;p=1;
for j=1:384
    for k=1:384
        position_G2(j,k)=map_data_G2{88,1}.Data(i,1);
        if position_G2(j,k)==100
            G2_coordination(p,1)=j;
            G2_coordination(p,2)=k;
            p=p+1;
        end
        i=i+1;         
     end
end