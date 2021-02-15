organized_H_position=zeros(384,384);
H_coordination=zeros(2000,2);
i=2049001;p=1;
for j=1:384
    for k=1:384
        organized_H_position(j,k)=map_data_H{116,1}.Data(i,1);
        if organized_H_position(j,k)==100
            H_coordination(p,1)=j;
            H_coordination(p,2)=k;
            p=p+1;
        end
        i=i+1;         
     end
end