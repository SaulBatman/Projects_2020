position_0=zeros(384,384);
standard_coordination=zeros(2000,2);
sum_distance=0;

for i=1:2000
    standard_coordination(i,1)=Coordination_X;
    standard_coordination(i,2)=Coordination_Y;
    for distance_i=1:5
        X_search = 0-distance_i:1:distance_i;
        Y_search = 0-distance_i:1:distance_i;
        Coordination_X2=Coordination_X-distance_i;
        Coordination_Y2=Coordination_Y-distance_i;
        if data_coordination(Coordination_X2,Coordination_Y2)==100
            sum_distance = sum_distance+ distance(Coordination_X2,Coordination_X,Coordination_Y2,Coordination_Y);
        elseif 
            
            
    
    
end
    