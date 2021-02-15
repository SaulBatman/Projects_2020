function d = distance(X_1,Y_1,X_2,Y_2)
    X_diff = abs(X_1-X_2);
    Y_diff = abs(Y_1-Y_2);
    d = (X_diff^2+Y_diff^2)^0.5;
end