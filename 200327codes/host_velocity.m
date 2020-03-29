function vh = host_velocity(v0,T_total)
for i = linspace(1,T_total,5)
    if i < (1/5*T_total)
        vh = v0;
    elseif i>= (1/5*T_total) && i < (2/5*T_total)
        vh = vh + 0.5*i;
    elseif i>= (2/5*T_total) && i < (3/5*T_total)
        vh = vh - 0.5*i;
        if vh == v0
            break;
        end
    end
end