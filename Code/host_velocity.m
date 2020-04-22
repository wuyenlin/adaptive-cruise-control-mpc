function vh = host_velocity(v0, T_total)

vh = v0 * ones(1, T_total);

% for i = 1:T_total
%     
%     if i < (1/5*T_total) % stay at v0 
%         vh(i) = v0;
%     
%     elseif i>= (1/5*T_total) && i < (2/5*T_total) % accelerate
%         vh(i) = vh(i-1) + 0.5*i;
%         v_max = vh(i);
%     
%     elseif i>= (2/5*T_total) && i < (3/5*T_total) % stay at high speed
%         vh(i) = v_max;
%         
%     elseif i>= (3/5*T_total) && i < (4/5*T_total) % decel
%         vh(i) = vh(i-1) - 0.25*i;
%     
%         if vh(i) <= v0 % decel to v0 and stay const
%             vh(i:end) = v0;
%             break;
%         end
%     end
% end