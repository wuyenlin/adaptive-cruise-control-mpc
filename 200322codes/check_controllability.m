function check_controllability(A,B)
    disp('Checking controllability of the given matrices');
    rank_val = rank(ctrb(A,B));
    if rank(A) == rank_val
        disp('The system A,B is controllable!');
    else
        disp('The system is not controllable!');
    end
end