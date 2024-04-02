function [thetalist, success] = IKinBodyIterations(Blist, M, T, thetalist0, eomg, ev)

thetalist = thetalist0;
i = 0;
maxiterations = 20;
%Added an iteration matrix that contains all the thetalists inlcuding the
%intial guess
iterationMatrix = thetalist';

Vb = se3ToVec(MatrixLog6(TransInv(FKinBody(M, Blist, thetalist)) * T));
err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
while err && i < maxiterations
   
    thetalist = thetalist + pinv(JacobianBody(Blist, thetalist)) * Vb;
    %The iteration matrix grows each step with a new list
    iterationMatrix = [iterationMatrix; thetalist'];
    
    i = i + 1;
    Tsb = FKinBody(M, Blist, thetalist);
    Vb = se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    err = norm(Vb(1: 3)) > eomg || norm(Vb(4: 6)) > ev;
    
    %Iteration report that prints what is happening each step
    fprintf('Iteration Report: \n')
    fprintf('iteration number (i) = %i \n',i)
    fprintf('joint vector (thetalist) = '); thetalist
    fprintf('\nend-effector config. (Tsb) = '); Tsb     
    fprintf('\nerror twist (Vb) = '); Vb
    fprintf('\nerror magnitude wb = '); norm(Vb(1: 3))
    fprintf('\nerror magnitude vb = '); norm(Vb(4: 6))
    fprintf('\n\n')
     
end
success = ~ err;
%Write the iteration matrix to a csv file
%csvwrite('iterationMatrix.csv',iterationMatrix);
end