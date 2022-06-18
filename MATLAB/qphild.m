function eta=qphild(H,f,A,b)
% finds the global optimal solution and checks if all the con-
% straints are satisfied. If so, the program returns the optimal solution η. If
% not, the program then begins to calculate the dual variable λ

% F=f;
% M=A;
% gamma=b;
% eta =x
[n1,~]=size(A);
eta=-H\f;
kk=0;
for i=1:n1
    if (A(i,:)*eta>b(i))
        kk=kk+1;
    else
        kk=kk+0;
    end
end
if (kk==0)
    return;
end
P=A*(H\A');
d=(A*(H\f)+b);
[n,m]=size(d);
x_ini=zeros(n,m);
lambda=x_ini;
%al=10;
for km=1:128
    %find the elements in the solution vector one by one
    % km could be larger if the Lagranger multiplier has a slow
    % convergence rate.
    lambda_p=lambda;
    for i=1:n
        w= P(i,:)*lambda-P(i,i)*lambda(i,1);
        w=w+d(i,1);
        la=-w/P(i,i);
        lambda(i,1)=max(0,la);
    end
    al=(lambda-lambda_p)'*(lambda-lambda_p);
    if (al<10e-12)
        break;
    end
end

eta=-H\f -H\A'*lambda;


