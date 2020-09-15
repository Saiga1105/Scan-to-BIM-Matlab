function [x] = F_optimisecontrolpoints(Mx,My,Px,Py,dw)
%F_OPTIMISECONTROLPOINTS adjust controlpoints of the fitted function with respect to the wall observations and the wall thickness 
%   mx = input X coordinate control point list
%   my = input Y coordinate control point list
%   Px= X coordinate list observations
%   Py= Y coordinate list observations
%   dw= wall thickness
%   Mx= adjusted X coordinate list
%   My= adjusted Y coordinate list

% find neighbors in range dw from [mx
N=[Mx My];
x=zeros(length(Mx),2);
P=[Px Py];
for i=1:length(N)
    %syms x y 
    [Idx,~] = rangesearch(P,N(i,:),dw/2);
    C=P(cell2mat(Idx),:);
    %theta=zeros(length(C),1)';
    A=zeros(2*length(C)+2,2);
    A(1:2:end-1,1)=1;
    A(2:2:end,2)=1;
    A(1,3)=-N(i,1);
    A(2,3)=-N(i,2);
    for j=1:length(C)
        A(2*j+1,3)= -C(j,1); 
        A(2*j+1,3+j)=dw/2*acos(1);
        A(2*j+2,3)=-C(j,2); 
        A(2*j+2,3+j)=dw/2*asin(1);
    end
    b=zeros(size(A,1),1);
    b(1)=N(i,1);
    b(2)=N(i,2);
    b(3:2:end-1,1)=C(:,1);
    b(4:2:end,1)=C(:,2);
    %b=b-dw*0.5;
    %b(1)=0;b(2)=0;
    %x(i,:)=(b\A)';
    y=(b\A)
    X = linsolve(A,b);
    
    syms x y;
    eqn2 = (x-C(:,1)).^2+(y-C(:,2)).^2 == (dw/2)^2;
    eqn0 = x-N(i,1)==0;
    eqn1= y--N(i,2)==0;
    [A,B] = equationsToMatrix([eqn0 eqn1 eqn2], [x, y])
    S=solve([eqn0 eqn1 eqn2],[x y])

end

end

