temp=zeros(215,10);

 temp(:,i)=Xnode(1:215);
for i = 1:10
    temp(:,i)=Xnode((i-1)*215+1:i*215);
end

Xnode_edge=temp;