function MRP = qtoMRP(q,which)

if which == 1
%scalar first
    MRP = (1/(1+q(1)))*[q(2);q(3);q(4)];
elseif which == 2
%vector first
    MRP = (1/(1+q(4)))*[q(1);q(2);q(3)];
else
    error('1 is scalar first, 2 is vector first')
end
end