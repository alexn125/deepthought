function out = DCM_MRP(which,input)

%for going between DCM and MRP, from Schaub Junkins 3rd ed

if strcmp(which,'MRPtoDCM') == 1
    s = input;

    s2 = s'*s;
    sk = skew(s);

    out = eye(3,3) + (1/((1+s2)^2))*(8*sk*sk - 4*(1-s2)*sk);

elseif strcmp(which,'DCMtoMRP') == 1
    C = input;

    quat = MRPtoq(C);

    out1 = quat;
    out2 = (1/(1+quat(1)))*[quat(2);quat(3);quat(4)]; 
    
    out = [out1;out2];

    % t = sqrt(trace(C)+1);
    %
    % out = (1/(t^2 + 2*t))*[C(2,3)-C(3,2);C(3,1)-C(1,3);C(1,2)-C(2,1)];

else
    error('check inputs')
end

    function a_s = skew(a)
        a_s = [0 -a(3) a(2); a(3) 0 -a(1);-a(2) a(1) 0];
    end

    function quat = MRPtoq(C)

        T = trace(C);
        b2(1) = (1+T)/4;
        b2(2) = (1+2*C(1,1)-T)/4;
        b2(3) = (1+2*C(2,2)-T)/4;
        b2(4) = (1+2*C(3,3)-T)/4;

        [~,i] = max(b2);
        switch i
            case 1
        		b(1) = sqrt(b2(1));
        		b(2) = (1/(4*b(1)))*(C(2,3)-C(3,2));
        		b(3) = (1/(4*b(1)))*(C(3,1)-C(1,3));
        		b(4) = (1/(4*b(1)))*(C(1,2)-C(2,1));
        	case 2
        		b(2) = sqrt(b2(2));
        		b(1) = (1/(4*b(2)))*(C(2,3)-C(3,2));
        		if (b(1)<0)
        			b(2) = -b(2);
        			b(1) = -b(1);
        		end
        		b(3) = (1/(4*b(2)))*(C(1,2)+C(2,1));
        		b(4) = (1/(4*b(2)))*(C(3,1)+C(1,3));
        	case 3
        		b(3) = sqrt(b2(3));
        		b(1) = (1/(4*b(3)))*(C(3,1)-C(1,3));
        		if (b(1)<0)
        			b(3) = -b(3);
        			b(1) = -b(1);
        		end
        		b(2) = (1/(4*b(3)))*(C(1,2)+C(2,1));
        		b(4) = (1/(4*b(3)))*(C(2,3)+C(3,2));
        	case 4
        		b(4) = sqrt(b2(4));
        		b(1) = (1/(4*b(4)))*(C(1,2)-C(2,1));
        		if (b(1)<0)
        			b(4) = -b(4);
        			b(1) = -b(1);
        		end
        		b(2) = (1/(4*b(4)))*(C(3,1)+C(1,3));
        		b(3) = (1/(4*b(4)))*(C(2,3)+C(3,2));
        end
        quat = b';
    end
end