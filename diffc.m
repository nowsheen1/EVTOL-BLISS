
    function cd= diffc(aT,aR)
        %cd=-(aT-aR)/(aT) ; %target - response
        cd=(aT-aR).^2 ;
%         c=abs(c);
    end
