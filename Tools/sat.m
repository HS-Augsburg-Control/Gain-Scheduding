% Implementation of saturation function
%
% Autor: Klaus Kefferpütz (OPS3-F1)
% Datum: 10.11.2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, umax)
    
    out = zeros(size(in,1),1);
    for(ii=1:1:size(in,1))
        
        if(in(ii)>umax(ii))
            out(ii) = umax(ii);
        elseif(in(ii)<-umax(ii))
            out(ii) = -umax(ii);
        else
            out(ii) = in(ii);
        end
                
    end

end