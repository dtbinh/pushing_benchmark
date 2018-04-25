classdef LinePusher < dynamicprops 
    % Contains all object properties
    properties (Constant)
        %pusher parameters
%         nu_p = 0.3;
        lp = 0.030;
        xp = sym('xp', [3,1]); %xp = [xp;yp;thetap]
        uc = sym('uc', [5,1]); %xp = [fn1;fn2;ft1;ft2;dry]
        num_contacts = 2;
    end
    
    properties
        fn;
        ft;
        dry;
        d;
        nu_p;
    end
   
    methods
        %% Constructor
        function obj = LinePusher(nu_p)  
            obj.nu_p=nu_p;
            obj.d = obj.lp/2;
            obj.fn = obj.uc(1:2);
            obj.ft = obj.uc(3:4);
            obj.dry = obj.uc(5);
        end
    end
end