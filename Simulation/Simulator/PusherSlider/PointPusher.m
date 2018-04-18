classdef PointPusher < dynamicprops 
    % Contains all object properties
    properties (Constant)
        %pusher parameters
        xp = sym('xp', [2,1]); %xp = [xp;yp]
        uc = sym('uc', [3,1]); %xp = [fn;ft;dry]
        d = 0; %dummy value for compatibility with line pusher
        num_contacts = 1;
    end
    
    properties
        fn;
        ft;
        dry;
        nu_p;
    end
   
    methods
        %% Constructor
        function obj = PointPusher(nu_p)  
            obj.fn = obj.uc(1);
            obj.ft = obj.uc(2);
            obj.dry = obj.uc(3);
            obj.nu_p=nu_p;
        end
    end
end