classdef Square < dynamicprops
    properties (Constant)
        a = 0.09;
        b = 0.09;
        rho = 10000;
        height = 0.013;
        xo = sym('xo', [3,1]); %x = [x;y;theta]
    end
    
    properties
       c;
       m_max;
       f_max; 
       A_ls;
       m;
       V; 
       A; 
       d;
    end
    
    methods
        %% Constructor
        function obj = Square() 
            obj.initialize_properties();
        end
        function obj = initialize_properties(obj)
            %Set constant equations
            obj.A = obj.a*obj.b;
            obj.V = obj.A*obj.height;
            obj.m = obj.rho*obj.V;
        end

    end
end