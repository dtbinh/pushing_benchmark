classdef Surface < dynamicprops
    properties (Constant)
    end
    
    properties
        nu;
    end
    
    methods
        function obj = Surface(nu)  
            obj.nu = nu;
        end
    end
    
end