classdef Helper < dynamicprops
    properties (Constant)
      g = 9.81;
      n1 = [1;0;0];
      n2 = [0;1;0];
      n3 = [0;0;1];
    end
    
    properties

    end
    
    methods
        %% Constructor
        function obj = Helper
        end
        
    end
    methods(Static)
        %3d (proper) cross product operator  
        function cross = cross( x)
            cross = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
        end
        %%2d Cross operator used for 2d pushing
        function w = cross2d(r,v)
            r3 = [r;0];
            v3 = [v;0];
            
            r3_cross = [0 -r3(3) r3(2); r3(3) 0 -r3(1); -r3(2) r3(1) 0];
            w3 = r3_cross*v3;
            w = w3(3);
        end
        %%3d cross operator (return only first two elements)
        function w = cross3d(w,x)
            r = [0;0;w];
            v = [x;0];
            r_cross = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
            w3 = r_cross*v;
            w = w3(1:2);
        end
        
        function out=S2(r)
            out = [-r(2); r(1)];
        end
       
        %% Rotation Matrices
        function C1 = C1(x)
            C1 = [1 0 0; 0 cos(x) sin(x); 0 -sin(x) cos(x)];
        end
        
        function C2 = C2(x)
            C2 = [cos(x) 0 -sin(x); 0 1 0; sin(x) 0 cos(x)];
        end
        
        function C3 = C3(x)
            C3 = [ cos(x) sin(x) 0; -sin(x) cos(x) 0; 0 0 1 ];
        end
        
        function C3 = C3_2d(x)
            C3 = [ cos(x) sin(x); -sin(x) cos(x)];
        end
        
        function dM = jacobian_higher(Matrix, x)
            dM = {};
            for i=1:length(x)
                dM{i} = jacobian(Matrix, x(i));
            end    
        end
        %% Numerical Integration
        function integral = gaussQuad(fun,a,b)
            %Change of Variables
            h1 = (b-a)/2;
            h2 = (b+a)/2;
            %Define Weights (for 3 points)
            w(1) =  0.5688888888888889;
            w(2) =  0.4786286704993665;
            w(3) =  0.4786286704993665;
            w(4) =  0.2369268850561891;
            w(5) =  0.2369268850561891;
            %Define Points
            x(1) =  0.0000000000000000;
            x(2) =  -0.5384693101056831;
            x(3) =  0.5384693101056831;
            x(4) =  -0.9061798459386640;
            x(5) =  0.9061798459386640;
            %gaussian integration
            sum = 0;
            for lv1=1:5
                sum = sum+w(lv1)*fun(h1*x(lv1)+h2);
            end
            integral = h1 * sum;
        end
        %% Numerical Integration
        function integral = DoubleGaussQuad(fun1,a,b,c,d)
            %Change of Variables
            h1 = (b-a)/2;
            h2 = (b+a)/2;
            h3 = (d-c)/2;
            h4 = (d+c)/2;

            %Define Weights (for 3 points)
            w1 = 1;
            w2 = 1;

            %Define Points
            x1 = sqrt(1/3);
            x2 = -sqrt(1/3);

            integral = h1 * h3 * (w1*w1*fun1(h1*x1+h2, h3*x1+h4) + w1*w2*fun1(h1*x1+h2, h3*x2+h4) +...
                w2*w1*fun1(h1*x2+h2, h3*x1+h4) + w2*w2*fun1(h1*x2+h2, h3*x2+h4) );
        end
        
        %% Smooth function
        function smoothedVal = smooth(obj, data, filterVal, smoothedVal)
            smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
        end
        
    end
end