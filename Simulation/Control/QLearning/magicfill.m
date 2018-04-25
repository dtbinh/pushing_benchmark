function varargout = magicfill
   nOutputs = nargout;
   varargout = cell(1,nOutputs);

   for k = 1:nOutputs
      varargout{k} = magic(k);
   end