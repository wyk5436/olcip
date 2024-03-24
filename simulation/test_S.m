function test_S(block)

setup(block);
  
function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 1;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Complexity   = 'Real'; 
  block.InputPort(1).DataTypeId   = 0;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(1).Dimensions   = 4;   

  block.OutputPort(1).Complexity   = 'Real';
  block.OutputPort(1).DataTypeId   = 0;
  block.OutputPort(1).SamplingMode = 'Sample';
  block.OutputPort(1).Dimensions   = 1;

  block.SampleTimes = [-1 0];
%   block.SetAccelRunOnTLC(true);
  block.OperatingPointCompliance = 'Default';
  
  
  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);
  block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  % -----------------------------------------------------------------
  % Register methods called at run-time
  % -----------------------------------------------------------------
  block.RegBlockMethod('Outputs', @Outputs);
%   block.RegBlockMethod('Update', @Update);

function SetInpPortFrameData(block, idx, fd)
  
  block.InputPort(idx).SamplingMode = fd;
  block.OutputPort(1).SamplingMode  = fd;
  
%endfunction

function SetInpPortDims(block, idx, di)
  
  block.InputPort(idx).Dimensions = di;
  block.OutputPort(1).Dimensions  = di;
  
%endfunction

function SetOutPortDims(block, idx, di)
  
  block.OutputPort(idx).Dimensions = di;
  block.InputPort(1).Dimensions    = di;

%endfunction

function SetInpPortDataType(block, idx, dt)
  
  block.InputPort(idx).DataTypeID = dt;
  block.OutputPort(1).DataTypeID  = dt;

%endfunction
  
function SetOutPortDataType(block, idx, dt)

  block.OutputPort(idx).DataTypeID  = dt;
  block.InputPort(1).DataTypeID     = dt;

%endfunction  

    
function DoPostPropSetup(block)
  block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'state';
  block.Dwork(1).Dimensions      = 4*2000;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'time_called';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction


function Outputs(block)
  block.Dwork(2).Data = block.Dwork(2).Data + 1;
  i = block.Dwork(2).Data;
  state = block.Dwork(1).Data;
  x = block.InputPort(1).Data;
  mat = 0;
  
  if i <= 2000
      idx = (i-1)*4 + 1;
      state(idx:(idx+3)) = x;
      block.Dwork(1).Data = state;
      state_data = state(1:idx + 3);
      mat = reshape(state_data,[4,i]);
  else
      state(1:1997) = state(4:2000);
      state(1997:2000) = x;
      block.Dwork(1).Data = state;
      mat = reshape(state,[4,2000]);
  end
  
  u = mat(:,end);
  u = u';
  
  block.OutputPort(1).Data = u*x;
  
%endfunction

% function Update(block)
%   
%   block.Dwork(1).Data = block.InputPort(1).Data;
%   
% %endfunction
