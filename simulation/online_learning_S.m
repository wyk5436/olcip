function online_learning_S(block)

setup(block);
  
function setup(block)

  % Register the number of ports.
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 1;
  
  % Set up the port properties to be inherited or dynamic.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;

  block.InputPort(1).Complexity   = 'Real'; 
  block.InputPort(1).DataTypeId   = 0;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(1).Dimensions   = 4;  
  
  block.InputPort(2).Complexity   = 'Real'; 
  block.InputPort(2).DataTypeId   = 0;
  block.InputPort(2).SamplingMode = 'Sample';
  block.InputPort(2).Dimensions   = 1;  

  block.OutputPort(1).Complexity   = 'Real';
  block.OutputPort(1).DataTypeId   = 0;
  block.OutputPort(1).SamplingMode = 'Sample';
  block.OutputPort(1).Dimensions   = 1;

  block.SampleTimes = [-1 0];
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
  block.RegBlockMethod('Update', @Update);
  


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
  block.NumDworks = 3;
  
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
  
  block.Dwork(3).Name            = 'control_gain';
  block.Dwork(3).Dimensions      = 4;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction


function Outputs(block)
  K1 = 0.2065;
  J = 0.0076;
  l = 0.337;
  r = 0.216;
  B = [0; 0; K1/J;-r*K1/(J*l)];
  Q = diag([5 30 0 0]);
  R = 1;
  dt = 0.01;
  
  i = block.Dwork(2).Data;
  state = block.Dwork(1).Data;
  x = block.InputPort(1).Data;
  if (block.InputPort(2).Data == 3)
      idx = (i-1)*4 + 1;
      state_data = state(1:(idx + 3));
      mat = reshape(state_data,[4,i]);
      X = mat(:,1:i-1);
      Y = mat(:,2:end);
      Am = Y*pinv(X);
      
      sys = ss(zeros(4),B,eye(4),0);
      dis_sys = c2d(sys,dt);
      disc_B = dis_sys.B;
      K = dlqr(Am,disc_B,Q,R);
      block.Dwork(3).Data = K;
      block.OutputPort(1).Data = -K*x;
  elseif (block.InputPort(2).Data > 3)
      gain = block.Dwork(3).Data;
      gain = reshape(gain,[1,4]);
      block.OutputPort(1).Data = -gain*x;
  end
  
%endfunction

function Update(block)
  if (block.InputPort(2).Data > 2) && (block.InputPort(2).Data < 3)
      block.Dwork(2).Data = block.Dwork(2).Data + 1;
      i = block.Dwork(2).Data;
      state = block.Dwork(1).Data;
      x = block.InputPort(1).Data;

      if i <= 2000
          idx = (i-1)*4 + 1;
          state(idx:(idx+3)) = x;
          block.Dwork(1).Data = state;
          state_data = state(1:idx + 3);
      else
          state(1:1997) = state(4:2000);
          state(1997:2000) = x;
          block.Dwork(1).Data = state;
      end
  end
  
%endfunction

