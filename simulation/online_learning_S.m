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
  block.NumDworks = 7;
  
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
  
  block.Dwork(4).Name            = 'control';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name            = 'xkp1';
  block.Dwork(5).Dimensions      = 4;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  block.Dwork(6).Name            = 'Am';
  block.Dwork(6).Dimensions      = 16;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
  
  block.Dwork(7).Name            = 'P';
  block.Dwork(7).Dimensions      = 16;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
  
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
  disc_B = B*dt;
  

  state = block.Dwork(1).Data;
  x = block.InputPort(1).Data;
  
  if (block.InputPort(2).Data == 3)
      block.Dwork(2).Data = block.Dwork(2).Data + 1;
      i = block.Dwork(2).Data;
      idx = (i-1)*4 + 1;
      state(idx:(idx+3)) = x;
      block.Dwork(1).Data = state;    
      state_data = state(1:(idx + 3));
      mat = reshape(state_data,[4,i]);
      X = mat(:,1:i-1);
      Y = mat(:,2:end);
      
      xkp1 = Y(:,end);
      block.Dwork(5).data = xkp1;
      
      Am = Y*pinv(X);
      block.Dwork(6).data = reshape(Am,[16,1]);
      P = inv(X*X');
      block.Dwork(7).data = reshape(P,[16,1]);
      
      K = dlqr(Am,disc_B,Q,R);
      block.Dwork(3).Data = K;
      
      u = -K*x;
      block.Dwork(4).data = u;
      block.OutputPort(1).Data = u;
  elseif (block.InputPort(2).Data > 3)
      xkp1 = block.Dwork(5).Data;
      xkp1 = reshape(xkp1,[4,1]);
      
      P = block.Dwork(7).Data;
      P = reshape(P,[4,4]);
      
      gamma = 1/(1 + xkp1'*P*xkp1);
      
      uk = block.Dwork(4).data;
      ykp1 = x - disc_B*uk;
      
      At = block.Dwork(6).Data;
      At = reshape(At,[4,4]);
      At = At + gamma*(ykp1 - At*xkp1)*xkp1'*P;
      
      P = P - gamma*P*(xkp1*xkp1')*P;
      K = dlqr(At, disc_B,Q,R);
      
      xkp1 = x;
      block.Dwork(5).Data = xkp1;
      block.Dwork(6).data = reshape(At,[16,1]);
      block.Dwork(7).data = reshape(P,[16,1]);
      block.Dwork(3).Data = K;
      
      u = -K*x;
      block.Dwork(4).data = u;
      
      block.OutputPort(1).Data = u;
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

