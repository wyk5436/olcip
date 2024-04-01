function online_learning_S(block)

    setup(block);

function [Am,P,xkp1] = init_dmd(data)
    i = length(data);
    X = data(:,1:i-1);
    Y = data(:,2:end);
    
    xkp1 = Y(:,end);
    Am = Y*pinv(X);
    P = inv(X*X');

function [At,Pt,xkp1t] = online_dmd_update(A,P,xkp1,x,uk) 
    %{
    A: A matrix at the previous time step
    xkp1: x_{k+1} paramete for online DMD.
    P: P matrix at the previous time step   
    uk: the control at the previous step.
    x: The state at this time step, resulting from uk
    %}

    K1 = 0.2065;
    J = 0.0076;
    l = 0.337;
    r = 0.216;
    B = [0; 0; K1/J;-r*K1/(J*l)];
    dt = 0.01;
    disc_B = B*dt;

    gamma = 1/(1 + xkp1'*P*xkp1);
    ykp1 = x - disc_B*uk;
    
    At = A + gamma*(ykp1 - A*xkp1)*xkp1'*P;
    Pt = P - gamma*P*(xkp1*xkp1')*P;
    xkp1t = x;
  
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

  block.SampleTimes = [0.01 0];
  block.OperatingPointCompliance = 'Default';
  
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
  block.RegBlockMethod('Start', @Start);
  block.RegBlockMethod('Update', @Update);
  block.RegBlockMethod('Outputs', @Outputs);
 
%endfunction

    
function DoPostPropSetup(block)
  block.NumDworks = 8;
  
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

  block.Dwork(8).Name            = 'flag';
  block.Dwork(8).Dimensions      = 1;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction

function Start(block)
  block.Dwork(4).Data = 0;
  block.Dwork(8).Data = 0;

%endfunction

function Update(block)
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

  if (block.InputPort(2).Data > 2) && (block.InputPort(2).Data < 3) % learning
      block.Dwork(2).Data = block.Dwork(2).Data + 1;
      i = block.Dwork(2).Data;

      if i == 99 % end of learning period
          block.Dwork(8).Data = block.Dwork(8).Data + 1; % setting flag for control
      end

      if i <= 2000
          idx = (i-1)*4 + 1;
          state(idx:(idx+3)) = x;
          block.Dwork(1).Data = state;
      else
          state(1:1997) = state(4:2000);
          state(1997:2000) = x;
          block.Dwork(1).Data = state;
      end
  end

  if (block.Dwork(8).Data == 1)
      block.Dwork(8).Data = block.Dwork(8).Data + 1;

      block.Dwork(2).Data = block.Dwork(2).Data + 1;
      i = block.Dwork(2).Data;
      idx = (i-1)*4 + 1;
      state(idx:(idx+3)) = x;
      block.Dwork(1).Data = state;    
      state_data = state(1:(idx + 3));
      mat = reshape(state_data,[4,i]);
      [Am,P,xkp1] = init_dmd(mat);

      K = dlqr(Am,disc_B,Q,R);
      u = -K*x;

      block.Dwork(3).Data = K;
      block.Dwork(4).data = u;
      block.Dwork(5).data = xkp1;
      block.Dwork(6).data = reshape(Am,[16,1]);
      block.Dwork(7).data = reshape(P,[16,1]);
      

  elseif (block.Dwork(8).Data > 1)
      xkp1 = block.Dwork(5).Data;
      xkp1 = reshape(xkp1,[4,1]);

      P = block.Dwork(7).Data;
      P = reshape(P,[4,4]);

      A = block.Dwork(6).Data;
      A = reshape(A,[4,4]);

      uk = block.Dwork(4).data;

      [At,Pt,xkp1t] = online_dmd_update(A,P,xkp1,x,uk);
 
      K = dlqr(At, disc_B,Q,R);
      u = -K*x;

      block.Dwork(3).Data = K;
      block.Dwork(4).data = u;
      block.Dwork(5).Data = xkp1t;
      block.Dwork(6).data = reshape(At,[16,1]);
      block.Dwork(7).data = reshape(Pt,[16,1]);
  end
  
%endfunction


function Outputs(block)
  u = block.Dwork(4).data;
  block.OutputPort(1).Data = u;

%endfunction

