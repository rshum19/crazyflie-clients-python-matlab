function zmq_input( varargin )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

port = 1024 + 188;
%port = 5006;
if (nargin > 0)
    port =  varargin{1};
end

context = zmq.core.ctx_new();
sender = zmq.core.socket(context, 'ZMQ_PUSH');
bind_addr = sprintf('tcp://127.0.0.1:%d',port);
zmq.core.connect(sender, bind_addr);

fprintf('Starting to send control commands!\n')

roll = 0;
pitch = 0;
yaw = 0;
thrust = 100;
cmdmsg = sprintf('%d %d %d %d', roll, pitch, yaw, thrust);

% Unlock thrust protection
while(1)
    zmq.core.send(sender,uint8(cmdmsg));
end



end

