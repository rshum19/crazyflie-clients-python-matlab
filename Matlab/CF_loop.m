
% Optitrack Matlab / NatNet Sample
% 
%  Requirements:
%   - OptiTrack Motive 1.5 or later
%   - OptiTrack NatNet 2.5 or later
%   - Matlab R2013
%

function NatNetMatlabSample()

    display('NatNet Sample Begin')
    
    global frameRate;
    lastFrameTime = -1.0;
    lastFrameID = -1.0;
    useUI = false;
    
    % Define ZeroMQ binding port and address to connect to CrazyFlie client
    port = 1024 + 188;
    bind_addr = sprintf('tcp://127.0.0.1:%d',port);

    persistent arr;
    % Open figure
    if(useUI)
        hFigure = figure('Name','OptiTrack NatNet Matlab Sample','NumberTitle','off');
    end

    try
        % Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
        % Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
        % are both in the same folder and/or path if you move them.
        display('[NatNet] Creating Client.')
		curDir = pwd;
		mainDir = fileparts(fileparts(curDir));
		dllPath = fullfile(curDir,'NatNetSDK','lib','x64','NatNetML.dll');
        assemblyInfo = NET.addAssembly(dllPath);

        % Create an instance of a NatNet client
        theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
        version = theClient.NatNetVersion();
        fprintf( '[NatNet] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );

        % Connect to an OptiTrack server (e.g. Motive)
        display('[NatNet] Connecting to OptiTrack Server.')
        hst = java.net.InetAddress.getLocalHost;
        LocalIP = char(hst.getHostAddress);
        HostIP = char('192.168.1.205');
        flg = theClient.Initialize(LocalIP, HostIP); % Flg = returnCode: 0 = Success
        if (flg == 0)
            display('[NatNet] Initialization Succeeded')
        else
            display('[NatNet] Initialization Failed')
        end
        
        % print out a list of the active tracking Models in Motive
        GetDataDescriptions(theClient)
        
        % Test - send command/request to Motive
        [byteArray, retCode] = theClient.SendMessageAndWait('FrameRate');
        if(retCode ==0)
            byteArray = uint8(byteArray);
            frameRate = typecast(byteArray,'single');
        end
        
        % --------- ZeroMQ message protocol ------------
        % Add ZeroMQ - Matlab Binding library
         display('[ZeroMQ] Creating socket...')
        
        % Create and a ZeroMQ socket
%         context = zmq.core.ctx_new();
%         sender = zmq.core.socket(context, 'ZMQ_PUSH');
%         zmq.core.connect(sender, bind_addr);
%         display('[ZeroMQ] Socket initialized')
        
        % ---------- get the mocap data -------------------
        % approach 1 : get data by polling - just grab 5 secs worth of data in a tight loop
        for idx = 1 : 10000   
           % Note: sleep() accepts [mSecs] duration, but does not process any events.
           % pause() processes events, but resolution on windows can be at worst 15 msecs
           java.lang.Thread.sleep(5);  

            % Poll for latest frame instead of using event callback
            data = theClient.GetLastFrameOfData();
            frameTime = data.fLatency;
            frameID = data.iFrame;
            if(frameTime ~= lastFrameTime)
                fprintf('FrameTime: %0.3f\tFrameID: %5d\n',frameTime, frameID);
                lastFrameTime = frameTime;
                lastFrameID = frameID;

                % Parse mocap data
                 mocap_data  = parseMocapData( data );
                %%%%%%% Add Controller here %%%%%%%%%
                %[quad_cmd] = mocap2cmd( moca_data, frameTime);
                
                % Send command message
%                 cmdmsg = sprintf('%d %d %d %d', quad_cmd.phi, quad_cmd.theta, quad_cmd.psi, quad_cmd.thrust);
%                 fprintf('[ZeroMQ] Sending command: %d %d %d %d\n',  quad_cmd.phi, quad_cmd.theta, quad_cmd.psi, quad_cmd.thrust);
%                 zmq.core.send(sender,uint8(cmdmsg));
            
            else
                display('Duplicate frame');
            end
        end
                
    catch err
        display(err);
    end

    % cleanup
    
%     zmq.core.disconnect(sender, bind_addr);
%     zmq.core.close(sender);
% 
%     zmq.core.ctx_shutdown(context);
%     zmq.core.ctx_term(context);
    
    clear functions;

    display('Crazyflie Control Ended')
    
end

% Update a Matlab Plot with values from a single frame of mocap data
function UpdateUI( frameOfData )

    persistent lastFrameTime;
    persistent lastFrameID;
    persistent hX;
    persistent hY;
    persistent hZ;
    persistent arrayIndex;
    persistent frameVals;
    persistent xVals;
    persistent yVals;
    persistent zVals;
    persistent bufferModulo;

    global frameRate;
    
    % first time - generate an array and a plot
    if isempty(hX)
        % initialize statics
        bufferModulo = 256;
        frameVals = 1:255;
        xVals = zeros([1,255]);
        yVals = zeros([1,255]);
        zVals = zeros([1,255]);
        arrayIndex = 1;
        lastFrameTime = frameOfData.fLatency;
        lastFrameID = frameOfData.iFrame;
       
        % create plot
        hX = plot(frameVals, xVals, 'color', 'r');
        hold on;
        hY = plot(frameVals, yVals, 'color', 'g');
        hZ = plot(frameVals, zVals, 'color', 'b');
        title('Mocap Angle Plot');
        xlabel('Frame number');
        ylabel('Angle (degrees)');
        set(gca,'YLim',[-180 180]);    
        set(gca,'XGrid','on','YGrid','on');
    end

    % calculate the frame increment based on mocap frame's timestamp
    % in general this should be monotonically increasing according
    % To the mocap framerate, however frames are not guaranteed delivery
    % so to be accurate we test and report frame drop or duplication
    newFrame = true;
    droppedFrames = false;
    frameTime = frameOfData.fLatency;
    frameID = frameOfData.iFrame;
    calcFrameInc = round( (frameTime - lastFrameTime) * frameRate );
    % clamp it to a circular buffer of 255 frames
    arrayIndex = mod(arrayIndex + calcFrameInc, bufferModulo);
    if(arrayIndex==0)
        arrayIndex = 1;
    end
    if(calcFrameInc > 1)
        % debug
        % fprintf('\nDropped Frame(s) : %d\n\tLastTime : %.3f\n\tThisTime : %.3f\n', calcFrameInc-1, lastFrameTime, frameTime);
        droppedFrames = true;
    elseif(calcFrameInc == 0)
        % debug
        % display('Duplicate Frame')      
        newFrame = false;
    end
    
    % debug
    % fprintf('FrameTime: %0.3f\tFrameID: %d\n',frameTime, frameID);
    
    try
        if(newFrame)
            if(frameOfData.RigidBodies.Length() > 0)

                rigidBodyData = frameOfData.RigidBodies(1);

                % Test : Marker Y Position Data
                % angleY = data.LabeledMarkers(1).y;

                % Test : Rigid Body Y Position Data
                % angleY = rigidBodyData.y;

                % Test : Rigid Body 'Yaw'
                % Note : Motive display euler's is X (Pitch), Y (Yaw), Z (Roll), Right-Handed (RHS), Relative Axes
                % so we decode eulers heres to match that.
                q = quaternion( rigidBodyData.qx, rigidBodyData.qy, rigidBodyData.qz, rigidBodyData.qw );
                qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
                q = mtimes(q, qRot);
                angles = EulerAngles(q,'zyx');
                angleX = -angles(1) * 180.0 / pi;   % must invert due to 180 flip above
                angleY = angles(2) * 180.0 / pi;
                angleZ = -angles(3) * 180.0 / pi;   % must invert due to 180 flip above
                           
                if(droppedFrames)
                    for i = 1 : calcFrameInc
                        fillIndex = arrayIndex - i;
                        if(fillIndex < 1)
                            fillIndex = bufferModulo-(abs(fillIndex)+1);
                        end
                        xVals(fillIndex) = angleX;  
                        yVals(fillIndex) = angleY;  
                        zVals(fillIndex) = angleZ;  
                    end
                end

                % update the array/plot for this frame
                xVals(arrayIndex) = angleX;  
                yVals(arrayIndex) = angleY;  
                zVals(arrayIndex) = angleZ;  
                set(hX, 'YData', xVals);
                set(hY, 'YData', yVals);
                set(hZ, 'YData', zVals);

           end
        end
    catch err
        display(err);
    end
    
    lastFrameTime = frameTime;
    lastFrameID = frameID;

end

% Print out a description of actively tracked models from Motive
function GetDataDescriptions( theClient )

    dataDescriptions = theClient.GetDataDescriptions();
    
    % print out 
    fprintf('[NatNet] Tracking Models : %d\n\n', dataDescriptions.Count);
    for idx = 1 : dataDescriptions.Count
        descriptor = dataDescriptions.Item(idx-1);
        if(descriptor.type == 0)
            fprintf('\tMarkerSet \t: ');
        elseif(descriptor.type == 1)
            fprintf('\tRigid Body \t: ');                
        elseif(descriptor.type == 2)
            fprintf('\tSkeleton \t: ');               
        else
            fprintf('\tUnknown data type : ');               
        end
        fprintf('%s\n', char(descriptor.Name));
    end

    for idx = 1 : dataDescriptions.Count
        descriptor = dataDescriptions.Item(idx-1);
        if(descriptor.type == 0)
            fprintf('\n\tMarkerset : %s\t(%d markers)\n', char(descriptor.Name), descriptor.nMarkers);
            markerNames = descriptor.MarkerNames;
            for markerIndex = 1 : descriptor.nMarkers
                name = markerNames(markerIndex);
                fprintf('\t\tMarker : %-20s\t(ID=%d)\n', char(name), markerIndex);             
            end
        elseif(descriptor.type == 1)
            fprintf('\n\tRigid Body : %s\t\t(ID=%d, ParentID=%d)\n', char(descriptor.Name),descriptor.ID,descriptor.parentID);
        elseif(descriptor.type == 2)
            fprintf('\n\tSkeleton : %s\t(%d bones)\n', char(descriptor.Name), descriptor.nRigidBodies);
            %fprintf('\t\tID : %d\n', descriptor.ID);
            rigidBodies = descriptor.RigidBodies;
            for boneIndex = 1 : descriptor.nRigidBodies
                rigidBody = rigidBodies(boneIndex);
                fprintf('\t\tBone : %-20s\t(ID=%d, ParentID=%d)\n', char(rigidBody.Name), rigidBody.ID, rigidBody.parentID);
            end               
        end
    end

end



 