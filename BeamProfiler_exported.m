classdef BeamProfiler_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        BeamprofilerUIFigure       matlab.ui.Figure
        CameraPositionLabel        matlab.ui.control.Label
        ProfileView                matlab.ui.container.Panel
        PerformGaussianFitButton   matlab.ui.control.Button
        showfitCheckBox            matlab.ui.control.CheckBox
        w2EditField                matlab.ui.control.NumericEditField
        w0_yLabel                  matlab.ui.control.Label
        w1EditField                matlab.ui.control.NumericEditField
        w0_xLabel                  matlab.ui.control.Label
        Profile2                   matlab.ui.control.UIAxes
        Profile1                   matlab.ui.control.UIAxes
        CameraControlPanel         matlab.ui.container.Panel
        gainSlider                 matlab.ui.control.Slider
        gainSliderLabel            matlab.ui.control.Label
        framerateSlider            matlab.ui.control.Slider
        framerateSliderLabel       matlab.ui.control.Label
        exposuretimeSlider         matlab.ui.control.Slider
        exposuretimeSliderLabel    matlab.ui.control.Label
        TakeImageButton            matlab.ui.control.Button
        CloseCameraButton          matlab.ui.control.Button
        LiveViewButton             matlab.ui.control.StateButton
        PosReadout                 matlab.ui.control.NumericEditField
        PosGauge                   matlab.ui.control.LinearGauge
        MovementPanel              matlab.ui.container.Panel
        referencedLamp             matlab.ui.control.Lamp
        referencedLampLabel        matlab.ui.control.Label
        ReferenceRunButton         matlab.ui.control.Button
        MOVEButton                 matlab.ui.control.Button
        TypeofMovementButtonGroup  matlab.ui.container.ButtonGroup
        relativeButton             matlab.ui.control.RadioButton
        absoluteButton             matlab.ui.control.RadioButton
        PositionmmEditField        matlab.ui.control.NumericEditField
        PositionmmEditFieldLabel   matlab.ui.control.Label
        ImageViewPanel             matlab.ui.container.Panel
        FalseColorImage            matlab.ui.control.UIAxes
        GreyscaleImage             matlab.ui.control.UIAxes
        SerialConnectionPanel      matlab.ui.container.Panel
        PortlistRefreshButton      matlab.ui.control.Button
        PortDropDown               matlab.ui.control.DropDown
        PortDropDownLabel          matlab.ui.control.Label
        CloseConnectionButton      matlab.ui.control.Button
        ConnectButton              matlab.ui.control.Button
    end

    
    properties (Access = private)
        serPort % serial port for communication with arduino
        serialInputString % serial input buffer for strings
        serialInputInt % serial input buffer for position integers
        errorHandle % handle for error dialog windows
        
        axisReferenced % flag if axis is referenced
        axisPosition % position of camera in mm
        axisDirection % current direction of axis movement
        
        himg % matlab figure handle of camera image
        img % image data handle
        cam % camera handle
        pixelSize % pixel size of camera
        
        estimatedParameters
        estGauss
    end
    
    methods (Access = private)
        
        function success = establishSerialConnection(app)
            port = app.PortDropDown.Value;
            try
                app.serPort = serialport(port, 9600);
                configureCallback(app.serPort, "terminator", @app.serialCallbackFunction);
                success = true;
            catch ME
                app.errorHandle = errordlg("Establishing the serial connection to the" + ...
                    " arduino failed." + newline +"Try to pull the " +...
                    "usb cable and plug it back in." + newline + ...
                    newline + ME.message);
                success = false;
            end
        end
        
        function serialCallbackFunction(app, src, ~)
            data = readline(src);
            % app.SerialMonitor.Value = app.SerialMonitor.Value + data + newline;
            pos = str2double(data);
            if isnan(pos)
                switch data.strip()
                    case "s"
                        app.animateGauge();
                    case "Referenced"
                        app.referencedLamp.Enable = 1;
                        app.referencedLamp.Color = [0 1 0];
                        app.axisReferenced = 1;
                        app.relativeButton.Enable = 1;
                    case "LowEndSwitch"
                    case "highEnd"
                    case "NotYetRef"
                    case "BadCmd"
                end
            else
                app.axisPosition = pos;
                app.PosReadout.Value = pos/100; 
                app.PosGauge.Value = pos/100;
                app.MOVEButton.Enable = 1;
                app.ReferenceRunButton.Enable = 1;
            end
        end
        
        function animateGauge(app)
                app.PosGauge.Value = app.PosGauge.Value + app.axisDirection*0.1;
                app.PosReadout.Value = app.PosReadout.Value + app.axisDirection*0.1;
        end
        
        function initCamera(app)
            asm = System.AppDomain.CurrentDomain.GetAssemblies;
            if ~any(arrayfun(@(n) strncmpi(char(asm.Get(n-1).FullName), ...
                    'uEyeDotNet', length('uEyeDotNet')), 1:asm.Length))
                NET.addAssembly(...
                    'C:\Program Files\IDS\uEye\Develop\DotNet\signed\uEyeDotNet.dll');
            end
            %   Create camera object handle
            app.cam = uEye.Camera;
            %   Open 1st available camera
            %   Returns if unSuccessful
            if ~strcmp(char(app.cam.Init), 'Success')
                error('Could not initialize camera');
            end
            %   Set display mode to bitmap (DiB)
            if ~strcmp(char(app.cam.Display.Mode.Set(uEye.Defines.DisplayMode.DiB)), ...
                    'Success')
                error('Could not set display mode');
            end
            %   Set colormode to 8-bit RAW
            if ~strcmp(char(app.cam.PixelFormat.Set(uEye.Defines.ColorMode.SensorRaw8)), ...
                    'Success')
                error('Could not set pixel format');
            end
            %   Set trigger mode to software (single image acquisition)
%             if ~strcmp(char(app.cam.Trigger.Set(uEye.Defines.TriggerMode.Software)), 'Success')
%                 error('Could not set trigger format');
%             end
             %   Allocate image memory
            [ErrChk, app.img.ID] = app.cam.Memory.Allocate(true);
            if ~strcmp(char(ErrChk), 'Success')
                error('Could not allocate memory');
            end
            %   Obtain image information
            % [ErrChk, app.img.Width, app.img.Height] = app.cam.Memory.GetSize(app.img.ID);
            [ErrChk, app.img.Width, app.img.Height, app.img.Bits, app.img.Pitch] ...
                = app.cam.Memory.Inquire(app.img.ID);
            if ~strcmp(char(ErrChk), 'Success')
                error('Could not get image information');
            end
        end
        
        function image = acquireImage(app)
        %   Acquire image
            if ~strcmp(char(app.cam.Acquisition.Freeze(true)), 'Success')
                error('Could not acquire image');
            end
            
            %   Extract image
            [ErrChk, image] = app.cam.Memory.CopyToArray(app.img.ID);
            if ~strcmp(char(ErrChk), 'Success')
                error('Could not obtain image data');
            end
        end
        
        function res = gauss2DcostFcn(~, params, xx, yy, zz)
            A = params(1);
            x0 = params(2);
            y0 = params(3);
            w1 = params(4);
            w2 = params(5);
            phi = params(6);
            A0 = params(7);

            exp1 = ((xx-x0)*cos(phi) + (yy-y0)*sin(phi)).^2;
            exp2 = ((yy-y0)*cos(phi) - (xx-x0)*sin(phi)).^2;

            gauss2D = A * exp(-2 * exp1 / w1^2) .* exp(-2 * exp2 / w2^2) + A0;
            res = zz - gauss2D;
            
        end
        
        function gauss2D = gauss2D(~, params, xx, yy)
            A = params(1);
            x0 = params(2);
            y0 = params(3);
            w1 = params(4);
            w2 = params(5);
            phi = params(6);
            A0 = params(7);

            exp1 = ((xx-x0)*cos(phi) + (yy-y0)*sin(phi)).^2;
            exp2 = ((yy-y0)*cos(phi) - (xx-x0)*sin(phi)).^2;

            gauss2D = A * exp(-2 * exp1 / w1^2) .* exp(-2 * exp2 / w2^2) + A0;
        end
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            app.PortDropDown.Items = serialportlist;
            app.axisPosition = 0.;
            app.axisReferenced = 0;
            app.PosGauge.MajorTicks = 0:10:300;
            app.PosGauge.MajorTickLabels = string(app.PosGauge.MajorTicks);
                        
            app.pixelSize = 5.2;
            drawnow();
        end

        % Button pushed function: ConnectButton
        function ConnectButtonPushed(app, event)
            app.ConnectButton.Enable = 0;
            app.ConnectButton.Text = "Connecting...";
            app.ConnectButton.BackgroundColor = [1.00 1.00 0.07];
            if ~app.establishSerialConnection
                app.ConnectButton.Text = "Error";
                app.ConnectButton.BackgroundColor = [1.0 0 0];
                uiwait(app.errorHandle);
                app.ConnectButton.Text = "Connect";
                app.ConnectButton.BackgroundColor = [0.96 0.96 0.96];
                app.ConnectButton.Enable = 1;
            else
                app.ConnectButton.Text = "Connected";
                app.ConnectButton.BackgroundColor = [0 1.00 0];
                app.CloseConnectionButton.Enable = 1;
                app.ReferenceRunButton.Enable = 1;
            end
        end

        % Button pushed function: CloseConnectionButton
        function CloseConnectionButtonPushed(app, event)
            app.serPort = [];
            app.CloseConnectionButton.Enable = 0;
            app.ConnectButton.Enable = 1;
            app.ConnectButton.Text = "Connect";
            app.ConnectButton.BackgroundColor = [0.96 0.96 0.96];
            app.PortDropDown.Items = serialportlist;
            app.ReferenceRunButton.Enable = 0;
            app.referencedLamp.Color = [0.94,0.94,0.94];
            app.MOVEButton.Enable = 0;
        end

        % Button pushed function: PortlistRefreshButton
        function PortlistRefreshButtonPushed(app, event)
            app.PortDropDown.Items = serialportlist;
        end

        % Button pushed function: ReferenceRunButton
        function ReferenceRunButtonPushed(app, event)
            app.PosGauge.Value = 0;
            app.PosReadout.Value = 0;
            desiredPosition = round(app.PositionmmEditField.Value, 2);
            steps = desiredPosition * 100;
            app.axisDirection = 1;
            app.MOVEButton.Enable = 0;
            app.ReferenceRunButton.Enable = 0;
            stringToSend = sprintf("R;M;%i;1;", steps);
            app.serPort.write(stringToSend, 'char');
        end

        % Button pushed function: MOVEButton
        function MOVEButtonPushed(app, event)
            if app.axisReferenced==0
                return
            end
            desiredPos = round(app.PositionmmEditField.Value, 2);
            if app.absoluteButton.Value==1
                currentPos = app.axisPosition;
            elseif app.relativeButton.Value==1
                currentPos = 0;
            end
            deltaSteps = desiredPos*100 - currentPos;
            steps = abs(deltaSteps);
            app.axisDirection = sign(deltaSteps);
            
            app.ReferenceRunButton.Enable = 0;
            app.MOVEButton.Enable = 0;
            
            stringToSend = sprintf("M;%i;%i;", steps, app.axisDirection);
            app.serPort.write(stringToSend, 'char');
        end

        % Selection changed function: TypeofMovementButtonGroup
        function TypeofMovementButtonGroupSelectionChanged(app, event)
            selectedButton = app.TypeofMovementButtonGroup.SelectedObject;
            if selectedButton.Text=="relative"
                app.PositionmmEditField.Limits = [-290 290];
            elseif selectedButton.Text=="absolute"
                app.PositionmmEditField.Limits = [0 290];
            end
        end

        % Value changed function: LiveViewButton
        function LiveViewButtonValueChanged(app, event)
            value = app.LiveViewButton.Value;
            
            if value==1
                app.initCamera;
                cmap = jet(256);
                cmap(251:end, :) = zeros(6, 3);
                app.himg = imagesc;
                himgAxes = app.himg.Parent;
                himgFig = himgAxes.Parent;
                himgFig.NumberTitle = 'off';
                himgFig.Name = "Live Camera View";
                himgFig.Position = [1053 472 547 344];
                axis(himgAxes, 'image');
                axis(himgAxes, 'tight');
                axis(himgAxes, [-inf inf -inf inf -inf inf 0 255]);
                colormap(himgAxes, cmap);
                colorbar(himgAxes);
                % hx = line(0, 0, 'Color', 'r', 'LineWidth', 2);
                % hy = line(0, 0, 'Color', 'r', 'LineWidth', 2);

                app.cam.Acquisition.Capture;
                app.exposuretimeSlider.Enable = 1;
                app.framerateSlider.Enable = 1;
                app.gainSlider.Enable = 1;
                
%                 [~, val] = app.cam.Timing.Framerate.GetFrameRateRange;
%                 app.framerateSlider.Limits = [val.Minimum val.Maximum];
                app.cam.Timing.Framerate.Set(app.framerateSlider.Value);
                
                [~, val] = app.cam.Timing.Exposure.GetRange;
                app.exposuretimeSlider.Limits = [val.Minimum val.Maximum];
                app.cam.Timing.Exposure.Set(app.exposuretimeSlider.Value);
                
                app.cam.Gain.Hardware.Factor.SetMaster(app.gainSlider.Value*100);

                app.CloseCameraButton.Enable = 1;
                app.TakeImageButton.Enable = 1;
                
                T = zeros(10, 1);
                tic
                while app.LiveViewButton.Value==1 && ishghandle(app.himg)
                    % Copy image from graphics card to RAM (wait for completion)
                    app.cam.DirectRenderer.StealNextFrame(uEye.Defines.DeviceParameter.Wait);
                    
                    % Copy image from RAM to Matlab array
                    [~, I] = app.cam.Memory.CopyToArray(app.img.ID);
                    app.img.Data = reshape(uint8(I), app.img.Width, app.img.Height).';
                    Imax = max(app.img.Data, [], 'all');
                    
                    % Calculate marginals
                    % Ix = sum(uint64(app.img.Data));
                    % Iy = sum(uint64(app.img.Data), 2);
                    
                    % Plot data
                    app.himg.CData = app.img.Data;
                    % hx.XData = 1:length(Ix);
                    % hx.YData = Ix*.2*diff(app.himg.YData)/max(Ix);
                    % hy.YData = 1:length(Iy);
                    % hy.XData = Iy*.2*diff(app.himg.XData)/max(Iy);
                    
                    T = [T(2:end); toc];
                    title(sprintf('FPS: %.1f ;  I_{max} = %i', 10/diff(T([1 end])), ...
                        Imax));
                    drawnow;
                end
                app.CloseCameraButton.Enable = 0;
                app.LiveViewButton.Value = 0;
                % Stop capture
                app.cam.Acquisition.Stop;

                % Free image memory
                app.cam.Memory.Free(app.img.ID);

                if ~strcmp(char(app.cam.Exit), 'Success')   % close camera properly
                        error('Could not close camera');
                end
            end
        end

        % Button pushed function: CloseCameraButton
        function CloseCameraButtonPushed(app, event)
            if ~strcmp(char(app.cam.Exit), 'Success')
                    error('Could not close camera');
            end
        end

        % Button pushed function: TakeImageButton
        function TakeImageButtonPushed(app, event)
            app.LiveViewButton.Value = 0;
            
            ixy = app.img.Data;
            tmpImg = imshow(ixy, 'Parent', app.GreyscaleImage);
            set(tmpImg.Parent, 'YDir', 'normal');
            axis(tmpImg.Parent, 'image');
            axis(tmpImg.Parent, 'tight');
            
            imgDims=size(ixy);
            xPxls=1:imgDims(2);
            yPxls=1:imgDims(1);
            [app.img.X,app.img.Y] = meshgrid(xPxls*app.pixelSize,yPxls*app.pixelSize);
            
            tmpImg = imagesc(app.FalseColorImage, app.img.X(1,:), app.img.Y(:,1), ixy);
            set(tmpImg.Parent, 'YDir', 'normal');
            cmap = jet(256);
            cmap(251:end, :) = zeros(6, 3);
            axis(tmpImg.Parent, 'image');
            axis(tmpImg.Parent, 'tight');
            axis(tmpImg.Parent, [-inf inf -inf inf -inf inf 0 255]);
            colormap(tmpImg.Parent, cmap);
            colorbar(tmpImg.Parent);
            
            app.PerformGaussianFitButton.Enable = 1;
        end

        % Button pushed function: PerformGaussianFitButton
        function PerformGaussianFitButtonPushed(app, event)
            app.LiveViewButton.Value = 0;
            
            cla(app.Profile1);
            cla(app.Profile2);

            fig = uifigure;
            d = uiprogressdlg(fig, 'Title', 'Please wait',...
                'Message', 'performing gaussian fit');
            drawnow;
            % Defining X, Y and Z arrays
            X = app.img.X;
            Y = app.img.Y;
            Z = double(app.img.Data);
            d.Value = 0.1;
            d.Message = 'loading data';
            %% Initial guesses for fit parameters
            xProfile = mean(Z, 1);
            yProfile = mean(Z, 2);
            [~, x0_ini] = max(xProfile);
            [~, y0_ini] = max(yProfile);
            A0_ini = max(Z, [], 'all')*0.8;
            d.Value = 0.3;
            d.Message = 'defining initial fitting parameters and parameter bounds';
            initialGuess = [A0_ini, x0_ini*app.pixelSize, y0_ini*app.pixelSize, 500, 500, 0, 0];

            %% Define lower and upper bounds ANGLE IS FIXED TO ZERO FOR NOW !!!
            lb = [1, 1, 1, 1, 1, 0, 0];
            ub = [256, 1280*app.pixelSize, 1024*app.pixelSize, 3000, 3000, 0, 50];
            
            %% Fitting
            d.Indeterminate = 1;
            d.Message = 'performing gaussian fit ...';
            app.estimatedParameters = lsqnonlin(@(params)app.gauss2DcostFcn(params,X,Y,Z), initialGuess, lb, ub);
            
            % estParsInt = uint16(estimatedParameters);
            app.estGauss = app.gauss2D(app.estimatedParameters, X, Y);
            d.Indeterminate = 0;
            d.Value = 0.8;
            d.Message = 'calculating estimated 2D gauss';
            %% Calculate profiles along main axes
            x0 = app.estimatedParameters(2);
            x0pxls = x0/app.pixelSize;
            y0 = app.estimatedParameters(3);
            y0pxls = y0/app.pixelSize;
            px = improfile(Z, [1 1280], [y0pxls y0pxls]);
            py = improfile(Z, [x0pxls x0pxls], [1 1024]);
            
            d.Value = 0.9;
            d.Message = 'plotting of profiles';
            %% Show profile lines in false color image
            line(app.FalseColorImage, [1 1280]*app.pixelSize, [y0 y0], ...
                'Color', [1 0 0 0.8], 'LineWidth', 1);
            line(app.FalseColorImage, [x0 x0], [1 1024]*app.pixelSize, ...
                'Color', [1 0 0 0.8], 'LineWidth', 1);
            
            %% Plot profiles with fits and display corresponding widths
            line(app.Profile1, app.img.X(1,:), px, 'DisplayName', 'camera data', ...
                 'LineWidth', 2);
            line(app.Profile1, app.img.X(1,:), app.estGauss(uint16(y0pxls), :), ...
                'Color', 'r', 'DisplayName', 'gaussian fit', 'LineWidth', 2);
            legend(app.Profile1);
            
            line(app.Profile2, app.img.Y(:,1), py, 'DisplayName', 'camera data', ...
                 'LineWidth', 2);
            line(app.Profile2, app.img.Y(:,1), app.estGauss(:, uint16(x0pxls)), ...
                'Color', 'r', 'DisplayName', 'gaussian fit', 'LineWidth', 2);
            legend(app.Profile2);
            d.Value = 1;
            d.Message = 'finished';
            app.w1EditField.Value = app.estimatedParameters(4);
            app.w2EditField.Value = app.estimatedParameters(5);
            
            close(d);
            close(fig);
        end

        % Value changed function: exposuretimeSlider
        function exposuretimeSliderValueChanged(app, event)
            value = app.exposuretimeSlider.Value;
            app.cam.Timing.Exposure.Set(value);
        end

        % Value changed function: framerateSlider
        function framerateSliderValueChanged(app, event)
            value = app.framerateSlider.Value;
            app.cam.Timing.Framerate.Set(value);
            
            [~, val] = app.cam.Timing.Exposure.GetRange;
            app.exposuretimeSlider.Limits = [val.Minimum val.Maximum];
%             [~, val] = app.cam.Timing.Exposure.Get;
%             app.exposuretimeSlider.Value = val;
        end

        % Value changed function: gainSlider
        function gainSliderValueChanged(app, event)
            value = app.gainSlider.Value;
            app.cam.Gain.Hardware.Factor.SetMaster(value*100);
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create BeamprofilerUIFigure and hide until all components are created
            app.BeamprofilerUIFigure = uifigure('Visible', 'off');
            app.BeamprofilerUIFigure.Position = [2 42 1050 828];
            app.BeamprofilerUIFigure.Name = 'Beamprofiler';

            % Create SerialConnectionPanel
            app.SerialConnectionPanel = uipanel(app.BeamprofilerUIFigure);
            app.SerialConnectionPanel.Title = 'Serial Connection';
            app.SerialConnectionPanel.BackgroundColor = [0.902 0.902 0.902];
            app.SerialConnectionPanel.FontWeight = 'bold';
            app.SerialConnectionPanel.Position = [7 709 140 115];

            % Create ConnectButton
            app.ConnectButton = uibutton(app.SerialConnectionPanel, 'push');
            app.ConnectButton.ButtonPushedFcn = createCallbackFcn(app, @ConnectButtonPushed, true);
            app.ConnectButton.Position = [20 38 100 22];
            app.ConnectButton.Text = 'Connect';

            % Create CloseConnectionButton
            app.CloseConnectionButton = uibutton(app.SerialConnectionPanel, 'push');
            app.CloseConnectionButton.ButtonPushedFcn = createCallbackFcn(app, @CloseConnectionButtonPushed, true);
            app.CloseConnectionButton.Enable = 'off';
            app.CloseConnectionButton.Position = [15 8 110 22];
            app.CloseConnectionButton.Text = 'Close Connection';

            % Create PortDropDownLabel
            app.PortDropDownLabel = uilabel(app.SerialConnectionPanel);
            app.PortDropDownLabel.HorizontalAlignment = 'center';
            app.PortDropDownLabel.Position = [6 67 25 22];
            app.PortDropDownLabel.Text = 'Port';

            % Create PortDropDown
            app.PortDropDown = uidropdown(app.SerialConnectionPanel);
            app.PortDropDown.Items = {};
            app.PortDropDown.Position = [37 67 70 22];
            app.PortDropDown.Value = {};

            % Create PortlistRefreshButton
            app.PortlistRefreshButton = uibutton(app.SerialConnectionPanel, 'push');
            app.PortlistRefreshButton.ButtonPushedFcn = createCallbackFcn(app, @PortlistRefreshButtonPushed, true);
            app.PortlistRefreshButton.Icon = 'refresh.png';
            app.PortlistRefreshButton.IconAlignment = 'center';
            app.PortlistRefreshButton.Position = [110 67 25 22];
            app.PortlistRefreshButton.Text = '';

            % Create ImageViewPanel
            app.ImageViewPanel = uipanel(app.BeamprofilerUIFigure);
            app.ImageViewPanel.Title = 'Image View';
            app.ImageViewPanel.BackgroundColor = [0.902 0.902 0.902];
            app.ImageViewPanel.FontWeight = 'bold';
            app.ImageViewPanel.Position = [156 409 881 344];

            % Create GreyscaleImage
            app.GreyscaleImage = uiaxes(app.ImageViewPanel);
            title(app.GreyscaleImage, 'Greyscale Image')
            app.GreyscaleImage.DataAspectRatio = [1 1 1];
            app.GreyscaleImage.PlotBoxAspectRatio = [5.01960784313725 4.0156862745098 1];
            app.GreyscaleImage.XLim = [0 1280];
            app.GreyscaleImage.YLim = [0 1024];
            app.GreyscaleImage.ZLim = [0 255];
            app.GreyscaleImage.Box = 'on';
            app.GreyscaleImage.Position = [8 13 422 303];

            % Create FalseColorImage
            app.FalseColorImage = uiaxes(app.ImageViewPanel);
            title(app.FalseColorImage, 'False Color Image')
            xlabel(app.FalseColorImage, 'x / \mum')
            ylabel(app.FalseColorImage, 'y / \mum')
            app.FalseColorImage.DataAspectRatio = [1 1 1];
            app.FalseColorImage.PlotBoxAspectRatio = [26.1019607843137 20.8823529411765 1];
            app.FalseColorImage.XLim = [0 6656];
            app.FalseColorImage.YLim = [0 5325];
            app.FalseColorImage.ZLim = [0 255];
            app.FalseColorImage.Box = 'on';
            app.FalseColorImage.Position = [441 13 429 303];

            % Create MovementPanel
            app.MovementPanel = uipanel(app.BeamprofilerUIFigure);
            app.MovementPanel.Title = 'Movement';
            app.MovementPanel.BackgroundColor = [0.902 0.902 0.902];
            app.MovementPanel.FontWeight = 'bold';
            app.MovementPanel.Position = [7 422 140 277];

            % Create PositionmmEditFieldLabel
            app.PositionmmEditFieldLabel = uilabel(app.MovementPanel);
            app.PositionmmEditFieldLabel.HorizontalAlignment = 'right';
            app.PositionmmEditFieldLabel.Position = [33 226 78 22];
            app.PositionmmEditFieldLabel.Text = 'Position / mm';

            % Create PositionmmEditField
            app.PositionmmEditField = uieditfield(app.MovementPanel, 'numeric');
            app.PositionmmEditField.Limits = [0 290];
            app.PositionmmEditField.ValueDisplayFormat = '%11.2f';
            app.PositionmmEditField.Position = [29 205 86 22];
            app.PositionmmEditField.Value = 150;

            % Create TypeofMovementButtonGroup
            app.TypeofMovementButtonGroup = uibuttongroup(app.MovementPanel);
            app.TypeofMovementButtonGroup.SelectionChangedFcn = createCallbackFcn(app, @TypeofMovementButtonGroupSelectionChanged, true);
            app.TypeofMovementButtonGroup.Title = 'Type of Movement';
            app.TypeofMovementButtonGroup.Position = [8 119 123 75];

            % Create absoluteButton
            app.absoluteButton = uiradiobutton(app.TypeofMovementButtonGroup);
            app.absoluteButton.Text = 'absolute';
            app.absoluteButton.Position = [11 29 67 22];
            app.absoluteButton.Value = true;

            % Create relativeButton
            app.relativeButton = uiradiobutton(app.TypeofMovementButtonGroup);
            app.relativeButton.Enable = 'off';
            app.relativeButton.Text = 'relative';
            app.relativeButton.Position = [11 7 65 22];

            % Create MOVEButton
            app.MOVEButton = uibutton(app.MovementPanel, 'push');
            app.MOVEButton.ButtonPushedFcn = createCallbackFcn(app, @MOVEButtonPushed, true);
            app.MOVEButton.Enable = 'off';
            app.MOVEButton.Position = [19 86 100 22];
            app.MOVEButton.Text = 'MOVE';

            % Create ReferenceRunButton
            app.ReferenceRunButton = uibutton(app.MovementPanel, 'push');
            app.ReferenceRunButton.ButtonPushedFcn = createCallbackFcn(app, @ReferenceRunButtonPushed, true);
            app.ReferenceRunButton.Enable = 'off';
            app.ReferenceRunButton.Position = [20 50 100 22];
            app.ReferenceRunButton.Text = 'Reference Run';

            % Create referencedLampLabel
            app.referencedLampLabel = uilabel(app.MovementPanel);
            app.referencedLampLabel.HorizontalAlignment = 'right';
            app.referencedLampLabel.FontColor = [0.502 0.502 0.502];
            app.referencedLampLabel.Position = [28 18 63 22];
            app.referencedLampLabel.Text = 'referenced';

            % Create referencedLamp
            app.referencedLamp = uilamp(app.MovementPanel);
            app.referencedLamp.Enable = 'off';
            app.referencedLamp.Position = [98 22 14 14];
            app.referencedLamp.Color = [0.9412 0.9412 0.9412];

            % Create PosGauge
            app.PosGauge = uigauge(app.BeamprofilerUIFigure, 'linear');
            app.PosGauge.Limits = [0 300];
            app.PosGauge.Position = [156 764 881 60];

            % Create PosReadout
            app.PosReadout = uieditfield(app.BeamprofilerUIFigure, 'numeric');
            app.PosReadout.Limits = [0 290];
            app.PosReadout.ValueDisplayFormat = '%11.2f mm';
            app.PosReadout.Editable = 'off';
            app.PosReadout.HorizontalAlignment = 'center';
            app.PosReadout.Position = [597 768 66 22];

            % Create CameraControlPanel
            app.CameraControlPanel = uipanel(app.BeamprofilerUIFigure);
            app.CameraControlPanel.Title = 'Camera Control';
            app.CameraControlPanel.BackgroundColor = [0.902 0.902 0.902];
            app.CameraControlPanel.Position = [7 16 140 394];

            % Create LiveViewButton
            app.LiveViewButton = uibutton(app.CameraControlPanel, 'state');
            app.LiveViewButton.ValueChangedFcn = createCallbackFcn(app, @LiveViewButtonValueChanged, true);
            app.LiveViewButton.Text = 'Live View';
            app.LiveViewButton.Position = [17 344 106 22];

            % Create CloseCameraButton
            app.CloseCameraButton = uibutton(app.CameraControlPanel, 'push');
            app.CloseCameraButton.ButtonPushedFcn = createCallbackFcn(app, @CloseCameraButtonPushed, true);
            app.CloseCameraButton.Position = [19 17 100 22];
            app.CloseCameraButton.Text = 'Close Camera';

            % Create TakeImageButton
            app.TakeImageButton = uibutton(app.CameraControlPanel, 'push');
            app.TakeImageButton.ButtonPushedFcn = createCallbackFcn(app, @TakeImageButtonPushed, true);
            app.TakeImageButton.Enable = 'off';
            app.TakeImageButton.Position = [20 313 100 22];
            app.TakeImageButton.Text = 'Take Image';

            % Create exposuretimeSliderLabel
            app.exposuretimeSliderLabel = uilabel(app.CameraControlPanel);
            app.exposuretimeSliderLabel.HorizontalAlignment = 'right';
            app.exposuretimeSliderLabel.Enable = 'off';
            app.exposuretimeSliderLabel.Position = [28 270 81 22];
            app.exposuretimeSliderLabel.Text = 'exposure time';

            % Create exposuretimeSlider
            app.exposuretimeSlider = uislider(app.CameraControlPanel);
            app.exposuretimeSlider.Limits = [0.6675 205];
            app.exposuretimeSlider.MajorTicks = [0.6675 50 100 150 200];
            app.exposuretimeSlider.MajorTickLabels = {'min', '50', '100', '150', '200'};
            app.exposuretimeSlider.ValueChangedFcn = createCallbackFcn(app, @exposuretimeSliderValueChanged, true);
            app.exposuretimeSlider.MinorTicks = [25 75 125 175];
            app.exposuretimeSlider.Enable = 'off';
            app.exposuretimeSlider.Position = [14 257 110 3];
            app.exposuretimeSlider.Value = 205;

            % Create framerateSliderLabel
            app.framerateSliderLabel = uilabel(app.CameraControlPanel);
            app.framerateSliderLabel.HorizontalAlignment = 'right';
            app.framerateSliderLabel.Enable = 'off';
            app.framerateSliderLabel.Position = [39 190 57 22];
            app.framerateSliderLabel.Text = 'framerate';

            % Create framerateSlider
            app.framerateSlider = uislider(app.CameraControlPanel);
            app.framerateSlider.Limits = [4.87 15];
            app.framerateSlider.MajorTicks = [5 10 15];
            app.framerateSlider.ValueChangedFcn = createCallbackFcn(app, @framerateSliderValueChanged, true);
            app.framerateSlider.MinorTicks = [6 7 8 9 11 12 13 14];
            app.framerateSlider.Enable = 'off';
            app.framerateSlider.Position = [18 177 101 3];
            app.framerateSlider.Value = 4.87;

            % Create gainSliderLabel
            app.gainSliderLabel = uilabel(app.CameraControlPanel);
            app.gainSliderLabel.HorizontalAlignment = 'right';
            app.gainSliderLabel.Enable = 'off';
            app.gainSliderLabel.Position = [55 118 28 22];
            app.gainSliderLabel.Text = 'gain';

            % Create gainSlider
            app.gainSlider = uislider(app.CameraControlPanel);
            app.gainSlider.Limits = [1 10];
            app.gainSlider.MajorTicks = [1 5 10];
            app.gainSlider.ValueChangedFcn = createCallbackFcn(app, @gainSliderValueChanged, true);
            app.gainSlider.MinorTicks = [2 3 4 6 7 8 9];
            app.gainSlider.Enable = 'off';
            app.gainSlider.Position = [11 105 110 3];
            app.gainSlider.Value = 1;

            % Create ProfileView
            app.ProfileView = uipanel(app.BeamprofilerUIFigure);
            app.ProfileView.Title = 'Profile View';
            app.ProfileView.BackgroundColor = [0.902 0.902 0.902];
            app.ProfileView.FontWeight = 'bold';
            app.ProfileView.Position = [156 16 881 381];

            % Create Profile1
            app.Profile1 = uiaxes(app.ProfileView);
            title(app.Profile1, 'Profile in x direction')
            xlabel(app.Profile1, 'X / \mum')
            ylabel(app.Profile1, 'I / a.u.')
            app.Profile1.PlotBoxAspectRatio = [1.48290598290598 1 1];
            app.Profile1.XLim = [0 6656];
            app.Profile1.YLim = [0 255];
            app.Profile1.ZLim = [0 255];
            app.Profile1.NextPlot = 'replaceall';
            app.Profile1.Box = 'on';
            app.Profile1.Position = [21 62 395 290];

            % Create Profile2
            app.Profile2 = uiaxes(app.ProfileView);
            title(app.Profile2, 'Profile in y direction')
            xlabel(app.Profile2, 'Y / \mum')
            ylabel(app.Profile2, 'I / a.u.')
            app.Profile2.PlotBoxAspectRatio = [1.45762711864407 1 1];
            app.Profile2.XLim = [0 5325];
            app.Profile2.YLim = [0 255];
            app.Profile2.ZLim = [0 255];
            app.Profile2.NextPlot = 'replaceall';
            app.Profile2.Box = 'on';
            app.Profile2.Position = [460 62 392 292];

            % Create w0_xLabel
            app.w0_xLabel = uilabel(app.ProfileView);
            app.w0_xLabel.HorizontalAlignment = 'right';
            app.w0_xLabel.Position = [156 17 34 22];
            app.w0_xLabel.Text = 'w0_x';

            % Create w1EditField
            app.w1EditField = uieditfield(app.ProfileView, 'numeric');
            app.w1EditField.ValueDisplayFormat = '%11.1f µm';
            app.w1EditField.Position = [205 17 100 22];

            % Create w0_yLabel
            app.w0_yLabel = uilabel(app.ProfileView);
            app.w0_yLabel.HorizontalAlignment = 'right';
            app.w0_yLabel.Position = [597 17 34 22];
            app.w0_yLabel.Text = 'w0_y';

            % Create w2EditField
            app.w2EditField = uieditfield(app.ProfileView, 'numeric');
            app.w2EditField.ValueDisplayFormat = '%11.1f µm';
            app.w2EditField.Position = [646 17 100 22];

            % Create showfitCheckBox
            app.showfitCheckBox = uicheckbox(app.ProfileView);
            app.showfitCheckBox.Enable = 'off';
            app.showfitCheckBox.Visible = 'off';
            app.showfitCheckBox.Text = 'show fit';
            app.showfitCheckBox.Position = [409 17 64 22];

            % Create PerformGaussianFitButton
            app.PerformGaussianFitButton = uibutton(app.ProfileView, 'push');
            app.PerformGaussianFitButton.ButtonPushedFcn = createCallbackFcn(app, @PerformGaussianFitButtonPushed, true);
            app.PerformGaussianFitButton.Enable = 'off';
            app.PerformGaussianFitButton.Position = [8 17 129 22];
            app.PerformGaussianFitButton.Text = 'Perform Gaussian Fit';

            % Create CameraPositionLabel
            app.CameraPositionLabel = uilabel(app.BeamprofilerUIFigure);
            app.CameraPositionLabel.FontWeight = 'bold';
            app.CameraPositionLabel.Position = [498 768 100 22];
            app.CameraPositionLabel.Text = 'Camera Position';

            % Show the figure after all components are created
            app.BeamprofilerUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = BeamProfiler_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.BeamprofilerUIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.BeamprofilerUIFigure)
        end
    end
end