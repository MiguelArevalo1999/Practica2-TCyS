function varargout = FFT(varargin)
% FFT MATLAB code for FFT.fig
%      FFT, by itself, creates a new FFT or raises the existing
%      singleton*.
%
%      H = FFT returns the handle to a new FFT or the handle to
%      the existing singleton*.
%
%      FFT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FFT.M with the given input arguments.
%
%      FFT('Property','Value',...) creates a new FFT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before FFT_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to FFT_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FFT

% Last Modified by GUIDE v2.5 25-Jan-2021 11:31:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FFT_OpeningFcn, ...
                   'gui_OutputFcn',  @FFT_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before FFT is made visible.
function FFT_OpeningFcn(hObject, eventdata, handles, varargin)
handles.output = hObject;
axes(handles.axes1)
imshow('logoipn.png')
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to FFT (see VARARGIN)

% Choose default command line output for FFT
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FFT wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FFT_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
close(FFT)
menudis
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function Y = myFFT(x)
    indices = [0 : length(x)-1];
    revIndices = bin2dec(fliplr(dec2bin(indices, 8))); % bit reversed indices
    revX = x(indices+1);
    temp = zeros(1, length(revX));
    N = 2;
    
    while(N <= length(revX))
        for n = (0:length(revX)/N-1)
            for k = (0:N -1)
                subOff = (N * n);
                even = (mod(k, N/2)) + subOff + 1;
                temp(k + subOff + 1) = revX(even) + (exp(-1i * k * 2 * pi / N) * revX(even + (N/2)));
            end
        end
        revX = temp;
        N = N * 2;
    end
    Y = revX;

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
x = str2num(get(handles.edit1,'string'));
N=length(x);
%Fs=512;
y = myFFT(x);                               % Compute DFT of x
m = abs(y);                               % Magnitude
y(m<1e-6) = 0;
%Ts=1/Fs;
p = unwrap(angle(y)); 
f = (0:length(y)-1)*100/length(y);        % Frequency vector
tiempo=linspace(0,6,N)
axes(handles.axes2)
stem(x,tiempo,'k');
xlim([-20 20]);
figure();
subplot(2,1,1)
plot(f,m,'r')
title('Magnitud')
ax = gca;
ax.XTick = [15 40 60 85];

subplot(2,1,2)
plot(f,p*180/pi,'g')
title('Fase')
ax = gca;
ax.XTick = [15 40 60 85];
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
