function varargout = menudis(varargin)
% MENUDIS MATLAB code for menudis.fig
%      MENUDIS, by itself, creates a new MENUDIS or raises the existing
%      singleton*.
%
%      H = MENUDIS returns the handle to a new MENUDIS or the handle to
%      the existing singleton*.
%
%      MENUDIS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MENUDIS.M with the given input arguments.
%
%      MENUDIS('Property','Value',...) creates a new MENUDIS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before menudis_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to menudis_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help menudis

% Last Modified by GUIDE v2.5 09-Jun-2021 19:24:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @menudis_OpeningFcn, ...
                   'gui_OutputFcn',  @menudis_OutputFcn, ...
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


% --- Executes just before menudis is made visible.
function menudis_OpeningFcn(hObject, eventdata, handles, varargin)
axes(handles.menudis);
[x,map]=imread('fourier.jpg');
image(x)
colormap(map);
axis off
hold on
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to menudis (see VARARGIN)

% Choose default command line output for menudis
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes menudis wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = menudis_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in sum.
function sum_Callback(hObject, eventdata, handles)
close(menudis)
SumRes
% hObject    handle to sum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Amp.
function Amp_Callback(hObject, eventdata, handles)
close(menudis)
Amplificacion
% hObject    handle to Amp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in reflejo.
function reflejo_Callback(hObject, eventdata, handles)
close(menudis)
Reflejo
% hObject    handle to reflejo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in desp.
function desp_Callback(hObject, eventdata, handles)
close(menudis)
Desplazamiento
% hObject    handle to desp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in diez.
function diez_Callback(hObject, eventdata, handles)
close(menudis)
Diezmacion
% hObject    handle to diez (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in conv.
function conv_Callback(hObject, eventdata, handles)
close(menudis)
Convolucion
% hObject    handle to conv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in FFT.
function FFT_Callback(hObject, eventdata, handles)
close(menudis)
FFT
% hObject    handle to FFT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Atras.
function Atras_Callback(hObject, eventdata, handles)
close(menudis)
Menu
% hObject    handle to Atras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbuttonInterpolacion.
function pushbuttonInterpolacion_Callback(hObject, eventdata, handles)
close(menudis)
Interpolacion
% hObject    handle to pushbuttonInterpolacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbuttonAtenuacion.
function pushbuttonAtenuacion_Callback(hObject, eventdata, handles)
close(menudis)
Atenuacion
% hObject    handle to pushbuttonAtenuacion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over diez.
function diez_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to diez (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
