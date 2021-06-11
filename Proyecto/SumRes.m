function varargout = SumRes(varargin)
% SUMRES MATLAB code for SumRes.fig
%      SUMRES, by itself, creates a new SUMRES or raises the existing
%      singleton*.
%
%      H = SUMRES returns the handle to a new SUMRES or the handle to
%      the existing singleton*.
%
%      SUMRES('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SUMRES.M with the given input arguments.
%
%      SUMRES('Property','Value',...) creates a new SUMRES or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SumRes_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SumRes_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SumRes

% Last Modified by GUIDE v2.5 24-Jan-2021 01:16:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SumRes_OpeningFcn, ...
                   'gui_OutputFcn',  @SumRes_OutputFcn, ...
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


% --- Executes just before SumRes is made visible.
function SumRes_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SumRes (see VARARGIN)

% Choose default command line output for SumRes
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SumRes wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SumRes_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function xn_Callback(hObject, eventdata, handles)
% hObject    handle to xn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xn as text
%        str2double(get(hObject,'String')) returns contents of xn as a double


% --- Executes during object creation, after setting all properties.
function xn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function hn_Callback(hObject, eventdata, handles)
% hObject    handle to hn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hn as text
%        str2double(get(hObject,'String')) returns contents of hn as a double


% --- Executes during object creation, after setting all properties.
function hn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pause on
aux1=str2num(get(handles.xn,'string'));
aux2=str2num(get(handles.hn,'string'));
aux3=str2double(get(handles.x0,'string'));
aux4=str2double(get(handles.h0,'string'));
x=[aux1];
h=[aux2];
a=length(x);
b=length(h);
c=find(x==aux3);%busca elementos no nulos en el vector
d=find(h==aux4);%busca elementos no nulos en el otro vector
if(length(c)>1)
    c1=c(1,1); %condicion de no negatividad
else
    c1=c;
end
if(length(d)>1)%condicion de no negatividad
    d1=d(1,1);
else
    d1=d;
end
ts=a+b;
X1=[x,zeros(1,ts-a)];
H1=[h,zeros(1,ts-b)];
k=20;
X=[x,zeros(1,40-a)];
H=[h,zeros(1,40-b)];
if(c1<d1)
    s=d1;
    X1=circshift(X1,[0 (d1-c1)]);
    X=circshift(X,[0 (d1-c1)]);
elseif(c1>d1)
    s=c1;
    H1=circshift(H1,[0 (c1-d1)]);
    H=circshift(H,[0 (c1-d1)]);
elseif(c1==d1)
    s=c1;
end
rs=X1+H1;
X= X([ end-k+1:end 1:end-k ]);
H= H([ end-k+1:end 1:end-k ]);
Sum=X+H;
n=-20:20-1;
figure();
if(c1==d1)
    h1=subplot(3,1,1);
    stem(n-s+1,X,'MarkerFaceColor','magenta')
    title('\bfx[n]')
    h2=subplot(3,1,2);
    stem(n-s+1,H,'MarkerFaceColor','green')
    title('\bfh[n]')
    h3=subplot(3,1,3);
    stem(n-s+1,Sum,'MarkerFaceColor','red')
    title('\bfSuma')
else
    h1=subplot(3,1,1);
    stem(n-s+1,X,'MarkerFaceColor','magenta')
    title('\bfx[n]')
    h2=subplot(3,1,2);
    stem(n-s+1,H,'MarkerFaceColor','green')
    title('\bfh[n]')
    h3=subplot(3,1,3);
    stem(n-s+1,Sum,'MarkerFaceColor','red')
    title('\bfSuma')
end
rs = [regexprep(num2str(rs),' +',' ')];
set(handles.ResSuma,'string',rs);
pause off


function h0_Callback(hObject, eventdata, handles)
% hObject    handle to h0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of h0 as text
%        str2double(get(hObject,'String')) returns contents of h0 as a double


% --- Executes during object creation, after setting all properties.
function h0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to h0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x0_Callback(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x0 as text
%        str2double(get(hObject,'String')) returns contents of x0 as a double


% --- Executes during object creation, after setting all properties.
function x0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Atras.
function Atras_Callback(hObject, eventdata, handles)
close(SumRes)
menudis
% hObject    handle to Atras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
pause on
aux1=str2num(get(handles.xn,'string'));
aux2=str2num(get(handles.hn,'string'));
aux3=str2double(get(handles.x0,'string'));
aux4=str2double(get(handles.h0,'string'));
x=[aux1];
h=[aux2];
a=length(x);
b=length(h);
c=find(x==aux3);
d=find(h==aux4);
if(length(c)>1)
    c1=c(1,1);
else
    c1=c;
end
if(length(d)>1)
    d1=d(1,1);
else
    d1=d;
end
ts=a+b;
X1=[x,zeros(1,ts-a)];
H1=[h,zeros(1,ts-b)];
k=20;
X=[x,zeros(1,40-a)];
H=[h,zeros(1,40-b)];
if(c1<d1)
    s=d1;
    X1=circshift(X1,[0 (d1-c1)]);
    X=circshift(X,[0 (d1-c1)]);
elseif(c1>d1)
    s=c1;
    H1=circshift(H1,[0 (c1-d1)]);
    H=circshift(H,[0 (c1-d1)]);
elseif(c1==d1)
    s=c1;
end
rr=X1-H1;
X= X([ end-k+1:end 1:end-k ]);
H= H([ end-k+1:end 1:end-k ]);
Res=X-H;
n=-20:20-1;
figure();
if(c1==d1)
    h1=subplot(3,1,1);
    stem(n-s+1,X,'MarkerFaceColor','magenta')
    title('\bfx[n]')
    h2=subplot(3,1,2);
    stem(n-s+1,H,'MarkerFaceColor','green')
    title('\bfh[n]')
    h3=subplot(3,1,3);
    stem(n-s+1,Res,'MarkerFaceColor','red')
    title('\bfSuma')
else
    h1=subplot(3,1,1);
    stem(n-s+1,X,'MarkerFaceColor','magenta')
    title('\bfx[n]')
    h2=subplot(3,1,2);
    stem(n-s+1,H,'MarkerFaceColor','green')
    title('\bfh[n]')
    h3=subplot(3,1,3);
    stem(n-s+1,Res,'MarkerFaceColor','red')
    title('\bfResta')
end
rr = [regexprep(num2str(rr),' +',' ')];
set(handles.ResResta,'string',rr);
pause off