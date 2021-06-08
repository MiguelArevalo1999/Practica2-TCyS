function varargout = Interfaz(varargin)
% INTERFAZ MATLAB code for Interfaz.fig
%      INTERFAZ, by itself, creates a new INTERFAZ or raises the existing
%      singleton*.
%
%      H = INTERFAZ returns the handle to a new INTERFAZ or the handle to
%      the existing singleton*.
%
%      INTERFAZ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFAZ.M with the given input arguments.
%
%      INTERFAZ('Property','Value',...) creates a new INTERFAZ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Interfaz_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Interfaz_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Interfaz

% Last Modified by GUIDE v2.5 24-Jan-2021 16:52:32

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Interfaz_OpeningFcn, ...
                   'gui_OutputFcn',  @Interfaz_OutputFcn, ...
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
function r = getGlobalx
global x
r = x;

function setGlobalx(val)
global x
x = val;

function r = getGlobaln
global x
r = x;

function setGlobaln(val)
global x
x = val;

function r = getGlobaldir
global dir
r = dir;

function setGlobaldir(val)
global dir
dir = val;

% --- Executes just before Interfaz is made visible.
function Interfaz_OpeningFcn(hObject, eventdata, handles, varargin)
axes(handles.background);
[x,map]=imread('fondo.jpg');
image(x)
colormap(map);
axis off
hold on
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Interfaz (see VARARGIN)

% Choose default command line output for Interfaz
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Interfaz wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Interfaz_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Grabar.
function Grabar_Callback(hObject, eventdata, handles)
Fs=44100;
voz=audiorecorder(Fs,16,1);
recordblocking(voz,3);
datos=getaudiodata(voz);
audiowrite('voz.wav',datos,Fs);
n=numel(datos);
setGlobalx(datos)
tiempo=linspace(0,3,n)
axes(handles.Voz)
plot(tiempo,datos);
set(gca,'ycolor','w') 
xlabel Tiempo
set(gca,'xcolor','w') 
ylabel Datos




% hObject    handle to Grabar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% --- Executes on button press in Reproducir.
function Reproducir_Callback(hObject, eventdata, handles)
Fs=44100;
[y,Fs]=audioread('voz.wav');
sound(y,Fs);
pause(2);
% hObject    handle to Reproducir (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in sum.
function sum_Callback(hObject, eventdata, handles)
Fs=44100;
tem=getGlobalx
tem1=tem+tem;
tiempo=linspace(0,3,132300)
axes(handles.Resultado)
plot(tiempo,tem1);
set(gca,'ycolor','w')
xlabel Tiempo
set(gca,'xcolor','w')
ylabel Datos
sound(tem1, Fs);
% hObject    handle to sum (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function senialReflejada = reflejo(senialOriginal, Fs)
%Esta función realiza el reflejo de una señal ingresada por el usuario
%Se hace uso de la función de Matlab 'flipud', esta función lo que hace es
%voltear el array de arriba hacia abajo, o sea, con respecto al eje
%horizontal. Esto es así debido a que los datos muestreados de la señal de
%audio están colocados en 2 columnas; una columna para el audio del lado
%izquierdo y la otra columna para el audio de la derecha si la tipo de
%grabación fue estereo. Por lo tanto si queremos reflejar la señal, se
%tiene que invertir la última fila de la columna 1 con la primera fila de
%esta misma columna.
senialReflejada = flipud(senialOriginal);
%Se crea un vector x para que al graficar la señal de audio, el eje x se
%pueda escalar a los segundos que duró la grabación. En este caso, 3
%segundos. Y entre esos 3 segundos, se va a graficar cada punto muestreado.
%Como se grabó con una frecuencia de 44100 muestras por segundo. 
%Entre el segundo 0 y 3 habrá 132300. Eso es lo que se pretende con x
x = 0:(1/Fs):(3-(1/Fs));
axes(handles.Resultado)
plot(x',senialReflejada);
set(gca,'ycolor','w') 
xlabel Tiempo
set(gca,'xcolor','w') 
ylabel Datos
title('Señal Reflejada');
sound(senialReflejada, Fs);


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
senialOriginal=getGlobalx;
Fs=44100;
senialreleja=reflejo(senialOriginal, Fs)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function senialDesplazada = desplazamiento(senialOriginal, N, direccion, Fs, canales)
%Esta función desplaza la señal original N muestras a la direccion que
%especifica el usuario. 
%Para especificar la dirección se toma el número '1' como derecha y '0'
%como izquierda.
%Se crea un vector con el mismo número de índices que la senial original,
%si la señal original fue muestreada con una frecuencia de 44100 muestras
%por segundo y si se grabó por 3 segundos, la variable senialOriginal será
%una matriz de 132300 renglones y 2 columnas. Por lo tanto, la
%senialDesplazada será un vector con el mismo número de renglones que la
%senialOriginal
senialDesplazada = zeros(Fs*3, canales);
if (direccion == 1)
    senialDesplazada(N+1:end, :) = senialOriginal(1:(end-N), :);
else
    senialDesplazada(1:(end-N), :) = senialOriginal(N+1:end, :);
end
figure();
%Para graficar 2 graficas en una misma ventana
subplot(2,1,1);
%Se crea un vector x para que al graficar la señal de audio, el eje x se
%pueda escalar a los segundos que duró la grabación. En este caso, 3
%segundos. Y entre esos 3 segundos, se va a graficar cada punto muestreado.
%Como se grabó con una frecuencia de 44100 muestras por segundo. 
%Entre el segundo 0 y 3 habrá 132300. Eso es lo que se pretende con x
x = 0:(1/Fs):(3-(1/Fs));
plot(x',senialOriginal);
title('Señal Original');
subplot(2,1,2);
plot(x,senialDesplazada);
title('Señal Desplazada');
sound(senialDesplazada, Fs);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
senialOriginal=getGlobalx;
N=str2num(get(handles.edit1,'string'));
direccion=getGlobaldir;
Fs=44100;
canales=1;
senialDes = desplazamiento(senialOriginal, N, direccion, Fs, canales)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
Fs=44100;
senialOriginal=getGlobalx;
senial=fft(senialOriginal);
t=linspace(0,5,length(senialOriginal));
f=linspace(0,Fs,length(senial));
fase=unwrap(angle(senial));
magn=abs(senial);
figure();
subplot(411);plot(t,senialOriginal);
title('Señal Original');
ylabel('x');
xlabel('tiempo[s]');
subplot(412);plot(f,senial);
ylabel('Tansformada');
xlabel('frecuencia[Hz]');
subplot(413);plot(f,fase);
ylabel('Angulo');
xlabel('frecuencia[Hz]');
subplot(414);plot(f,magn);
ylabel('Magnitud');
xlabel('frecuencia[Hz]');

% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in Atras.
function Atras_Callback(hObject, eventdata, handles)
close(Interfaz)
Menu
% hObject    handle to Atras (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
%radiobutton2.handles
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1


% --- Executes on button press in checkbox2.
function checkbox2_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox2


% --- Executes on button press in izqui.
function izqui_Callback(hObject, eventdata, handles)
% hObject    handle to izqui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of izqui


% --- Executes during object creation, after setting all properties.
function izqui_CreateFcn(hObject, eventdata, handles)
set(hObject,'Value',0);
% hObject    handle to izqui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when selected object is changed in Direccion.
function Direccion_SelectionChangedFcn(hObject, eventdata, handles)
A=get(hObject,'String');
% if hObject == handles.izqui
%     dir=0;
%     setGlobaldir(dir);
% else
%     dir=1;
%     setGlobaldir(dir);
% end
switch A
    case 'Izquierda'
        dir=0;
    case 'Derecha'
        dir=1;
end
setGlobaldir(dir);
% hObject    handle to the selected object in Direccion 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function dS = diezmacion(dataSignal, N, Fs)
%Esta función realiza la diezmación de una señal mustreada
%El usurario ingresa el factor N por el cual se quiere hacer la
%diezmacion
%Fs es el periodo de muestro con el cual se muestreo la señal original
%Usa la funcion 'downsample' de Matlab perteneciente al paquete Signal
%Processing Toolbox. Esta función lo que hace es decrementar el periodo de
%muestreo de la señal original llamada 'dataSignal' manteniendo la primera
%muestra y después cada N-muestra después de la primera.
dS = downsample(dataSignal,N);
figure();
%Para graficar 2 graficas en una misma ventana
subplot(2,1,1);
%Crea un vector x con valores linealmente separados entre 0 y casi 3. Casi
%3 porque si se llegara hasta el número 3, se tendría una muestra de mas
%que las muestradas de la señal original. Por este motivo, se le resta
%un número a x con la instrucción '(3 - (1/Fs))'. Así, 'dataSignal' 
%y x tienen la misma cantidad de valores y así ya se pueden graficar.
%Se usa un incremento de (1/Fs) entre valores 0 y (3 - 1/Fs)  
x = 0:(1/Fs):(3-(1/Fs));
y = dataSignal;
plot(x',y);
title('Señal Original Muestreada');
subplot(2,1,2);
%Crea un vector x con valores linealmente separados entre 0 y casi 3. Casi
%3 porque si se llegara hasta el número 3, se tendría una muestra de mas
%que las retornadas por la funcion 'downsample'. Por este motivo, se le resta
%un número a x con la instrucción '(3 - 1/(Fs/N))'. Así, uS y x tienen la misma
%cantidad de valores y así ya se pueden graficar.
%Se usa un incremento de (1/(Fs/N)) entre valores 0 y (3 - 1/(Fs*N)) 
x = 0:(1/(Fs/N)):3-(1/(Fs/N));
y = dS;
plot(x',y);
title('Señal Diezmada');
sound(dS, Fs/N);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
Fs=44100;
N=str2num(get(handles.edit2,'string'));
dataSignal=getGlobalx;
senialdiez=diezmacion(dataSignal, N, Fs);
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function vectorInterpolado = interpolacion(dataSignal, N, Fs, value)
%Esta funcion hace la interpolacion 
%El usurario ingresa el factor N por el cual se quiere hacer la
%interpolacion
%Fs es el periodo de muestro con el cual se muestreo la señal original
%Usa la funcion 'upsample' de Matlab perteneciente al paquete Signal
%Processing Toolbox. Esta función lo que hace es incrementar el periodo de
%muestreo de la señal original llamada 'dataSignal' insertando n-1 ceros
%'0' entre muestras
if(value)
    vectorInterpolado = upsample(dataSignal,N);
else
    vectorInterpolado = zeros(1, length(dataSignal) * N);
    vectorInterpolado(1:N:end) = dataSignal(1:end);
    for i = 1:length(dataSignal)
        if i == length(dataSignal)
            w = (0 - dataSignal(i))/N;
        else
            w = (dataSignal(i+1) - dataSignal(i))/N;
        end
        aux = ((i-1)*N)+1;
        for j = 1:(N - 1)
            vectorInterpolado(aux+j) = vectorInterpolado(aux+j-1) + w;
        end
    end
end


figure();
%Para graficar 2 graficas en una misma ventana
subplot(2,1,1);
%Crea un vector x con valores linealmente separados entre 0 y casi 3. Casi
%3 porque si se llegara hasta el número 3, se tendría una muestra de mas
%que las muestradas de la señal original. Por este motivo, se le resta
%un número a x con la instrucción '(3 - (1/Fs))'. Así, 'dataSignal' 
%y x tienen la misma cantidad de valores y así ya se pueden graficar.
%Se usa un incremento de (1/Fs) entre valores 0 y (3 - 1/Fs) 
x = 0:(1/Fs):(3-(1/Fs));
y = dataSignal;
plot(x',y);
title('Señal Original');
subplot(2,1,2);
%Crea un vector x con valores linealmente separados entre 0 y casi 3. Casi
%3 porque si se llegara hasta el número 3, se tendría una muestra de mas
%que las retornadas por la funcion 'upsample'. Por este motivo, se le resta
%un número a x con la instrucción '(3 - 1/(Fs*N))'. Así, uS y x tienen la misma
%cantidad de valores y así ya se pueden graficar.
%Se usa un incremento de (1/Fs*N) entre valores 0 y (3 - 1/(Fs*N)) 
x = 0:(1/(Fs*N)):(3-(1/(Fs*N)));
y = vectorInterpolado;
plot(x',y);
title('Señal Interpolada');
sound(vectorInterpolado, Fs*N);

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
Fs=44100;
N=str2num(get(handles.edit2,'string'));
dataSignal=getGlobalx;
value = get(handles.radiobuttonCero, "value");
senialinter=interpolacion(dataSignal, N, Fs, value)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function senialAtenuada = atenuacion(senialOriginal, N, Fs)
%Esta función atenua la señal original por un factor de N ingresado por el
%usuario
senialAtenuada = senialOriginal / N;
figure();
%Para graficar 2 graficas en una misma ventana
subplot(2,1,1);
%Se crea un vector x para que al graficar la señal de audio, el eje x se
%pueda escalar a los segundos que duró la grabación. En este caso, 3
%segundos. Y entre esos 3 segundos, se va a graficar cada punto muestreado.
%Como se grabó con una frecuencia de 44100 muestras por segundo. 
%Entre el segundo 0 y 3 habrá 132300. Eso es lo que se pretende con x
x = 0:(1/Fs):(3-(1/Fs));
plot(x',senialOriginal);
title('Señal Original');
subplot(2,1,2);
plot(x',senialAtenuada);
title('Señal Atenuada');
sound(senialAtenuada, Fs);

% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
senialOriginal=getGlobalx;
Fs=44100;
N=str2num(get(handles.edit3,'string'));
senialate=atenuacion(senialOriginal, N, Fs)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function senialAmplificada = amplificacion(senialOriginal, N, Fs)
%Esta funcion amplifica la senial original por un factor N ingresado por el
%usuario
senialAmplificada = senialOriginal * N;
figure();
%Para graficar 2 graficas en una misma ventana
subplot(2,1,1);
%Se crea un vector x para que al graficar la señal de audio, el eje x se
%pueda escalar a los segundos que duró la grabación. En este caso, 3
%segundos. Y entre esos 3 segundos, se va a graficar cada punto muestreado.
%Como se grabó con una frecuencia de 44100 muestras por segundo. 
%Entre el segundo 0 y 3 habrá 132300. Eso es lo que se pretende con x
x = 0:(1/Fs):(3-(1/Fs));
plot(x',senialOriginal);
title('Señal Original');
subplot(2,1,2);
plot(x',senialAmplificada);
title('Señal Amplificada');
sound(senialAmplificada, Fs);

% --- Executes on button press in pushbutton13.
function pushbutton13_Callback(hObject, eventdata, handles)
senialOriginal=getGlobalx;
Fs=44100;
N=str2num(get(handles.edit3,'string'));
senialampli=amplificacion(senialOriginal, N, Fs)
% hObject    handle to pushbutton13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
