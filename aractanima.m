function varargout = aractanima(varargin)
% ARACTANIMA MATLAB code for aractanima.fig
%      ARACTANIMA, by itself, creates a new ARACTANIMA or raises the existing
%      singleton*.
%
%      H = ARACTANIMA returns the handle to a new ARACTANIMA or the handle to
%      the existing singleton*.
%
%      ARACTANIMA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARACTANIMA.M with the given input arguments.
%
%      ARACTANIMA('Property','Value',...) creates a new ARACTANIMA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before aractanima_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to aractanima_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help aractanima

% Last Modified by GUIDE v2.5 27-May-2020 22:28:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @aractanima_OpeningFcn, ...
                   'gui_OutputFcn',  @aractanima_OutputFcn, ...
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


% --- Executes just before aractanima is made visible.
function aractanima_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to aractanima (see VARARGIN)

% Choose default command line output for aractanima
handles.output = hObject;
% create an axes that spans the whole gui
ah = axes('unit', 'normalized', 'position', [0 0 1 1]); 
% import the background image and show it on the axes
bg = imread('bg.png'); imagesc(bg);
% prevent plotting over the background and turn the axis off
set(ah,'handlevisibility','off','visible','off')
% making sure the background is behind all the other uicontrols
uistack(ah, 'bottom');
%set(gcf, 'units','normalized','outerposition',[0 0 1 1]);
% Update handles structure
guidata(hObject, handles);


% --- Outputs from this function are returned to the command line.
function varargout = aractanima_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in exitButton.
function exitButton_Callback(hObject, eventdata, handles)
% hObject    handle to exitButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 msgbox('Cikisi yapiliyor...','Bilgilendirme Mesajý');

clear;
clc;
close all;



% --- Executes on button press in resetButton.
function resetButton_Callback(hObject, eventdata, handles)
% hObject    handle to resetButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.axes2);
cla(handles.axes3);

        

% --- Executes on button press in browseButton.
function browseButton_Callback(hObject, eventdata, handles)
% hObject    handle to browseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% 
global im;
global img;
[filename, pathname] = uigetfile({'*.jpg;*.jpeg;*.png'},'Resim Seç');
img = [pathname, filename];
assignin('base','testimage',img);
im = imread(img);
guidata(hObject,handles);
axes(handles.axes2);
imshow(im);

% kategoriAdi= char(label); % Show the label

%set(handles.edit2,'String',kategori);




% --- Executes on button press in recbutton.
function recbutton_Callback(hObject, eventdata, handles)
% hObject    handle to recbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global im  ;
global mynet ;

%mynet= load('mynet.mat'); ikinci egitim cnn
mynet= load('cnn.mat');


im1 = im ;
%im1= imresize(im1,[227 227]);

[w, h , d] = size(im1);
H_= zeros(w,h) ;
counter = 1;

position = [] ; say = 0;
windowHeight = 40;
windowWidth = 40;
c1 = 1 ; r1= 1 ; sayac= 0 ;
framegray = rgb2gray(im1);
framegray = imadjust(framegray);

 Message= [] ; 
 Message{1} = sprintf('Bolgeler taraniyor.. ') ;


 set(handles.edit2,'string',Message) ;
for y = 1 : windowWidth/2 : w - windowHeight
   
    for x = 1 : windowHeight/2 : h - windowWidth
   
       p1 = [x,y];
      p2 = [x + (windowWidth - 1), y + (windowHeight - 1)];
      po = [p1; p2] ;

      % resim kirpma ve tarama islemi
      crop_px = [po(1,1) po(2,1)];
      crop_py  = [po(1,2) po(2,2)];
      topLeftRow = ceil(min(crop_px));
      topLeftCol = ceil(min(crop_py)); 
      say = say +1 ;
      bottomRightRow = ceil(max(crop_px));
      bottomRightCol = ceil(max(crop_py));

      cropedImage = framegray(topLeftCol:bottomRightCol,topLeftRow:bottomRightRow,:);
      axes(handles.axes3);
  %% grayscaleden tekrar rgb donusumu
     RGB = cat(3, cropedImage, cropedImage, cropedImage);
  
     [kategori, skor]= classify(mynet.net,RGB);
     imshow(RGB)

     disp(kategori,skor)
     %% cnnde kategorisi arac ve skoru 0.8 dan buyuk alanlari bulma
    if(kategori== 'vehicle') && (skor(2)>0.8)
        disp('arac bulundu') ; 
      sayac = sayac +1 ; % bulunan arac bolgesi sayisi
        H_(crop_px(1),crop_py(1)) = skor(2) ; %% koordinatlar
       end
      counter = counter + 1;
      x = x + 1;
    end 
end
 
[rw,cl] = find(H_>0.8); %% 0.8 'dan buyuk skorlu alanlar
 
 axes(handles.axes3);
 imshow(im1)

  

 Message{1} = sprintf('Arac bulunan muhtemel bolgeler.. ') ;


 set(handles.edit2,'string',Message) ; 
 
 for d = 1:length(rw)
 pause(1)
 imrect(gca,[cl(d),rw(d),80,80]);
 end


guidata(hObject,handles);







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


% --- Executes on button press in siniflandirgoruntuisleme.
function siniflandirgoruntuisleme_Callback(hObject, eventdata, handles)
% hObject    handle to siniflandirgoruntuisleme (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% goruntu isleme ile arac tespiti
%% resmin grayscale donusturulmesi
global im ;
f = rgb2gray(im);

figure;
subplot(3,3,1);
imshow(f), title('Gri olcekli resim');

%% on islem
% goruntu ayd?nlatmak icin gamma düzeltimi
f = imadjust(f,[],[],0.5);

subplot(3,3,2);
imshow(f), title('Gamma sonras resim ');

% gurultu silerek resim yumusatma islemi
mask = ones(5) / (25);
f_smooth = imfilter(f,mask);

subplot(3,3,3);
imshow(f_smooth), title('Yumusatilmis resim');
%% kenar(edge) bulma
% Sobel kullanarak kenar belirleme

f_edge = edge(f_smooth, 'Sobel');

subplot(3,3,4);
imshow(f_edge), title('Sobel sonrasi kenarlar');
%% esik deger 
f_edge = im2uint8(f_edge);
level = graythresh(f_edge);
f_th  = imbinarize(f_edge, level);

 subplot(3,3,5);
 imshow(f_th), title('Goruntu Esigi');
%% morfolojik islem

se1 = strel('rectangle', [15 40]);
se2 = strel('rectangle', [15 10]);

%Kenarlarin iliskili kisimlari arasindaki bosluklara kapama islemi 
f_close = imclose(f_th,se1);

subplot(3,3,6);
imshow(f_close), title('Kapama islemi sonrasi');

% istenmeyen objeler icin acma islemi
f_open = imopen(f_close,se2);

subplot(3,3,7);
imshow(f_open), title('Acma islemi sonrasi');
%% baglantili bilesenler
cc1 = bwconncomp(f_open);
% regionpropsla ozellik cikarimi
stats1 = regionprops(cc1, {'Area','Eccentricity'});
area = [stats1.Area];
sumArea = 0;


for i = 1 : cc1.NumObjects
    sumArea = sumArea + area(i);
 
end

avArea= sumArea / cc1.NumObjects;


%  ortalamanin 1/4 'unden buyuk alanlari bulma
idx = find([stats1.Area]>(avArea * 1 / 4)); 
BW = ismember(labelmatrix(cc1), idx);
subplot(3,3,8);

imshow(BW), title('Bagli bilesenlerin binary hali');


cc2 = bwconncomp(BW);
stats2 = regionprops(cc2, 'BoundingBox');

%% araclari alg?lama
subplot(3,3,9);
imshow(f), title('Bulunan Araclar');

% bulunan araclari bounding box ile dikdortgen icinde gosterme islemi 
for i = 1 : cc2.NumObjects
rectangle('Position', stats2(i).BoundingBox, ...
    'EdgeColor','r','LineWidth',2);
end
axes(handles.axes3)
imshow(f);
% her bulunan arac icin uygulama
for i = 1 : cc2.NumObjects
rectangle('Position', stats2(i).BoundingBox, ...
    'EdgeColor','r','LineWidth',2);
end

guidata(handles,varargin);


% --- Executes on button press in playaudio.
function playaudio_Callback(hObject, eventdata, handles)
% hObject    handle to playaudio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% 

   [y, Fs] = audioread('music.mp3');
   PO=audioplayer(y,Fs);
   % Play audio
   playblocking(PO)

% [handles.y, handles.Fs] = audioread('intheyear2525.mp3');
%  handles.player = audioplayer(handles.y, handles.Fs);
% play(handles.player);


guidata(handles,varargin);
