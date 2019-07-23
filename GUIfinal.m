function varargout = GUIfinal(varargin)
% GUIFINAL MATLAB code for GUIfinal.fig
%      GUIFINAL, by itself, creates a new GUIFINAL or raises the existing
%      singleton*.
%
%      H = GUIFINAL returns the handle to a new GUIFINAL or the handle to
%      the existing singleton*.
%
%      GUIFINAL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUIFINAL.M with the given input arguments.
%
%      GUIFINAL('Property','Value',...) creates a new GUIFINAL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUIfinal_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUIfinal_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUIfinal

% Last Modified by GUIDE v2.5 30-Jun-2019 12:34:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUIfinal_OpeningFcn, ...
                   'gui_OutputFcn',  @GUIfinal_OutputFcn, ...
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


% --- Executes just before GUIfinal is made visible.
function [hObject,handles]= GUIfinal_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
% varargin   command line arguments to GUIfinal (see VARARGIN)

% Choose default command line output for GUIfinal
% Update handles structure
guidata(hObject, handles);
set(handles.x_axis_limits,'visible','off');
set(handles.y_axis_limits,'visible','off');
set(handles.z_axis_limits,'visible','off');
set(handles.Marker_number,'string','');
set(handles.marker_number_copy,'string','');
set(handles.Value_FLE,'string','0.35');
handles.max_markers = 10;
handles.max_targets = 30;
set(handles.x_min_text,'string','-150');
set(handles.x_max_text,'string','150');
set(handles.y_min_text,'string','-120');
set(handles.y_max_text,'string','120');
set(handles.z_min_text,'string','-60');
set(handles.z_max_text,'string','60');
handles.sliderLimits_marker = [];
for i = 1:handles.max_markers
handles.x_min_marker(i) = str2double(get(handles.x_min_text,'string'));
handles.x_min_prev_marker(i) = handles.x_min_marker(i);
handles.x_max_marker(i) = str2double(get(handles.x_max_text,'string'));
handles.x_max_prev_marker(i) = handles.x_max_marker(i);
handles.y_min_marker(i) = str2double(get(handles.y_min_text,'string'));
handles.y_min_prev_marker(i) = handles.y_min_marker(i);
handles.y_max_marker(i) = str2double(get(handles.y_max_text,'string'));
handles.y_max_prev_marker(i) = handles.y_max_marker(i);
handles.z_min_marker(i) = str2double(get(handles.z_min_text,'string'));
handles.z_min_prev_marker(i) = handles.z_min_marker(i);
handles.z_max_marker(i) = str2double(get(handles.z_max_text,'string'));
handles.z_max_prev_marker(i) = handles.z_max_marker(i);
handles.sliderLimits_marker = [handles.sliderLimits_marker
    handles.x_min_marker(i) handles.x_max_marker(i) handles.y_min_marker(i) handles.y_max_marker(i) handles.z_min_marker(i) handles.z_max_marker(i)];
end
handles.sliderLimits_target = [];
for i = 1:handles.max_targets
handles.x_min_target(i) = str2double(get(handles.x_min_text,'string'));
handles.x_min_prev_target(i) = handles.x_min_target(i);
handles.x_max_target(i) = str2double(get(handles.x_max_text,'string'));
handles.x_max_prev_target(i) = handles.x_max_target(i);
handles.y_min_target(i) = str2double(get(handles.y_min_text,'string'));
handles.y_min_prev_target(i) = handles.y_min_target(i);
handles.y_max_target(i) = str2double(get(handles.y_max_text,'string'));
handles.y_max_prev_target(i) = handles.y_max_target(i);
handles.z_min_target(i) = str2double(get(handles.z_min_text,'string'));
handles.z_min_prev_target(i) = handles.z_min_target(i);
handles.z_max_target(i) = str2double(get(handles.z_max_text,'string'));
handles.z_max_prev_target(i) = handles.z_max_target(i);
handles.sliderLimits_target = [handles.sliderLimits_target
    handles.x_min_target(i) handles.x_max_target(i) handles.y_min_target(i) handles.y_max_target(i) handles.z_min_target(i) handles.z_max_target(i)];
end
handles.selectedRow_marker =0;
handles.selectedRow_target =0;
handles.output = hObject;
set(handles.hint_text,'string','Use ADD MARKER to start');
handles.MarkerNames = cell(handles.max_markers,1);
handles.TargetNames = cell(handles.max_targets,1);
handles.TargetNames{1} ='Target 1';
set(handles.DataTarget,'RowName',handles.TargetNames);
handles.n_markers = 0;
handles.n_targets = 1;
handles.marker_pts = [];
handles.target_pts = [0 0 0 nan];
set(handles.DataMarker,'ColumnName',{'X','Y','Z'});
set(handles.DataTarget,'ColumnName',{'X','Y','Z','TRE Value'});
set(handles.DataTarget,'data',handles.target_pts);
set(handles.DataMarker,'data',handles.marker_pts);
handles = update_row_names(handles);
set(handles.limits_changer,'Visible','Off');
set(handles.localisation_tab,'Enable','Off');
set(handles.limits_tab,'Enable','On');
set(handles.axes,'View',[-37.5 30]);
handles.names=dir('C:\Users\Rashmi\Desktop\CT Data\*');
index=size(handles.names,1)-2;
for i=1:index
    x=handles.names(i+2).folder;
    y='\';
    z=handles.names(i+2).name;
    u=strcat(x,y,z);
    handles.P(:,:,i)=dicomread(u);
end
handles.intensity_value = 3000;
set(handles.intensity_slider, 'Min',2400);
set(handles.intensity_slider, 'Max', 3400);
ratio = 1/double(handles.intensity_slider.Max - handles.x_axis_slider.Min);
set(handles.intensity_slider, 'SliderStep', ratio*[1 10]);
set(handles.intensity_slider,'value',handles.intensity_value);
set(handles.intensity_value_text,'string',handles.intensity_value);
guidata(hObject,handles);
plotnow(handles)

% UIWAIT makes GUIfinal wait for user response (see UIRESUME)
% uiwait(handles.figurei);


% --- Outputs from this function are returned to the command line.
function varargout = GUIfinal_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



 function TRE_RMS = treapprox(X,T,RMS_FLE)
% TREAPPROX(MAX_X,T,FLE)
% Calculates an approximate expected RMS TRE value for a given point
% MAX_X = K by N set of fiducial positions; T = K by i target position;
% and RMS_FLE equals the rms value of FLE.
% Ref: Fitzpatrick, West, and Maurer, TMI Oct, i998. Eq. (46).
% 
[K,N] = size(X);
meanX = mean(X')';
Xc = X-meanX*ones(1,N);  % demeaned X
[V, Lambda, U] = svd(Xc);
[K,M_T] = size(T);
Tc = T-meanX*ones(1,M_T);  % T relative to centroid of X
Tv = V'*Tc;   % T referred to principal axes of X
% Distances of target to the markers' three principal axes:
D1 = Tv(2,:).^2 + Tv(3,:).^2;
D2 = Tv(3,:).^2 + Tv(1,:).^2;
D3 = Tv(1,:).^2 + Tv(2,:).^2;
% RMS distances of markers to their three principal axes:
F1 = (Lambda(2,2)^2 + Lambda(3,3)^2)/N;
F2 = (Lambda(3,3)^2 + Lambda(1,1)^2)/N;
F3 = (Lambda(1,1)^2 + Lambda(2,2)^2)/N;

TRE_RMS = sqrt((RMS_FLE^2/N)*(1 + (1/3)*(D1./F1 + D2./F2 + D3./F3)));         
    
% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton1.

function plotnow(handles)
[handles.x_angle,handles.y_angle]=view();    
data_cell_marker = get(handles.DataMarker, 'Data');
data_cell_target = get(handles.DataTarget,'Data');
[rn_marker cn_marker]=size(data_cell_marker);
[rn_target cn_target]=size(data_cell_target);
T=data_cell_target(1:rn_target,1:3);
M=data_cell_marker(1:rn_marker,:);
cla reset;
k = get(handles.loading_dicom,'Enable');
b = strcmp(k,'off');
if b==1
handles.scattering_dicom = scatter3(handles.A,handles.B,handles.C,2,handles.D);
hold on
set(handles.intensity_slider,'Enable','on');
end
h = scatter3(T(:,1), T(:,2), T(:,3),200);
h.MarkerFaceColor = 'k';
h.Marker= 'pentagram';
h.MarkerEdgeColor = 'k';
hold on  
if handles.selectedRow_marker ==0
    if handles.selectedRow_target~=0
        p = scatter3(T(handles.selectedRow_target,1),T(handles.selectedRow_target,2),T(handles.selectedRow_target,3),300);
        p.MarkerFaceColor = [0 1 0];
        p.Marker ='square';
        p.MarkerEdgeColor = [0 1 0];
    end
end
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
x_min_axis = min([min(handles.x_min_marker) min(handles.x_min_target)]);
x_max_axis = max([max(handles.x_max_marker) max(handles.x_max_target)]);
y_min_axis = min([min(handles.y_min_marker) min(handles.y_min_target)]);
y_max_axis = max([max(handles.y_max_marker) max(handles.y_max_target)]);
z_min_axis = min([min(handles.z_min_marker) min(handles.z_min_target)]);
z_max_axis = max([max(handles.z_max_marker) max(handles.z_max_target)]);
set(handles.axes,'XLim',[x_min_axis-10 x_max_axis+10]);
set(handles.axes,'YLim',[y_min_axis-10 y_max_axis+10]);
set(handles.axes,'ZLim',[z_min_axis-10 z_max_axis+10]);
%set(handles.hint_text,'string','Minimum 3 markers should be used');
valid_markers = [];
sum_valid_markers = 0;
 data_actual = [];
 if handles.n_markers>=1
  for i=1:handles.n_markers
     cond=isnan(data_cell_marker(i,:));
         if cond==0
         valid_markers = [valid_markers 1];
         
         data_actual = [data_actual; data_cell_marker(i,:)];
         else
         valid_markers  = [valid_markers 0];
         
         end
  end
 end
         handles.sum_valid_markers = sum(valid_markers);
         if handles.sum_valid_markers>=1
            h = scatter3(M(:,1), M(:,2), M(:,3),250);
            h.MarkerFaceColor ='r' ;
            h.MarkerEdgeColor  = 'r';
            hold on
         end
if handles.selectedRow_target==0
    if handles.selectedRow_marker~=0
        p = scatter3(M(handles.selectedRow_marker,1),M(handles.selectedRow_marker,2),M(handles.selectedRow_marker,3),400);
        p.MarkerFaceColor = [0.8500 0.3250 0.0980];
        p.Marker = 'square';
        p.MarkerEdgeColor = [0.8500 0.3250 0.0980];
    end
end

if handles.selectedRow_target==0
    if handles.selectedRow_marker~=0
        str = sprintf('MARKER %d',handles.selectedRow_marker);
        set(handles.Marker_number,'string',str);
        set(handles.marker_number_copy,'string', str);
        set(handles.x_axis_limits,'visible','on');
        set(handles.y_axis_limits,'visible','on');
        set(handles.z_axis_limits,'visible','on');
        x_min_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,1));
        x_max_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,2));
        X_limits= strcat('X axis:',x_min_string," ",'to'," ",x_max_string);
        set(handles.x_axis_limits,'string', X_limits);
        y_min_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,3));
        y_max_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,4));
        Y_limits= strcat('Y axis:',y_min_string," ",'to'," ",y_max_string);
        set(handles.y_axis_limits,'string',Y_limits);
        z_min_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,5));
        z_max_string = num2str(handles.sliderLimits_marker(handles.selectedRow_marker,6));
        Z_limits= strcat('Z axis:',z_min_string," ",'to'," ",z_max_string);
        set(handles.z_axis_limits,'string',Z_limits);
        handles = update_slider_marker(handles,handles.selectedCol_marker);
    end
end
if handles.selectedRow_marker==0
    if handles.selectedRow_target~=0
    str = sprintf('TARGET %d',handles.selectedRow_target);
    set(handles.Marker_number,'string',str);
    set(handles.marker_number_copy,'string', str);
    set(handles.x_axis_limits,'visible','on');
    set(handles.y_axis_limits,'visible','on');
    set(handles.z_axis_limits,'visible','on');
    x_min_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,1));
    x_max_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,2));
    X_limits= strcat('X axis:',x_min_string," ",'to'," ",x_max_string);
    set(handles.x_axis_limits,'string', X_limits);
    y_min_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,3));
    y_max_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,4));
    Y_limits= strcat('Y axis:',y_min_string," ",'to'," ",y_max_string);
    set(handles.y_axis_limits,'string',Y_limits);
    z_min_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,5));
    z_max_string = num2str(handles.sliderLimits_target(handles.selectedRow_target,6));
    Z_limits= strcat('Z axis:',z_min_string," ",'to'," ",z_max_string);
    set(handles.z_axis_limits,'string',Z_limits);
    handles = update_slider_target(handles,handles.selectedCol_target);
    end
end
set(handles.axes,'view',[handles.x_angle,handles.y_angle]);
       
 
function print_TRE(handles)
data_cell_marker = get(handles.DataMarker, 'Data');
data_cell_target = get(handles.DataTarget,'Data');
[rn_target cn_target]=size(data_cell_target);
T=data_cell_target(1:rn_target,1:3);
[rn_marker cn_marker]=size(data_cell_marker);
M=data_cell_marker(1:rn_marker,:);
valid_markers = [];
sum_valid_markers = 0;
 data_actual = [];
 for i=1:handles.n_markers
     cond=isnan(data_cell_marker(i,:));
         if cond==0
         valid_markers = [valid_markers 1];
         data_actual = [data_actual; data_cell_marker(i,:)];
         else
         valid_markers  = [valid_markers 0];
         
         end
 end
         sum_valid_markers = sum(valid_markers);
         if sum_valid_markers>=1
            h = scatter3(M(:,1), M(:,2), M(:,3),250);
            h.MarkerFaceColor ='r' ;
            h.MarkerEdgeColor  = 'r';
            hold on
         end
if handles.selectedRow_target==0
    if handles.selectedRow_marker~=0
        p = scatter3(M(handles.selectedRow_marker,1),M(handles.selectedRow_marker,2),M(handles.selectedRow_marker,3),400);
        p.MarkerFaceColor = [0.8500 0.3250 0.0980];
        p.Marker = 'square';
        p.MarkerEdgeColor = [0.8500 0.3250 0.0980];
    end
end

       if sum_valid_markers >=3
             set(handles.hint_text,'Visible','off');
              updated_data_target = [];
              for i=1:handles.n_targets
                 FLE=str2double(get(handles.Value_FLE,'string'));
                 TRE=treapprox(data_actual',T(i,1:3)',FLE);
                 TRE= round(TRE, 4);
                 trestring=num2str(TRE);
                 updated_data_target = [updated_data_target;data_cell_target(i,1:3) TRE];
              end
           else
            updated_data_target = [];
            for i=1:handles.n_targets
                updated_data_target = [updated_data_target;data_cell_target(i,1:3) nan];
            end
         end
        set(handles.DataTarget,'Data',updated_data_target);

 
 
% --- Executes on mouse press over figure background.
function figure1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figurei (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)


% --- Executes on button press in Add_Marker.
function Add_Marker_Callback(hObject, eventdata, handles)
% hObject    handle to Add_Marker (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
set(handles.hint_text,'string','Minimum 3 MARKERS should be used');
if handles.n_markers < handles.max_markers
    handles.n_markers = handles.n_markers + 1;
    handles.marker_pts(handles.n_markers,:) = [NaN NaN NaN];
    set(handles.DataMarker,'data',handles.marker_pts);
    handles = update_row_names(handles);
end
% if handles.n_markers==3
%     set(handles.hint_text,'Visible','off');
if handles.n_markers==10
    set(handles.Add_Marker,'Enable','off');
end
guidata(hObject, handles);


% --- Executes on button press in remove_marker.
function remove_marker_Callback(hObject, eventdata, handles)
if handles.selectedRow_marker==0
    errordlg("Select a MARKER to remove",'error','modal');
else
 if handles.selectedRow_marker <=handles.n_markers
 handles.marker_pts(handles.selectedRow_marker,:) = [NaN NaN NaN];
 set(handles.DataMarker,'data',handles.marker_pts);
 handles.sliderLimits_marker(handles.selectedRow_marker,:)=[-150 150 -120 120 -60 60];
 plotnow(handles)
 print_TRE(handles)
 end
end
guidata(hObject, handles);
handles = update_row_names(handles);

% hObject    handle to remove_marker (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

function handles = update_row_names(handles)
for i = 1:handles.n_markers
    str = sprintf('Marker %d', i);
    handles.MarkerNames{i} = str;
end
set(handles.DataMarker,'RowName',handles.MarkerNames);
for i = 2:handles.n_targets
    str = sprintf('Target %d', i);
    handles.TargetNames{i} = str;
end
handles.TargetNames{1} = 'Target 1';
set(handles.DataTarget,'RowName',handles.TargetNames);

function Value_FLE_Callback(hObject, eventdata, handles)
print_TRE(handles)
% hObject    handle to Value_FLE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Value_FLE as text
%        str2double(get(hObject,'String')) returns contents of Value_FLE as a double


% --- Executes during object creation, after setting all properties.
function Value_FLE_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Value_FLE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected cell(s) is changed in DataMarker.
function DataMarker_CellSelectionCallback(hObject, eventdata, handles)
%set(handles.value_slider,'visible','on');
%set(handles.ValuePanel,'visible','on');
data_cell_marker = get(handles.DataMarker,'data');
if size(eventdata.Indices,1)==0
    return;
end
handles.selectedRow_marker = eventdata.Indices(1);
handles.selectedCol_marker = eventdata.Indices(2);
handles.selectedRow_target = 0;
handles.selectedCol_target = 0;
set(handles.x_min_text,'string',handles.x_min_marker(handles.selectedRow_marker));
set(handles.x_max_text,'string',handles.x_max_marker(handles.selectedRow_marker));
set(handles.y_min_text,'string',handles.y_min_marker(handles.selectedRow_marker));
set(handles.y_max_text,'string',handles.y_max_marker(handles.selectedRow_marker));
set(handles.z_min_text,'string',handles.z_min_marker(handles.selectedRow_marker));
set(handles.z_max_text,'string',handles.z_max_marker(handles.selectedRow_marker));
if (handles.selectedRow_marker<=handles.n_markers)
         if isnan(data_cell_marker(handles.selectedRow_marker,1))
            set(handles.x_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,1)); 
         else 
            set(handles.x_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,1));   
         end  
         if isnan(data_cell_marker(handles.selectedRow_marker,2))
          set(handles.y_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,3));
         else
          set(handles.y_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,2));     
        end 
        if isnan(data_cell_marker(handles.selectedRow_marker,3))
           set(handles.z_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,5));
        else
           set(handles.z_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,3));  
        end
end
guidata(hObject,handles);
plotnow(handles)

function handles = update_slider_marker(handles,selectedCol_marker)
        set(handles.x_axis_slider, 'Min', handles.sliderLimits_marker(handles.selectedRow_marker,1));
        set(handles.x_axis_slider, 'Max', handles.sliderLimits_marker(handles.selectedRow_marker,2));
        ratio1 = 1/double(handles.x_axis_slider.Max - handles.x_axis_slider.Min);
        set(handles.x_axis_slider, 'SliderStep', ratio1*[1 10]);
        set(handles.y_axis_slider, 'Min', handles.sliderLimits_marker(handles.selectedRow_marker,3));
        set(handles.y_axis_slider, 'Max', handles.sliderLimits_marker(handles.selectedRow_marker,4));
        ratio2 = 1/double(handles.y_axis_slider.Max - handles.y_axis_slider.Min);
        set(handles.y_axis_slider, 'SliderStep', ratio2*[1 10]); 
        set(handles.z_axis_slider, 'Min', handles.sliderLimits_marker(handles.selectedRow_marker,5));
        set(handles.z_axis_slider, 'Max', handles.sliderLimits_marker(handles.selectedRow_marker,6));
        ratio3 = 1/double(handles.z_axis_slider.Max - handles.z_axis_slider.Min);
        set(handles.z_axis_slider, 'SliderStep', ratio3*[1 10]); 
% hObject    handle to DataMarker (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) currently selecteds
% handles    structure with handles and user datamarker (see GUIDATA)


% --- Executes when entered datamarker in editable cell(s) in DataMarker.
function DataMarker_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to DataMarker (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous datamarker for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the DataMarker property. Empty if DataMarker was not changed
%	Error: error string when failed to convert EditData to appropriate value for DataMarker
% handles    structure with handles and user datamarker (see GUIDATA)
handles.selectedRow_marker= eventdata.Indices(1);
handles.selectedCol_marker= eventdata.Indices(2);
handles.selectedRow_target =0;
handles.selectedCol_target =0;
data_cell_marker = get(handles.DataMarker,'Data');
Min = handles.sliderLimits_marker(handles.selectedRow_marker,2*handles.selectedCol_marker-1);
Max = handles.sliderLimits_marker(handles.selectedRow_marker,2*handles.selectedCol_marker);
if handles.selectedRow_marker>handles.n_markers
    h = errordlg('ADD MARKER','NO MARKER','modal');
else
    if (Min<=data_cell_marker(handles.selectedRow_marker,handles.selectedCol_marker)) && (data_cell_marker(handles.selectedRow_marker,handles.selectedCol_marker)<=Max)
    handles.marker_pts(handles.selectedRow_marker,:)=data_cell_marker(handles.selectedRow_marker,:);
         if isnan(data_cell_marker(handles.selectedRow_marker,1))
            set(handles.x_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,1)); 
         else 
            set(handles.x_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,1));   
         end  
         if isnan(data_cell_marker(handles.selectedRow_marker,2))
          set(handles.y_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,3));
         else
          set(handles.y_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,2));     
        end 
        if isnan(data_cell_marker(handles.selectedRow_marker,3))
           set(handles.z_axis_slider,'value',handles.sliderLimits_marker(handles.selectedRow_marker,5));
        else
           set(handles.z_axis_slider,'value',data_cell_marker(handles.selectedRow_marker,3));  
        end
plotnow(handles)
print_TRE(handles)
    else
        data_cell_marker(handles.selectedRow_marker,handles.selectedCol_marker)=NaN;
        set(handles.DataMarker,'Data',data_cell_marker);
        c=errordlg('Coordinate Limits not in Range','modal');
    end  
end
handles = update_slider_marker(handles,handles.selectedCol_marker);
guidata(hObject, handles);

% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles,varargin)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
set(handles.loading_dicom,'Enable','on');
[hObject,handles]=GUIfinal_OpeningFcn(hObject, eventdata, handles, varargin);
plotnow(handles)
handles = update_row_names(handles);


function localisation_tab_Callback(hObject, eventdata, handles)
% hObject    handle to localisation_tab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
set(handles.limits_changer,'Visible','Off');
set(handles.axes,'Visible','On');
set(handles.limits_tab,'Enable','On');
set(handles.localisation_tab,'Enable','Off');

function limits_tab_Callback(hObject, eventdata, handles)   
set(handles.limits_changer,'Visible','On');
set(handles.axes,'Visible','Off');
set(handles.limits_tab,'Enable','Off');
set(handles.localisation_tab,'Enable','On');

% --- Executes on button press in limits_tab.

% hObject    handle to limits_tab (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)



function x_min_text_Callback(hObject, eventdata, handles)
   
% hObject    handle to handles.x_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of handles.x_min_text as text
%        str2double(get(hObject,'String')) returns contents of handles.x_min_text as a double

    
    
% --- Executes during object creation, after setting all properties.
function x_min_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to handles.x_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_max_text_Callback(hObject, eventdata, handles)
% hObject    handle to z_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z_max_text as text
%        str2double(get(hObject,'String')) returns contents of z_max_text as a double


% --- Executes during object creation, after setting all properties.
function z_max_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_min_text_Callback(hObject, eventdata, handles)
% hObject    handle to z_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% --- Executes during object creation, after setting all properties.
function z_min_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_min_text_Callback(hObject, eventdata, handles)
% hObject    handle to y_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_min_text as text
%        str2double(get(hObject,'String')) returns contents of y_min_text as a double
 

% --- Executes during object creation, after setting all properties.
function y_min_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_min_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_max_text_Callback(hObject, eventdata, handles)
% hObject    handle to y_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_max_text as text
%        str2double(get(hObject,'String')) returns contents of y_max_text as a double


% --- Executes during object creation, after setting all properties.
function y_max_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_max_text_Callback(hObject, eventdata, handles)
 

% hObject    handle to x_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_max_text as text
%        str2double(get(hObject,'String')) returns contents of x_max_text as a double


% --- Executes during object creation, after setting all properties.
function x_max_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_max_text (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in default_in_limits_changer.
function default_in_limits_changer_Callback(hObject, eventdata, handles)
% hObject    handle to default_in_limits_changer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
set(handles.x_min_text,'string','-150');
set(handles.x_max_text,'string','150');
set(handles.y_min_text,'string','-120');
set(handles.y_max_text,'string','120');
set(handles.z_min_text,'string','-60');
set(handles.z_max_text,'string','60');
guidata(hObject, handles);


% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
set(handles.limits_changer,'Visible','Off');
set(handles.axes,'Visible','On');
set(handles.limits_tab,'Enable','On');
set(handles.localisation_tab,'Enable','Off');
guidata(hObject,handles);

 
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)


% --- Executes on button press in ok_button.
function ok_button_Callback(hObject, eventdata, handles)
% hObject    handle to ok_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
 data_cell_marker = get(handles.DataMarker,'Data');
 data_cell_target = get(handles.DataTarget,'Data');
if handles.selectedRow_marker ==0
   i = str2double(erase(handles.TargetNames{handles.selectedRow_target},"Target"));
    handles.x_min_target(i) = str2double(get(handles.x_min_text,'string'));
    handles.x_max_target(i) = str2double(get(handles.x_max_text,'string'));
    handles.y_min_target(i) = str2double(get(handles.y_min_text,'string'));
    handles.y_max_target(i) = str2double(get(handles.y_max_text,'string'));
    handles.z_min_target(i) = str2double(get(handles.z_min_text,'string'));
    handles.z_max_target(i) = str2double(get(handles.z_max_text,'string'));
   if i>handles.n_targets
     k = errordlg("Add the corresponding TARGET first",'error','modal');
   else
       if((handles.x_min_target(i)> data_cell_target(i,1) |  handles.x_max_target(i)<data_cell_target(i,1)) |(handles.y_min_target(i)> data_cell_target(i,2) | handles.y_max_target(i)<data_cell_target(i,2)) | (handles.z_min_target(i)> data_cell_target(i,3) | handles.z_max_target(i)<data_cell_target(i,3)))
      answer = questdlg(' Target Coordinates out of range','Error','Change Limits','Change Coordinates','');
      switch answer
          case 'Change Coordinates'
              handles.sliderLimits_target(i,:) = [handles.x_min_target(i) handles.x_max_target(i) handles.y_min_target(i) handles.y_max_target(i) handles.z_min_target(i) handles.z_max_target(i)];
          case 'Change Limits'
              handles.sliderLimits_target(i,:) = [handles.x_min_prev_target(i) handles.x_max_prev_target(i) handles.y_min_prev_target(i) handles.y_max_prev_target(i) handles.z_min_prev_target(i) handles.z_max_prev_target(i)];
            set(handles.x_min_text,'string',handles.x_min_prev_target(i));
            set(handles.x_max_text,'string',handles.x_max_prev_target(i));
            set(handles.y_min_text,'string',handles.y_min_prev_target(i));
            set(handles.y_max_text,'string',handles.y_max_prev_target(i));
            set(handles.z_min_text,'string',handles.z_min_prev_target(i));
            set(handles.z_max_text,'string',handles.z_max_prev_target(i));
        end  
    else
        handles.sliderLimits_target(i,:) = [handles.x_min_target(i) handles.x_max_target(i) handles.y_min_target(i) handles.y_max_target(i) handles.z_min_target(i) handles.z_max_target(i)];
        handles.x_min_prev_target(i)= handles.x_min_target(i);
        handles.x_max_prev_target(i)= handles.x_max_target(i);
        handles.y_min_prev_target(i)= handles.y_min_target(i);
        handles.y_max_prev_target(i)= handles.y_max_target(i);
        handles.z_min_prev_target(i)= handles.z_min_target(i);
        handles.z_max_prev_target(i)= handles.z_max_target(i);
       end 
   end
   
 else 
    i = str2double(erase(handles.MarkerNames{handles.selectedRow_marker},"Marker"));

    handles.x_min_marker(i) = str2double(get(handles.x_min_text,'string'));
    handles.x_max_marker(i) = str2double(get(handles.x_max_text,'string'));
    handles.y_min_marker(i) = str2double(get(handles.y_min_text,'string'));
    handles.y_max_marker(i) = str2double(get(handles.y_max_text,'string'));
    handles.z_min_marker(i) = str2double(get(handles.z_min_text,'string'));
    handles.z_max_marker(i) = str2double(get(handles.z_max_text,'string'));

  if i>handles.n_markers
      k = errordlg("Add the corresponding MARKER first",'error','modal');
  else
    if((handles.x_min_marker(i)> data_cell_marker(i,1) |  handles.x_max_marker(i)<data_cell_marker(i,1)) |(handles.y_min_marker(i)> data_cell_marker(i,2) | handles.y_max_marker(i)<data_cell_marker(i,2)) | (handles.z_min_marker(i)> data_cell_marker(i,3) | handles.z_max_marker(i)<data_cell_marker(i,3)))
      answer = questdlg(' Target Coordinates out of range','Error','Change Limits','Change Coordinates','');
      switch answer
          case 'Change Coordinates'
              handles.sliderLimits_marker(i,:) = [handles.x_min_marker(i) handles.x_max_marker(i) handles.y_min_marker(i) handles.y_max_marker(i) handles.z_min_marker(i) handles.z_max_marker(i)];
          case 'Change Limits'
              handles.sliderLimits_marker(i,:) = [handles.x_min_prev_marker(i) handles.x_max_prev_marker(i) handles.y_min_prev_marker(i) handles.y_max_prev_marker(i) handles.z_min_prev_marker(i) handles.z_max_prev_marker(i)];
            set(handles.x_min_text,'string',handles.x_min_prev_marker(i));
            set(handles.x_max_text,'string',handles.x_max_prev_marker(i));
            set(handles.y_min_text,'string',handles.y_min_prev_marker(i));
            set(handles.y_max_text,'string',handles.y_max_prev_marker(i));
            set(handles.z_min_text,'string',handles.z_min_prev_marker(i));
            set(handles.z_max_text,'string',handles.z_max_prev_marker(i));
        end  
    else
        handles.sliderLimits_marker(i,:) = [handles.x_min_marker(i) handles.x_max_marker(i) handles.y_min_marker(i) handles.y_max_marker(i) handles.z_min_marker(i) handles.z_max_marker(i)];
        handles.x_min_prev_marker(i)= handles.x_min_marker(i);
        handles.x_max_prev_marker(i)= handles.x_max_marker(i);
        handles.y_min_prev_marker(i)= handles.y_min_marker(i);
        handles.y_max_prev_marker(i)= handles.y_max_marker(i);
        handles.z_min_prev_marker(i)= handles.z_min_marker(i);
        handles.z_max_prev_marker(i)= handles.z_max_marker(i);
    end 
  end
end

set(handles.localisation_tab,'Enable','Off');
set(handles.limits_tab,'Enable','On');
plotnow(handles)
guidata(hObject,handles);

% --- Executes on button press in loading_dicom.
function loading_dicom_Callback(hObject, eventdata, handles)
% hObject    handle to loading_dicom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
%set(handles.intensity_slider,'visible','on');
%set(handles.intensity_slider,'Value',handles.intensity_value);
index=size(handles.names,1)-2;
si_ze=size(handles.P);
t=1;
handles.A=[];
handles.B=[];
handles.C=[];
handles.D=[];
count=0;
for k=1:si_ze(3)   
    for i=1:si_ze(1)
        for j=1:si_ze(2)
            if (handles.P(i,j,k)>handles.intensity_value)
                handles.A(t)=i;
                handles.B(t)=j;
                handles.C(t)=k;
                handles.D(t)=handles.P(i,j,k);
                t=t+1;
            end
        end
    end
end
meanA = (max(handles.A)-min(handles.A))/2;
handles.A = handles.A - min(handles.A)-meanA;
meanB =(max(handles.B)-min(handles.B))/2;
handles.B = handles.B -min(handles.B)-meanB;
meanC = (max(handles.C)-min(handles.C))/2;
handles.C = handles.C-min(handles.C)-meanC;
%handles.scattering_dicom = scatter3(handles.A,handles.B,handles.C,2,handles.D);
%plot3(A,B,C,'.');
hold on
set(handles.loading_dicom,'Enable','Off');
set(handles.remove_dicom,'Enable','On');
guidata(hObject,handles);
plotnow(handles)


% --- Executes on button press in remove_dicom.
function remove_dicom_Callback(hObject, eventdata, handles)
% hObject    handle to remove_dicom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)
[handles.x_angle,handles.y_angle]=view();
cla reset;
set(handles.loading_dicom,'Enable','On');
set(handles.remove_dicom,'Enable','Off');
set(handles.axes,'view',[handles.x_angle,handles.y_angle]);
guidata(hObject,handles);
plotnow(handles)

function z_axis_slider_Callback(hObject, eventdata, handles)
if handles.selectedRow_target == 0
    if (handles.selectedRow_marker<=handles.n_markers)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_marker = get(handles.DataMarker, 'Data');
    data_cell_marker(handles.selectedRow_marker,3)=newval;
    set(handles.DataMarker,'Data',data_cell_marker);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Marker and update value','NO Marker','modal');
    end
    if handles.selectedRow_marker<=handles.n_markers
         handles.marker_pts(handles.selectedRow_marker,:)=data_cell_marker(handles.selectedRow_marker,:);
    end
else 
     if (handles.selectedRow_target<=handles.n_targets)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_target = get(handles.DataTarget, 'Data');
    data_cell_target(handles.selectedRow_target,3)=newval;
    set(handles.DataTarget,'Data',data_cell_target);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Target and update value','NO Target','modal');
    end
    if handles.selectedRow_target<=handles.n_targets
             handles.target_pts(handles.selectedRow_target,1:3)=data_cell_target(handles.selectedRow_target,1:3);
    end
end
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function z_axis_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_axis_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function y_axis_slider_Callback(hObject, eventdata, handles)
% hObject    handle to y_axis_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
if handles.selectedRow_target == 0
    if (handles.selectedRow_marker<=handles.n_markers)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_marker = get(handles.DataMarker, 'Data');
    data_cell_marker(handles.selectedRow_marker,2)=newval;
    set(handles.DataMarker,'Data',data_cell_marker);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Marker and update value','NO Marker','modal');
    end
    if handles.selectedRow_marker<=handles.n_markers
         handles.marker_pts(handles.selectedRow_marker,:)=data_cell_marker(handles.selectedRow_marker,:);
    end
else 
     if (handles.selectedRow_target<=handles.n_targets)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_target = get(handles.DataTarget, 'Data');
    data_cell_target(handles.selectedRow_target,2)=newval;
    set(handles.DataTarget,'Data',data_cell_target);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Target and update value','NO Target','modal');
    end
    if handles.selectedRow_target<=handles.n_targets
             handles.target_pts(handles.selectedRow_target,1:3)=data_cell_target(handles.selectedRow_target,1:3);
    end
end
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function y_axis_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_axis_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function x_axis_slider_Callback(hObject, eventdata, handles)
% hObject    handle to x_axis_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user datamarker (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
if handles.selectedRow_target == 0
    if (handles.selectedRow_marker<=handles.n_markers)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_marker = get(handles.DataMarker, 'Data');
    data_cell_marker(handles.selectedRow_marker,1)=newval;
    set(handles.DataMarker,'Data',data_cell_marker);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Marker and update value','NO Marker','modal');
    end
    if handles.selectedRow_marker<=handles.n_markers
         handles.marker_pts(handles.selectedRow_marker,:)=data_cell_marker(handles.selectedRow_marker,:);
    end
else 
     if (handles.selectedRow_target<=handles.n_targets)
    newval = hObject.Value;                         %get value from the slider
    newval = round(newval);                         %round off this value
    set(hObject, 'Value', newval);                  %set slider position to rounded off value  
    data_cell_target = get(handles.DataTarget, 'Data');
    data_cell_target(handles.selectedRow_target,1)=newval;
    set(handles.DataTarget,'Data',data_cell_target);
    plotnow(handles)
    print_TRE(handles)
    else 
       q=errordlg('Add Target and update value','NO Target','modal');
    end
    if handles.selectedRow_target<=handles.n_targets
             handles.target_pts(handles.selectedRow_target,1:3)=data_cell_target(handles.selectedRow_target,1:3);
    end
end
guidata(hObject, handles);

 
% --- Executes during object creation, after setting all properties.
function x_axis_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_axis_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes when entered data in editable cell(s) in DataTarget.
function DataTarget_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to DataTarget (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
handles.selectedRow_target= eventdata.Indices(1);
handles.selectedCol_target= eventdata.Indices(2);
data_cell_target = get(handles.DataTarget,'Data');
Min = handles.sliderLimits_target(handles.selectedRow_target,2*handles.selectedCol_target-1);
Max = handles.sliderLimits_target(handles.selectedRow_target,2*handles.selectedCol_target);

if handles.selectedRow_target>handles.n_targets
    h = errordlg('ADD TARGET','NO TARGET','modal');
else
    if (Min<=data_cell_target(handles.selectedRow_target,handles.selectedCol_target)) && (data_cell_target(handles.selectedRow_target,handles.selectedCol_target)<=Max)
    handles.target_pts(handles.selectedRow_target,1:3)=data_cell_target(handles.selectedRow_target,1:3);
     set(handles.x_axis_slider,'value',data_cell_target(handles.selectedRow_target,1));
        set(handles.y_axis_slider,'value',data_cell_target(handles.selectedRow_target,2));
        set(handles.z_axis_slider,'value',data_cell_target(handles.selectedRow_target,3));  
plotnow(handles)
print_TRE(handles)
    else
        data_cell_target(handles.selectedRow_target,handles.selectedCol_target)=0;
        set(handles.DataTarget,'Data',data_cell_target);
        c=errordlg('Coordinate Limits not in Range','modal');
    end  
end
handles = update_slider_target(handles,handles.selectedCol_target);
guidata(hObject, handles);



% --- Executes on button press in add_target.
function add_target_Callback(hObject, eventdata, handles)
% hObject    handle to add_target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.hint_text,'visible','on')
if handles.n_targets < handles.max_targets
    handles.n_targets = handles.n_targets + 1;
    handles.target_pts(handles.n_targets,:) = [0 0 0 nan];
    set(handles.DataTarget,'data',handles.target_pts);
    handles = update_row_names(handles);
end
guidata(hObject, handles);
plotnow(handles)
print_TRE(handles)


function remove_target_Callback(hObject, eventdata, handles)
% hObject    handle to remove_target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if handles.selectedRow_target==0
    errordlg("Select a TARGET to remove",'error','modal');
else
if handles.n_targets==1
     f=errordlg(' All TARGETS cannot be deleted','Error','modal');
elseif handles.selectedRow_target<=handles.n_targets
    handles.target_pts(handles.selectedRow_target,:) = [];
    handles.TargetNames{handles.n_targets}='';
    
    set(handles.DataTarget,'data',handles.target_pts);
    for i =handles.selectedRow_target+1:handles.n_targets
        handles.x_min_target(i-1)=handles.x_min_target(i);
        handles.x_max_target(i-1)=handles.x_max_target(i);
        handles.y_min_target(i-1)=handles.y_min_target(i);
        handles.y_max_target(i-1)=handles.y_max_target(i);
        handles.z_min_target(i-1)=handles.z_min_target(i);
        handles.z_max_target(i-1)=handles.z_max_target(i);
        handles.sliderLimits_target(i-1,:) =[handles.x_min_target(i) handles.x_max_target(i) handles.y_min_target(i) handles.y_max_target(i) handles.z_min_target(i) handles.z_max_target(i)]; 
    end
    handles.sliderLimits_target(handles.n_targets,:)=[-150 150 -120 120 -60 60];
    handles.n_targets = handles.n_targets-1;
    if handles.selectedRow_target>1
        handles.selectedRow_target =handles.selectedRow_target-1;
    else
        handles.selectedRow_target =1;
    end
    plotnow(handles)
    print_TRE(handles)
end
end

guidata(hObject, handles);
handles = update_row_names(handles);


% --- Executes when selected cell(s) is changed in DataTarget.
function DataTarget_CellSelectionCallback(hObject, eventdata, handles)
% hObject    handle to DataTarget (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) currently selecteds
% handles    structure with handles and user data (see GUIDATA)
data_cell_target = get(handles.DataTarget,'data');
if size(eventdata.Indices,1)==0
    return;
end
handles.selectedRow_target = eventdata.Indices(1);
handles.selectedCol_target = eventdata.Indices(2);
handles.selectedRow_marker =0;
handles.selectedCol_marker =0;
set(handles.x_min_text,'string',handles.x_min_target(handles.selectedRow_target));
set(handles.x_max_text,'string',handles.x_max_target(handles.selectedRow_target));
set(handles.y_min_text,'string',handles.y_min_target(handles.selectedRow_target));
set(handles.y_max_text,'string',handles.y_max_target(handles.selectedRow_target));
set(handles.z_min_text,'string',handles.z_min_target(handles.selectedRow_target));
set(handles.z_max_text,'string',handles.z_max_target(handles.selectedRow_target));
data_cell_target = get(handles.DataTarget,'data');
if (handles.selectedRow_target<=handles.n_targets)
    set(handles.x_axis_slider,'value',data_cell_target(handles.selectedRow_target,1));
    set(handles.y_axis_slider,'value',data_cell_target(handles.selectedRow_target,2));
    set(handles.z_axis_slider,'value',data_cell_target(handles.selectedRow_target,3));
end

guidata(hObject,handles);
plotnow(handles)

function handles = update_slider_target(handles,selectedCol_target)
        set(handles.x_axis_slider, 'Min', handles.sliderLimits_target(handles.selectedRow_target,1));
        set(handles.x_axis_slider, 'Max', handles.sliderLimits_target(handles.selectedRow_target,2));
        ratio1 = 1/double(handles.x_axis_slider.Max - handles.x_axis_slider.Min);
        set(handles.x_axis_slider, 'SliderStep', ratio1*[1 10]);
        set(handles.y_axis_slider, 'Min', handles.sliderLimits_target(handles.selectedRow_target,3));
        set(handles.y_axis_slider, 'Max', handles.sliderLimits_target(handles.selectedRow_target,4));
        ratio2 = 1/double(handles.y_axis_slider.Max - handles.y_axis_slider.Min);
        set(handles.y_axis_slider, 'SliderStep', ratio2*[1 10]); 
        set(handles.z_axis_slider, 'Min', handles.sliderLimits_target(handles.selectedRow_target,5));
        set(handles.z_axis_slider, 'Max', handles.sliderLimits_target(handles.selectedRow_target,6));
        ratio3 = 1/double(handles.z_axis_slider.Max - handles.z_axis_slider.Min);
        set(handles.z_axis_slider, 'SliderStep', ratio3*[1 10]); 


% --- Executes on slider movement.
function intensity_slider_Callback(hObject, eventdata, handles)
% hObject    handle to intensity_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.intensity_slider,'Enable','off');
newval = hObject.Value;                         %get value from the slider
newval = round(newval);                         %round off this value
set(hObject, 'Value', newval);
handles.intensity_value = newval;
set(handles.intensity_value_text,'string',handles.intensity_value);
guidata(hObject,handles);
loading_dicom_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function intensity_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to intensity_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
function intensity_value_text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Value_FLE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function intensity_value_text_Callback(hObject, eventdata, handles)
set(handles.intensity_slider, 'Enable','off');
handles.intensity_value = str2double(get(handles.intensity_value_text, 'string'));
set(handles.intensity_slider, 'value', handles.intensity_value);
guidata(hObject,handles);
loading_dicom_Callback(hObject, eventdata, handles)    