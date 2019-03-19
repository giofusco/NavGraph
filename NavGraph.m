function varargout = NavGraph(varargin)
% NAVGRAPH MATLAB code for NavGraph.fig
%      NAVGRAPH, by itself, creates a new NAVGRAPH or raises the existing
%      singleton*.
%
%      H = NAVGRAPH returns the handle to a new NAVGRAPH or the handle to
%      the existing singleton*.
%
%      NAVGRAPH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in NAVGRAPH.M with the given input arguments.
%
%      NAVGRAPH('Property','Value',...) creates a new NAVGRAPH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before NavGraph_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to NavGraph_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help NavGraph

% Last Modified by GUIDE v2.5 18-Mar-2019 16:10:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @NavGraph_OpeningFcn, ...
                   'gui_OutputFcn',  @NavGraph_OutputFcn, ...
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


% --- Executes just before NavGraph is made visible.
function NavGraph_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to NavGraph (see VARARGIN)

% Choose default command line output for NavGraph
handles.output = hObject;
%handles.nodes = containers.Map('KeyType','uint32','ValueType','any');

setappdata(gcf, 'nodesID', 0);
setappdata(gcf, 'currentFloor', 0);
setappdata(gcf, 'prevClickedNode', 0);
setappdata(gcf, 'nodesHandles', []);
setappdata(gcf, 'nodes', containers.Map('KeyType','uint32','ValueType','any'));
setappdata(gcf, 'CLR_DEST', [0,0,1]);
setappdata(gcf, 'CLR_CTRL', [0,1,0]);
setappdata(gcf, 'CLR_LINK', [1,0,0]);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes NavGraph wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = NavGraph_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_loadBuilding.
function pushbutton_loadBuilding_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_loadBuilding (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[file,path] = uigetfile('*.yml');
if isequal(file,0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path, file)]);
    handles = parseBuildingFile(hObject, handles, fullfile(path, file), path);
    %disp(handles.floorsInfo);
end
guidata(hObject, handles);


function handles = parseBuildingFile(hObject, handles, filename, path)
    addpath(('YAMLMatlab_0.4.3'));
    result = ReadYaml(filename);
    popup_vals = {};
    for l = 1 : length(result.floors)
        info = [];
        info.map = imread(fullfile(path, result.floors{l}.walls));
        handles.rois = zeros(size(info.map));
        info.scale = result.floors{l}.scale;
        handles.floorsInfo(num2str(result.floors{l}.id)) = info; 
        popup_vals{l} = num2str(result.floors{l}.id);
        
%         handles.FloorsDropDown.Items{end+1} = num2str(result.floors{l}.id);
    end
    set(handles.popupmenu_floors, 'String', popup_vals);
    % visualize map on file load
    contents = cellstr(get(handles.popupmenu_floors,'String'));
    setappdata(gcf, 'currentFloor', contents{get(handles.popupmenu_floors,'Value')});
    visualizeFloor(handles, contents{get(handles.popupmenu_floors,'Value')});
    
function visualizeFloor(handles, floorNum)
    info = handles.floorsInfo(floorNum);
    imshow(info.map,'Parent', handles.axes1);    


% --- Executes on selection change in popupmenu_floors.
function popupmenu_floors_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu_floors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu_floors contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu_floors
contents = cellstr(get(hObject,'String'));
currentFloor = contents{get(hObject,'Value')};
setappdata(gcf, 'currentFloor', currentFloor);
visualizeFloor(handles, contents{get(hObject,'Value')});
handles = plot_nodes(handles);
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function popupmenu_floors_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu_floors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function color = getNodeColor(type)
color = [0 0 0];
if strcmp(type, 'destination')
    color = getappdata(gcf, 'CLR_DEST');
elseif strcmp(type, 'control')
    color = getappdata(gcf, 'CLR_CTRL');
elseif strcmp(type, 'link')
    color = getappdata(gcf, 'CLR_LINK');
end

function handles = addNode(handles, type)
nodes = getappdata(gcf, 'nodes');
[x, y] = ginput(1);
nodesID = getappdata(gcf, 'nodesID') + 1;
setappdata(gcf, 'nodesID', nodesID);
nodeinfo.id = nodesID;
nodeinfo.type = type;
nodeinfo.position = [x y];
nodeinfo.floor = getappdata(gcf, 'currentFloor');
nodes(int32(nodesID)) = nodeinfo;
color = getNodeColor(type);

nodesHandles(end+1) = line(x, y, 'marker', 'O', 'LineWidth',2, ...
        'MarkerSize',10, ...
        'MarkerFaceColor',color, ... 
        'userdata', nodesID, ...
        'ButtonDownFcn', @highlightNode);
    
setappdata(gcf, 'nodes', nodes);
setappdata(gcf, 'nodesHandles', nodesHandles);

% --- Executes on button press in pushbutton_addDestinationNode.
function pushbutton_addDestinationNode_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_addDestinationNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = addNode(handles, 'destination');
guidata(hObject, handles);

function highlightNode(hObject, eventdata)
idx = get(hObject, 'userdata');
nHandles = getappdata(gcf, 'nodesHandles');
prevClickedNode = getappdata(gcf, 'prevClickedNode');

if (prevClickedNode > 0 || prevClickedNode == idx)
    set(nHandles(prevClickedNode), 'MarkerEdgeColor', 'b');
    set(nHandles(prevClickedNode), 'MarkerSize', 10);
    draggable(nHandles(prevClickedNode), 'off');
%   handles.prevClickedNode = idx;
end
if prevClickedNode ~= idx
    set(hObject, 'MarkerEdgeColor', 'y');
    set(hObject, 'MarkerSize', 15);
    draggable(hObject, 'endfcn',@updateNodePosition);
   % handles.prevClickedNode = idx;
end

if (prevClickedNode == idx)
    setappdata(gcf, 'prevClickedNode', 0);
else
    setappdata(gcf, 'prevClickedNode', idx);
end

function updateNodePosition(hObject, eventdata)
get(hObject)
id = getappdata(gcf, 'prevClickedNode');
nodes = getappdata(gcf, 'nodes');
nodeinfo = nodes(id);
new_x = get(hObject, 'XData');
new_y = get(hObject, 'YData');
nodeinfo.position = [new_x new_y];
nodes(id) = nodeinfo;
setappdata(gcf, 'nodes', nodes);


function handles = plot_nodes(handles)

nodesHandles = [];
currentFloor = getappdata(gcf, 'currentFloor');
nodes = getappdata(gcf, 'nodes');
for n = 1 : length(nodes)
    nodeinfo = nodes(n);
    if nodeinfo.floor == currentFloor
        color = getNodeColor(nodeinfo.type);
        nodesHandles(end+1) = line(nodeinfo.position(1), nodeinfo.position(2), 'marker', 'O', 'LineWidth',2, ...
            'MarkerSize',10, ...
            'MarkerFaceColor',color, ... 
            'userdata', nodeinfo.id, ...
            'ButtonDownFcn', @highlightNode);
    end
end

setappdata(gcf, 'nodesHandles', nodesHandles);

% --- Executes on button press in pushbutton_createEdge.
function pushbutton_createEdge_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_createEdge (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton_addControlNode.
function pushbutton_addControlNode_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_addControlNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = addNode(handles, 'control');
guidata(hObject, handles);

% --- Executes on button press in pushbutton_addLinkNode.
function pushbutton_addLinkNode_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_addLinkNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = addNode(handles, 'link');
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function pushbutton_addControlNode_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_addControlNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called