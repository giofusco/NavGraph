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

% Last Modified by GUIDE v2.5 19-Mar-2019 12:59:53

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
setappdata(gcf, 'prevClickedEdge', 0);
setappdata(gcf, 'createEdge', 0);
setappdata(gcf, 'edgeNodes', []);
setappdata(gcf, 'currentNode', 0);
setappdata(gcf, 'nodesHandles', []);
setappdata(gcf, 'edgesHandles', []);
setappdata(gcf, 'nodes', containers.Map('KeyType','uint32','ValueType','any'));
setappdata(gcf, 'CLR_DEST', [0,0,1]);
setappdata(gcf, 'CLR_CTRL', [0,1,0]);
setappdata(gcf, 'CLR_LINK', [1,0,0]);
setappdata(gcf, 'imageSize', []);
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
    h = imshow(info.map,'Parent', handles.axes1);    
    setappdata(gcf, 'imageHandle', h);
    setappdata(gcf, 'imageSize', size(info.map));

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
setappdata(gcf, 'prevClickedNode', 0);
setappdata(gcf, 'currentNode', 0);
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
imsize = getappdata(gcf, 'imageSize');
if (x >= 1 && x <= imsize(2) && y >= 1 && y <= imsize(1))
    nodesID = getappdata(gcf, 'nodesID') + 1;
    setappdata(gcf, 'nodesID', nodesID);
    nodeinfo.id = nodesID;
    nodeinfo.type = type;
    nodeinfo.position = [x y];
    nodeinfo.floor = getappdata(gcf, 'currentFloor');
    nodeinfo.label = num2str(nodesID);
    nodeinfo.edges = [];
    nodes(int32(nodesID)) = nodeinfo;
    color = getNodeColor(type);
    nodesHandles = getappdata(gcf, 'nodesHandles');
    nodesHandles(end+1) = line(x, y, 'marker', 'O', 'LineWidth',1, ...
            'MarkerSize',8, ...
            'MarkerEdgeColor', 'm', ...
            'MarkerFaceColor',color, ... 
            'userdata', nodesID, ...
            'ButtonDownFcn', {@highlightNode handles});

    setappdata(gcf, 'nodes', nodes);
    setappdata(gcf, 'nodesHandles', nodesHandles);
end

% --- Executes on button press in pushbutton_addDestinationNode.
function pushbutton_addDestinationNode_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_addDestinationNode (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles = addNode(handles, 'destination');
guidata(hObject, handles);

function highlightNode(hObject, eventdata, handles)

create_edge = getappdata(gcf, 'createEdge');
idx = get(hObject, 'userdata');
if (create_edge)
    edgeNodes = getappdata(gcf, 'edgeNodes');
    edgeNodes(end+1) = idx;
    set(hObject, 'MarkerEdgeColor', 'c');
    set(hObject, 'MarkerSize', 15);
    setappdata(gcf, 'edgeNodes', edgeNodes);
    if (length(edgeNodes) == 2)
        setappdata(gcf, 'createEdge', 0);
        if (edgeNodes(1) ~= edgeNodes(2))
            createEdge();
        end
        disableSelection(edgeNodes, handles);
        setappdata(gcf, 'edgeNodes', []);
    end
else
%     idx = get(hObject, 'userdata');
    
    prevClickedNode = getappdata(gcf, 'prevClickedNode');

    if (prevClickedNode > 0 || prevClickedNode == idx)
        disableSelection(prevClickedNode, handles)
    end
    if prevClickedNode ~= idx
        nodes = getappdata(gcf, 'nodes');
        nodeinfo = nodes(idx);
        set(hObject, 'MarkerEdgeColor', 'y');
        set(hObject, 'MarkerSize', 11);
        draggable(hObject, 'endfcn',@updateNodePosition);
        set(handles.edit_node_label, 'String', nodeinfo.label);
        setappdata(gcf, 'currentNode', idx);
       % handles.prevClickedNode = idx;
    end

    if (prevClickedNode == idx)
        setappdata(gcf, 'prevClickedNode', 0);
    else
        setappdata(gcf, 'prevClickedNode', idx);
    end
end

function disableSelection(id, handles)
nHandles = getappdata(gcf, 'nodesHandles');
for i = id
    if (i > 0)
        set(nHandles(i), 'MarkerEdgeColor', 'm');
        set(nHandles(i), 'MarkerSize', 8);
        draggable(nHandles(i), 'off');
        set(nHandles(i),'ButtonDownFcn', {@highlightNode handles})
    end
end
setappdata(gcf, 'prevClickedNode', 0);
setappdata(gcf, 'nodesHandles', nHandles);

function createEdge()
nodes = getappdata(gcf, 'nodes');
edgeNodes = getappdata(gcf, 'edgeNodes');
imsize = getappdata(gcf, 'imageSize');
n1 = nodes(edgeNodes(1));
n2 = nodes(edgeNodes(2));

x = [n1.position(1) n2.position(1)];
y = [n1.position(2) n2.position(2)];
s2 = atan2d( ( y(2)-(imsize(1)-1) ) - (y(1)-(imsize(1)-1)), x(2) - x(1) );
s1 = atan2d( ( y(1)-(imsize(1)-1) ) - (y(2)-(imsize(1)-1)), x(1) - x(2) );
edgeinfo1 = [];
edgeinfo2 = [];
edgeinfo1.dest = edgeNodes(2);
edgeinfo1.slope = s1

edgeinfo2.dest = edgeNodes(1);
edgeinfo2.slope = s2

%edgesHandles = getappdata(gcf, 'edgesHandles');
edgeHandle = line(x, y, 'marker', 'O', 'LineWidth',1, ...
        'userdata', edgeNodes);
uistack(edgeHandle, 'bottom');
uistack(getappdata(gcf, 'imageHandle'), 'bottom');
edgeinfo1.handle = edgeHandle;
edgeinfo2.handle = edgeHandle;
    
n1.edges = [ n1.edges edgeinfo1 ];
n2.edges = [ n2.edges edgeinfo2 ];
nodes(edgeNodes(1)) = n1;
nodes(edgeNodes(2)) = n2;
setappdata(gcf, 'prevClickedNode', 0);
setappdata(gcf, 'nodes', nodes);

function highlightEdge(hObject, eventdata, handles)
%lastSelEdge = setappdata(gcf, 'prevClickedEdge', 0);
get(hObject)
% set(hObject, 'LineWidth', 2);
    
function updateNodePosition(hObject, eventdata)
id = getappdata(gcf, 'prevClickedNode');
nodes = getappdata(gcf, 'nodes');
nodeinfo = nodes(id);
new_x = get(hObject, 'XData');
new_y = get(hObject, 'YData');
nodeinfo.position = [new_x new_y];
nodes(id) = nodeinfo;
setappdata(gcf, 'nodes', nodes);
%update edges
updateEdgesLayout(id)

function updateEdgesLayout(id)
nodes = getappdata(gcf, 'nodes');
edges = nodes(id).edges;
n1 = nodes(id);
for e = 1 : length(edges)
    if (edges(e).dest > 0)
        n2 = nodes(edges(e).dest);
        x = [n1.position(1) n2.position(1)];
        y = [n1.position(2) n2.position(2)];
        set(edges(e).handle, 'XData', x);
        set(edges(e).handle, 'YData', y);
    end
end


function handles = plot_nodes(handles)
nodesHandles = [];
currentFloor = getappdata(gcf, 'currentFloor');
nodes = getappdata(gcf, 'nodes');
for n = 1 : length(nodes)
    nodeinfo = nodes(n);
    if nodeinfo.floor == currentFloor
        color = getNodeColor(nodeinfo.type);
        nodesHandles(end+1) = line(nodeinfo.position(1), nodeinfo.position(2), 'marker', 'O', 'LineWidth',1, ...
            'MarkerSize',8, ...
            'MarkerEdgeColor', 'm', ...
            'MarkerFaceColor',color, ... 
            'userdata', nodeinfo.id, ...
            'ButtonDownFcn', {@highlightNode handles});
    end
end

setappdata(gcf, 'nodesHandles', nodesHandles);

% --- Executes on button press in pushbutton_createEdge.
function pushbutton_createEdge_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_createEdge (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
setappdata(gcf, 'createEdge', 1);
setappdata(gcf, 'edge_cnt', 0);
prevClickedNode = getappdata(gcf, 'prevClickedNode');
disableSelection(prevClickedNode, handles);

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



function edit_nodeID_Callback(hObject, eventdata, handles)
% hObject    handle to edit_nodeID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_nodeID as text
%        str2double(get(hObject,'String')) returns contents of edit_nodeID as a double


% --- Executes during object creation, after setting all properties.
function edit_nodeID_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_nodeID (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_node_label_Callback(hObject, eventdata, handles)
% hObject    handle to edit_node_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_node_label as text
%        str2double(get(hObject,'String')) returns contents of edit_node_label as a double

idx = getappdata(gcf, 'currentNode');
if (idx > 0)
    nodes = getappdata(gcf, 'nodes');
    nodeinfo = nodes(idx);
    nodeinfo.label = get(hObject,'String');
    nodes(idx) = nodeinfo;
    setappdata(gcf, 'nodes', nodes);
end

% --- Executes during object creation, after setting all properties.
function edit_node_label_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_node_label (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on edit_node_label and none of its controls.
function edit_node_label_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to edit_node_label (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)
