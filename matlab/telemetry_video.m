function [mp4File, pngFile] = telemetry_video(outDir, varargin)
%TELEMETRY_VIDEO Layout-derived telemetry from an exported output directory.
% telemetry_video(outDir) writes mission_s_telemetry.mp4 at 24 fps.
% telemetry_video(outDir, 'Frames', [1 1], 'WriteVideo', false) writes one PNG.
% Options: CSV, Scene, FramesDir, FPS, Frames, WriteVideo, PNGDir, MP4.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(outDir), outDir = fullfile(here, '..', 'out', 'revised'); end
p = inputParser;
addParameter(p, 'CSV', fullfile(outDir, 'trajectory.csv'));
addParameter(p, 'Scene', fullfile(outDir, 'scene.json'));
addParameter(p, 'FramesDir', fullfile(outDir, 'frames'));
addParameter(p, 'FPS', 24, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'Frames', [], @(x) isempty(x) || (isnumeric(x) && numel(x) == 2));
addParameter(p, 'WriteVideo', true, @(x) islogical(x) && isscalar(x));
addParameter(p, 'PNGDir', fullfile(outDir, 'telemetry_matlab'));
addParameter(p, 'MP4', fullfile(outDir, 'mission_s_telemetry.mp4'));
parse(p, varargin{:}); opts = p.Results;
csvFile = opts.CSV; framesDir = opts.FramesDir; mp4File = opts.MP4; pngFile = '';

T = readtable(csvFile, 'TextType', 'string');
n = height(T); t = T.t;
assert(n > 0, 'Telemetry CSV contains no rows');
[groups, labels, throttle, kinds, layoutNote] = telemetryGroups(T, opts.Scene, fullfile(here, '..', 'thruster_layout.json'));
ng = numel(groups);
range = [1 n]; if ~isempty(opts.Frames), range = opts.Frames; end
assert(all(range == fix(range)) && 1 <= range(1) && range(1) <= range(2) && range(2) <= n, ...
    'Frames must be an inclusive integer range inside the CSV');
haveFrames = ~isempty(framesDir) && (isfolder(framesDir) || ~ismember('FramesDir', p.UsingDefaults));
if haveFrames
    required = arrayfun(@(i) fullfile(framesDir, sprintf('frame_%04d.png', i)), ...
        range(1):range(2), 'UniformOutput', false);
    missing = required(~cellfun(@isfile, required));
    if ~isempty(missing)
        error('MissionS:MissingRenderFrames', 'Missing rendered frames for range %d:%d: %s', ...
            range(1), range(2), strjoin(missing(1:min(8, numel(missing))), ', '));
    end
end
if ~exist(opts.PNGDir, 'dir'), mkdir(opts.PNGDir); end
modes = containers.Map( ...
    {'hold','launch','boost','coast','roll','roll_settle','pitch_kick','pitch_coast','pitch_catch', ...
     'descent','landing_burn','settle','rollout'}, ...
    {'HOLD','LAUNCH (V8 + e-motors)','SUPERDRACO BOOST','COAST','BARREL ROLL','ROLL SETTLE', ...
     'BACKFLIP KICK','BACKFLIP','BACKFLIP CATCH','DESCENT','LANDING BURN','SETTLE','ROLLOUT'});
BG = [11 13 16]/255; PANEL = [20 24 29]/255; FG = [232 234 237]/255; DIM = [138 146 156]/255;
cx = [79 195 247]/255; cy = [255 183 77]/255; cz = [129 199 132]/255; cw = [1 1 1];
csd = [255 138 80]/255; cdr = [159 184 255]/255; ccopv = [206 147 216]/255;

fig = figure('Color', BG, 'Units', 'pixels', 'Position', [30 30 1280 720], 'Visible', 'off', ...
             'InvertHardcopy', 'off', 'Resize', 'off', 'PaperUnits', 'inches', ...
             'PaperPositionMode', 'manual', 'PaperPosition', [0 0 19.2 10.8], 'PaperSize', [19.2 10.8]);
figCleanup = onCleanup(@() close(fig));
% video area, 1344 x 756 top-left
axv = axes(fig, 'Units', 'normalized', 'Position', [0 0.30 0.70 0.70]);
img = image(axv, zeros(756, 1344, 3)); axis(axv, 'off'); axv.XLim = [0.5 1344.5]; axv.YLim = [0.5 756.5];
hold(axv, 'on');
if ~haveFrames
    text(axv, 672, 378, '3D render goes here (out/frames)', 'Color', [.25 .27 .3], 'FontSize', 22, ...
         'HorizontalAlignment', 'center');
end
hud = text(axv, 20, 30, '', 'Color', FG, 'FontName', 'Consolas', 'FontSize', 15, 'VerticalAlignment', 'top', ...
           'BackgroundColor', [0 0 0], 'Margin', 6);
mlab = text(axv, 1324, 30, '', 'Color', [1 .8 .5], 'FontSize', 17, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'BackgroundColor', [0 0 0], 'Margin', 6);
% readout
axr = axes(fig, 'Position', [0.71 0.86 0.28 0.13], 'Visible', 'off'); axr.XLim = [0 1]; axr.YLim = [0 1];
readout = text(axr, 0, 1, '', 'Color', FG, 'FontName', 'Consolas', 'FontSize', 11.5, 'VerticalAlignment', 'top');
% throttle bars
axb = axes(fig, 'Position', [0.815 0.39 0.17 0.44], 'Color', PANEL, 'XColor', DIM, 'YColor', FG);
hold(axb, 'on'); grid(axb, 'on'); axb.GridColor = [.35 .38 .42]; axb.Layer = 'bottom';
yb = ng:-1:1;
bh = gobjects(ng, 1); vt = gobjects(ng, 1);
for k = 1:ng
    if strcmp(kinds{k}, 'superdraco'), col = csd; else, col = cdr; end
    bh(k) = barh(axb, yb(k), 0, 0.62, 'FaceColor', col, 'EdgeColor', 'none');
    vt(k) = text(axb, 2, yb(k), '', 'Color', FG, 'FontSize', 8);
end
axb.XLim = [0 118]; axb.YLim = [0.4 ng+0.6]; axb.XTick = 0:20:100;
axb.YTick = 1:ng; axb.YTickLabel = fliplr(labels); axb.FontName = 'Consolas';
axb.FontSize = 8; axb.TickLabelInterpreter = 'none';
title(axb, 'Delivered thrust (% of channel max)', 'Color', FG, 'FontSize', 10);
annotation(fig, 'textbox', [0.71 0.275 0.285 0.08], 'String', layoutNote, ...
    'Color', DIM, 'EdgeColor', 'none', 'FontSize', 7, 'Interpreter', 'none');
% traces
W = 0.215; gap = 0.033; x0 = 0.035;
mk = @(i) axes(fig, 'Position', [x0 + (i-1)*(W + gap), 0.05, W, 0.20], 'Color', PANEL, 'XColor', DIM, ...
               'YColor', DIM, 'XLim', [t(1) t(end)], 'NextPlot', 'add', 'XGrid', 'on', 'YGrid', 'on', ...
               'GridColor', [.35 .38 .42], 'FontSize', 8);
axg = mk(1); axp = mk(2); axq = mk(3); axc = mk(4);
title(axg, 'Accelerometer at CG (g, body axes)', 'Color', FG, 'FontSize', 10);
title(axp, 'Propellant remaining (%, MMH & NTO at O/F 1.65)', 'Color', FG, 'FontSize', 10);
title(axq, 'Helium COPVs (bar)', 'Color', FG, 'FontSize', 10);
title(axc, 'CG shift from launch (mm, body axes)', 'Color', FG, 'FontSize', 10);
L = @(ax, col, w, ls) animatedline(ax, 'Color', col, 'LineWidth', w, 'LineStyle', ls);
lg = {L(axg, cx, 1.4, '-'), L(axg, cy, 1.4, '-'), L(axg, cz, 1.4, '-'), L(axg, cw, 1.0, '-')};
lp = {L(axp, cy, 1.4, '-'), L(axp, cx, 1.0, '--')};
lq = {L(axq, ccopv, 1.4, '-')};
lc = {L(axc, cx, 1.4, '-'), L(axc, cy, 1.4, '-'), L(axc, cz, 1.4, '-')};
dcg = 1000*[T.cg_x - T.cg_x(1), T.cg_y - T.cg_y(1), T.cg_z - T.cg_z(1)];
axg.YLim = [min([T.g_x; T.g_y; T.g_z]) - 0.3, max([T.g_total; T.g_z]) + 0.3];
axp.YLim = [0 105];
axq.YLim = [min(min(T.copv_bar), 250) - 10, max(T.copv_bar) + 10];
sp = max(5, 1.15*max(abs(dcg(:)))); axc.YLim = [-sp sp];
legend(axg, [lg{:}], {'x fwd', 'y left', 'z up', '|g|'}, 'TextColor', FG, 'Color', PANEL, 'EdgeColor', PANEL, ...
       'FontSize', 7, 'NumColumns', 4, 'Location', 'southwest');
legend(axp, [lp{:}], {'MMH', 'NTO'}, 'TextColor', FG, 'Color', PANEL, 'EdgeColor', PANEL, 'FontSize', 7, ...
       'Location', 'southwest');
legend(axc, [lc{:}], {'x', 'y', 'z'}, 'TextColor', FG, 'Color', PANEL, 'EdgeColor', PANEL, 'FontSize', 7, ...
       'NumColumns', 3, 'Location', 'southwest');
cur = [xline(axg, 0, 'Color', [1 .8 .5], 'HandleVisibility', 'off'), ...
       xline(axp, 0, 'Color', [1 .8 .5], 'HandleVisibility', 'off'), ...
       xline(axq, 0, 'Color', [1 .8 .5], 'HandleVisibility', 'off'), ...
       xline(axc, 0, 'Color', [1 .8 .5], 'HandleVisibility', 'off')];
annotation(fig, 'textbox', [0.035 0.262 0.6 0.02], 'String', ...
           'Porsche Mission S  |  SuperDraco / Draco thruster flight  |  simulated', ...
           'Color', DIM, 'EdgeColor', 'none', 'FontSize', 9);

if opts.WriteVideo
    vw = VideoWriter(mp4File, 'MPEG-4');
    vw.FrameRate = opts.FPS; vw.Quality = 95;
    open(vw);
    writerCleanup = onCleanup(@() close(vw));
end
for i = range(1):range(2)
    if haveFrames
        fp = fullfile(framesDir, sprintf('frame_%04d.png', i));
        im = imread(fp);
        if size(im, 3) == 4, im = im(:, :, 1:3); end
        img.CData = im; axv.XLim = [0.5 size(im, 2) + 0.5]; axv.YLim = [0.5 size(im, 1) + 0.5];
        sx = size(im, 2)/1344; sy = size(im, 1)/756;
        hud.Position = [20*sx 30*sy 0]; mlab.Position = [1324*sx 30*sy 0];
    end
    addpoints(lg{1}, t(i), T.g_x(i)); addpoints(lg{2}, t(i), T.g_y(i));
    addpoints(lg{3}, t(i), T.g_z(i)); addpoints(lg{4}, t(i), T.g_total(i));
    addpoints(lp{1}, t(i), T.fuel_pct(i)); addpoints(lp{2}, t(i), T.ox_pct(i));
    addpoints(lq{1}, t(i), T.copv_bar(i));
    for k = 1:3, addpoints(lc{k}, t(i), dcg(i, k)); end
    for c = cur, c.Value = t(i); end
    for k = 1:ng
        v = throttle(i, k);
        bh(k).YData = v;                    % barh: XData is the bar position, YData its length
        if v >= 0.5, vt(k).String = sprintf('%.0f%%', v); else, vt(k).String = ''; end
        vt(k).Position = [v + 1.5, yb(k), 0];
    end
    ph = char(T.phase(i));
    if isKey(modes, ph), mlab.String = modes(ph); else, mlab.String = upper(ph); end
    hud.String = sprintf('T+%5.2f s   %5.1f km/h   CG %5.2f m\nroll %6.1f%s   pitch %6.1f%s   %4.2f g', ...
        t(i), 3.6*T.speed(i), T.z(i), rad2deg(T.roll(i)), char(176), rad2deg(T.pitch(i)), char(176), T.g_total(i));
    if T.gear(i) > 0, gs = sprintf('%d', T.gear(i)); else, gs = '-'; end
    readout.String = sprintf(['mass     %7.1f kg\nengine   %7.0f rpm  gear %s\n' ...
        'MMH/NTO  %5.1f / %5.1f %%\nCOPV     %6.1f bar   tank %4.1f bar'], ...
        T.mass_kg(i), T.rpm(i), gs, T.fuel_pct(i), T.ox_pct(i), T.copv_bar(i), T.tank_bar(i));
    drawnow;
    im = print(fig, '-RGBImage', '-r100');
    assert(isequal(size(im), [1080 1920 3]), 'Telemetry frame dimensions must be 1920x1080 RGB');
    if ~opts.WriteVideo || i == range(1)
        pngFile = fullfile(opts.PNGDir, sprintf('tele_%04d.png', i));
        imwrite(im, pngFile);
    end
    if opts.WriteVideo, writeVideo(vw, im); end
    if mod(i, 50) == 0, fprintf('frame %d/%d\n', i, n); end
end
if opts.WriteVideo
    clear writerCleanup;
    fprintf('wrote %s\n', mp4File);
else
    mp4File = '';
    fprintf('wrote %s\n', pngFile);
end
clear figCleanup;
end

function [groups, labels, values, kinds, note] = telemetryGroups(T, sceneFile, layoutFile)
layout = [];
if exist(sceneFile, 'file')
    scene = jsondecode(fileread(sceneFile));
    if isfield(scene, 'thruster_layout'), layout = scene.thruster_layout; end
end
vars = T.Properties.VariableNames;
if isempty(layout) && exist(layoutFile, 'file')
    candidate = jsondecode(fileread(layoutFile));
    names = {candidate.channels.name};
    if all(ismember(strcat('thr_', names), vars)), layout = candidate; end
end
if isempty(layout)
    names = vars(startsWith(vars, 'thr_'));
    channels = struct('name', cellfun(@(v) v(5:end), names, 'UniformOutput', false), 'kind', 'legacy');
    for k = 1:numel(channels)
        if startsWith(channels(k).name, 'SD'), channels(k).kind = 'superdraco'; end
    end
else
    channels = layout.channels;
end
assert(~isempty(channels), 'CSV contains no thruster channels');
assert(all(ismember(strcat('thr_', {channels.name}), vars)), 'Scene and CSV thruster channels disagree');
groups = {}; labels = {}; kinds = {}; values = zeros(height(T), 0); aggregateCounts = [];
for k = 1:numel(channels)
    c = channels(k); name = c.name; members = {name}; label = strrep(name, '_', ' '); key = name;
    if strcmp(c.kind, 'side') && (startsWith(name, 'S_L') || startsWith(name, 'S_R'))
        key = name(1:3);
        if ismember(key, groups), continue; end
        members = {channels(strcmp({channels.kind}, 'side') & startsWith({channels.name}, key)).name};
        aggregateCounts(end+1) = numel(members); %#ok<AGROW>
        if strcmp(key, 'S_L'), label = 'Sill left (mean)'; else, label = 'Sill right (mean)'; end
    end
    groups{end+1} = key; labels{end+1} = label; kinds{end+1} = c.kind; %#ok<AGROW>
    values(:, end+1) = mean(T{:, strcat('thr_', members)}, 2); %#ok<AGROW>
end
notes = {'Delivered thrust, after actuator lag'};
if isfield(channels, 'max_thrust_N')
    uniqueKinds = unique({channels.kind}, 'stable');
    for k = 1:numel(uniqueKinds)
        forces = unique([channels(strcmp({channels.kind}, uniqueKinds{k})).max_thrust_N])/1000;
        notes{end+1} = sprintf('%s: %s kN/channel', uniqueKinds{k}, strjoin(compose('%g', forces), '/')); %#ok<AGROW>
    end
else
    notes{end+1} = 'Legacy CSV channels; maximum forces unspecified';
end
if numel(channels) > numel(groups)
    notes{end+1} = sprintf('Sill bars: mean channel %% (%s per side)\nAll %d raw channels retained in CSV', ...
        strjoin(compose('%d', aggregateCounts), '/'), numel(channels));
end
note = strjoin(notes, newline);
end
