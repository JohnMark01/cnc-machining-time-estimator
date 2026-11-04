% CNC Time Estimation Algorithm
% Batch reads Siemens .mpf experimental files, estimates cycle time,
% includes acceleration/deceleration for G00/G01 and compares to a
% compares predicted time against actual machine time and SolidWorks CAM time.

clear; clc;

%% User settings
cfg.Vrapid_mm_min   = 12000;   % rapid speed (mm/min)
cfg.aRapid_mm_s2    = 1200;    % rapid accel (mm/s^2)
cfg.aFeed_mm_s2     = 800;     % feed accel (mm/s^2)
cfg.Ttool_s         = 60;      % tool change time (s), adjusted upward to reflect actual machine delay
cfg.TspindleStart_s = 2;       % spindle start time (s)
cfg.TspindleStop_s  = 1;       % spindle stop time (s)
cfg.Tcoolant_s      = 0.5;     % coolant on/off event cost (s)
cfg.defaultFeed_mm_min = 600;  % fallback feed
cfg.referenceActual_s = 665;   % actual machined time: 11 min 05 s
cfg.referenceSW_s     = 625;   % SolidWorks CAM time: 10 min 25 s

zipFile = "m2-a.zip";
extractDir = "m2a_extracted";
unsupportedLog = "unsupported_codes_log.txt";
if isfile(unsupportedLog), delete(unsupportedLog); end

%% Get NC files (.mpf, .nc, .txt) from current folder or zip
zipFile = "m2-a.zip";
extractDir = "m2a_extracted";
unsupportedLog = "unsupported_codes_log.txt";
if isfile(unsupportedLog), delete(unsupportedLog); end

if isfile(zipFile)
    if ~isfolder(extractDir)
        unzip(zipFile, extractDir);
    end
end

searchFolders = {pwd};
if isfolder(extractDir)
    searchFolders{end+1} = extractDir;
end

files = [];
for f = 1:numel(searchFolders)
    files = [files; ...
        dir(fullfile(searchFolders{f}, "*.mpf")); ...
        dir(fullfile(searchFolders{f}, "*.nc")); ...
        dir(fullfile(searchFolders{f}, "*.txt"))]; %#ok<AGROW>
end

% remove duplicates by full path
if ~isempty(files)
    fullNames = arrayfun(@(x) string(fullfile(x.folder, x.name)), files, 'UniformOutput', true);
    [~, ia] = unique(fullNames, 'stable');
    files = files(ia);
end

if isempty(files)
    error("No .mpf, .nc, or .txt files found. Place the zip or extracted NC files in the working folder.");
end

%% Estimate all files
results = table('Size', [numel(files), 11], ...
    'VariableTypes', {'string','double','double','double','double','double','double','double','double','double','double'}, ...
    'VariableNames', {'File','Predicted_s','Predicted_min','ActualRef_s','SolidWorksRef_s','AbsErrorActual_s','PctErrorActual','AbsErrorSW_s','PctErrorSW','Linear_s','Arc_s'});

for k = 1:numel(files)
    filePath = fullfile(files(k).folder, files(k).name);
    out = estimate_file_time(filePath, cfg, unsupportedLog);

    results.File(k)            = string(files(k).name);
    results.Predicted_s(k)     = out.total_s;
    results.Predicted_min(k)   = out.total_s/60;
    results.ActualRef_s(k)     = cfg.referenceActual_s;
    results.SolidWorksRef_s(k) = cfg.referenceSW_s;
    results.AbsErrorActual_s(k)= abs(out.total_s - cfg.referenceActual_s);
    results.PctErrorActual(k)  = 100 * abs(out.total_s - cfg.referenceActual_s) / cfg.referenceActual_s;
    results.AbsErrorSW_s(k)    = abs(out.total_s - cfg.referenceSW_s);
    results.PctErrorSW(k)      = 100 * abs(out.total_s - cfg.referenceSW_s) / cfg.referenceSW_s;
    results.Linear_s(k)        = out.breakdown.linear_s;
    results.Arc_s(k)           = out.breakdown.arc_s;
end

disp(results)

mapeActual = mean(results.PctErrorActual);
rmseActual = sqrt(mean((results.Predicted_s - results.ActualRef_s).^2));
hit5Actual = mean(results.PctErrorActual <= 5) * 100;
hit10Actual = mean(results.PctErrorActual <= 10) * 100;

mapeSW = mean(results.PctErrorSW);
rmseSW = sqrt(mean((results.Predicted_s - results.SolidWorksRef_s).^2));
hit5SW = mean(results.PctErrorSW <= 5) * 100;
hit10SW = mean(results.PctErrorSW <= 10) * 100;

fprintf('\nSummary metrics versus actual machine time\n');
fprintf('MAPE  : %.2f %%\n', mapeActual);
fprintf('RMSE  : %.2f s\n', rmseActual);
fprintf('Hit<=5%%  : %.1f %%\n', hit5Actual);
fprintf('Hit<=10%% : %.1f %%\n', hit10Actual);

fprintf('\nSummary metrics versus SolidWorks CAM time\n');
fprintf('MAPE  : %.2f %%\n', mapeSW);
fprintf('RMSE  : %.2f s\n', rmseSW);
fprintf('Hit<=5%%  : %.1f %%\n', hit5SW);
fprintf('Hit<=10%% : %.1f %%\n', hit10SW);

%% ---------------- Local functions ----------------

function out = estimate_file_time(filePath, cfg, unsupportedLog)
    lines = readlines(filePath);

    % Modal / state
    st.pos = [0 0 0];
    st.feed_mm_min = cfg.defaultFeed_mm_min;
    st.absMode = true;      % G90 by default after setup
    st.motion = "";         % modal motion code
    st.plane = "XY";        % default plane
    st.spindleOn = false;
    st.coolantOn = false;

    % Accumulators
    t_linear = 0; t_rapid = 0; t_arc = 0;
    t_dwell = 0;  t_tool = 0;  t_spindle = 0; t_coolant = 0;
    nMoves = 0;

    for i = 1:numel(lines)
        raw = string(lines(i));
        L = strip_comments(raw);
        if strlength(L) == 0
            continue
        end

        % Remove leading line number N123
        L = regexprep(L, '^\s*N\d+\s*', '');
        if strlength(strtrim(L)) == 0
            continue
        end

        % Skip Siemens setup / non-timing lines safely
        if contains(L, "WORKPIECE") || contains(L, "FGROUP") || contains(L, "SUPA")
            continue
        end

        % Detect unsupported or currently unhandled codes and log them
        warn_unsupported_codes(L, filePath, unsupportedLog);

        % Modal updates
        if contains(L, "G90"), st.absMode = true;  end
        if contains(L, "G91"), st.absMode = false; end
        if contains(L, "G17"), st.plane = "XY"; end
        if contains(L, "G18"), st.plane = "XZ"; end
        if contains(L, "G19"), st.plane = "YZ"; end

        F = getWordValue(L, 'F');
        if ~isnan(F), st.feed_mm_min = F; end

        % Non-motion events may coexist with motion
        if hasCode(L, "M03") || hasCode(L, "M3") || hasCode(L, "M04") || hasCode(L, "M4")
            if ~st.spindleOn
                t_spindle = t_spindle + cfg.TspindleStart_s;
                st.spindleOn = true;
            end
        end

        if hasCode(L, "M05") || hasCode(L, "M5")
            if st.spindleOn
                t_spindle = t_spindle + cfg.TspindleStop_s;
                st.spindleOn = false;
            end
        end

        if hasCode(L, "M06") || hasCode(L, "M6")
            t_tool = t_tool + cfg.Ttool_s;
        end

        if hasCode(L, "M08") || hasCode(L, "M8") || hasCode(L, "M09") || hasCode(L, "M9")
            t_coolant = t_coolant + cfg.Tcoolant_s;
        end

        if hasCode(L, "G04") || hasCode(L, "G4")
            P = getWordValue(L, 'P');
            if ~isnan(P), t_dwell = t_dwell + P; end
        end

        % Motion detection (latest motion code on the line wins)
        motion = "";
        if hasCode(L, "G00") || hasCode(L, "G0"), motion = "G00"; end
        if hasCode(L, "G01") || hasCode(L, "G1"), motion = "G01"; end
        if hasCode(L, "G02") || hasCode(L, "G2"), motion = "G02"; end
        if hasCode(L, "G03") || hasCode(L, "G3"), motion = "G03"; end
        if strlength(motion) > 0
            st.motion = motion;
        end

        X = getWordValue(L, 'X');
        Y = getWordValue(L, 'Y');
        Z = getWordValue(L, 'Z');
        I = getWordValue(L, 'I');
        J = getWordValue(L, 'J');
        K = getWordValue(L, 'K');
        R = getWordValue(L, 'R');
        CR = getCRValue(L);

        target = st.pos;
        if ~isnan(X), target(1) = apply_mode(st.pos(1), X, st.absMode); end
        if ~isnan(Y), target(2) = apply_mode(st.pos(2), Y, st.absMode); end
        if ~isnan(Z), target(3) = apply_mode(st.pos(3), Z, st.absMode); end

        moved = any(abs(target - st.pos) > 1e-12);

        if moved
            switch st.motion
                case "G00"
                    dist = norm(target - st.pos);
                    t_rapid = t_rapid + time_move_trap(dist, cfg.Vrapid_mm_min/60, cfg.aRapid_mm_s2);

                case "G01"
                    dist = norm(target - st.pos);
                    t_linear = t_linear + time_move_trap(dist, st.feed_mm_min/60, cfg.aFeed_mm_s2);

                case {"G02","G03"}
                    arcLen = estimate_arc_length(st.pos, target, I, J, K, R, CR, st.motion, st.plane);
                    t_arc = t_arc + time_move_trap(arcLen, st.feed_mm_min/60, cfg.aFeed_mm_s2);

                otherwise
                    % No known motion mode, ignore move safely
            end

            st.pos = target;
            nMoves = nMoves + 1;
        end
    end

    out.total_s = t_linear + t_rapid + t_arc + t_dwell + t_tool + t_spindle + t_coolant;
    out.breakdown.linear_s  = t_linear;
    out.breakdown.rapid_s   = t_rapid;
    out.breakdown.arc_s     = t_arc;
    out.breakdown.dwell_s   = t_dwell;
    out.breakdown.tool_s    = t_tool;
    out.breakdown.spindle_s = t_spindle;
    out.breakdown.coolant_s = t_coolant;
    out.moves = nMoves;
end

function v = apply_mode(current, wordVal, absMode)
    if absMode
        v = wordVal;
    else
        v = current + wordVal;
    end
end

function tf = hasCode(line, code)
    tf = contains(upper(line), upper(code));
end

function t = time_move_trap(L_mm, v_mm_s, a_mm_s2)
    if L_mm <= 0 || v_mm_s <= 0 || a_mm_s2 <= 0
        t = 0;
        return
    end

    t_acc = v_mm_s / a_mm_s2;
    d_acc = 0.5 * a_mm_s2 * t_acc^2;

    if L_mm >= 2*d_acc
        d_cruise = L_mm - 2*d_acc;
        t = 2*t_acc + d_cruise / v_mm_s;
    else
        t_acc_tri = sqrt(L_mm / a_mm_s2);
        t = 2 * t_acc_tri;
    end
end

function L = estimate_arc_length(p0, p1, I, J, K, R, CR, motion, plane)
    % Prefer IJK if available
    hasIJK = ~(isnan(I) && isnan(J) && isnan(K));
    if hasIJK
        [ax1, ax2, off1, off2, hIdx] = choose_plane_from_IJK(I, J, K, plane);
        s = [p0(ax1), p0(ax2)];
        e = [p1(ax1), p1(ax2)];
        c = [p0(ax1) + off1, p0(ax2) + off2];
        vs = s - c; ve = e - c;
        r = mean([norm(vs), norm(ve)]);
        theta = atan2(vs(1)*ve(2) - vs(2)*ve(1), dot(vs,ve));
        if motion == "G02" && theta > 0, theta = theta - 2*pi; end
        if motion == "G03" && theta < 0, theta = theta + 2*pi; end
        arc2d = abs(theta) * r;
        helix = abs(p1(hIdx) - p0(hIdx));
        L = hypot(arc2d, helix);
        return
    end

    % Then CR=
    if ~isnan(CR)
        [ax1, ax2, hIdx] = choose_plane_from_motion(p0, p1, plane);
        s = [p0(ax1), p0(ax2)];
        e = [p1(ax1), p1(ax2)];
        chord = norm(e - s);
        Ruse = abs(CR);
        if Ruse < chord/2 || chord == 0
            L = norm(p1 - p0);
            return
        end
        theta = 2 * asin(min(1, chord / (2*Ruse)));
        if CR < 0
            theta = 2*pi - theta;   % long arc convention
        end
        arc2d = Ruse * abs(theta);
        helix = abs(p1(hIdx) - p0(hIdx));
        L = hypot(arc2d, helix);
        return
    end

    % Finally R
    if ~isnan(R)
        [ax1, ax2, hIdx] = choose_plane_from_motion(p0, p1, plane);
        s = [p0(ax1), p0(ax2)];
        e = [p1(ax1), p1(ax2)];
        chord = norm(e - s);
        Ruse = abs(R);
        if Ruse < chord/2 || chord == 0
            L = norm(p1 - p0);
            return
        end
        theta = 2 * asin(min(1, chord / (2*Ruse)));
        if R < 0
            theta = 2*pi - theta;
        end
        arc2d = Ruse * abs(theta);
        helix = abs(p1(hIdx) - p0(hIdx));
        L = hypot(arc2d, helix);
        return
    end

    % Fallback
    L = norm(p1 - p0);
end

function [ax1, ax2, off1, off2, hIdx] = choose_plane_from_IJK(I, J, K, plane)
    if ~isnan(I) && ~isnan(J)
        ax1 = 1; ax2 = 2; off1 = I; off2 = J; hIdx = 3; return
    elseif ~isnan(I) && ~isnan(K)
        ax1 = 1; ax2 = 3; off1 = I; off2 = K; hIdx = 2; return
    elseif ~isnan(J) && ~isnan(K)
        ax1 = 2; ax2 = 3; off1 = J; off2 = K; hIdx = 1; return
    end
    % fallback to modal plane
    switch upper(char(plane))
        case 'XY', ax1=1; ax2=2; off1=0; off2=0; hIdx=3;
        case 'XZ', ax1=1; ax2=3; off1=0; off2=0; hIdx=2;
        otherwise, ax1=2; ax2=3; off1=0; off2=0; hIdx=1;
    end
end

function [ax1, ax2, hIdx] = choose_plane_from_motion(p0, p1, plane)
    changed = find(abs(p1 - p0) > 1e-12);
    if numel(changed) >= 2
        ax1 = changed(1);
        ax2 = changed(2);
        hIdx = setdiff([1 2 3], [ax1 ax2]);
        if isempty(hIdx), hIdx = 3; end
    else
        switch upper(char(plane))
            case 'XY', ax1=1; ax2=2; hIdx=3;
            case 'XZ', ax1=1; ax2=3; hIdx=2;
            otherwise, ax1=2; ax2=3; hIdx=1;
        end
    end
end

function out = strip_comments(in)
    out = upper(string(in));
    sc = strfind(out, ';');
    if ~isempty(sc)
        out = extractBefore(out, sc(1));
    end
    out = regexprep(out, '\(.*?\)', ' ');
    out = strtrim(out);
end

function val = getWordValue(line, letter)
    expr = [letter '\s*=?\s*([-+]?\d*\.?\d+)'];
    tok = regexp(char(line), expr, 'tokens', 'once');
    if isempty(tok)
        val = NaN;
    else
        val = str2double(tok{1});
    end
end

function val = getCRValue(line)
    tok = regexp(char(line), 'CR\s*=\s*([-+]?\d*\.?\d+)', 'tokens', 'once');
    if isempty(tok)
        val = NaN;
    else
        val = str2double(tok{1});
    end
end


function warn_unsupported_codes(L, filePath, unsupportedLog)
    % Flags codes that are detected but not yet modelled in this estimator.
    % This does not stop execution. It only logs for review.

    % Explicit unsupported groups for this project stage
    unsupportedPrefixes = {'G81','G82','G83','G84','G85','G86','G87','G88','G89', ...
                           'G41','G42','G40', ...
                           'M98','M99', ...
                           'CYCLE', 'MCALL', ...
                           'TRANS', 'ROT', 'MIRROR', ...
                           'TRAORI', 'CYCLE800'};

    % Detect all G and M words
    gmTokens = regexp(char(L), '\b([GM]\d+)\b', 'match');
    namedTokens = regexp(char(L), '\b([A-Z]+)\b', 'match');

    hits = strings(0);

    for i = 1:numel(gmTokens)
        tok = string(gmTokens{i});
        if any(strcmp(tok, unsupportedPrefixes))
            hits(end+1) = tok; %#ok<AGROW>
        end
    end

    for i = 1:numel(namedTokens)
        tok = string(namedTokens{i});
        if any(strcmp(tok, unsupportedPrefixes))
            hits(end+1) = tok; %#ok<AGROW>
        end
    end

    if ~isempty(hits)
        fid = fopen(unsupportedLog, 'a');
        if fid ~= -1
            fprintf(fid, 'File: %s\n', filePath);
            fprintf(fid, 'Line: %s\n', char(L));
            fprintf(fid, 'Unsupported/Unhandled: %s\n\n', char(strjoin(unique(hits), ', ')));
            fclose(fid);
        end
    end
end
