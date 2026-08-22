function results = run_throttle_char(varargin)
%RUN_THROTTLE_CHAR  单轴涡喷稳态模型：扫燃油，画地面节流特性
%
%  使用前（在 MATLAB 命令行）：
%    1. 已把 T-MATS 库加到路径（和平时跑示例一样）
%    2. 本文件夹下先运行:  GasTurbine_SS_setup_everything
%    3. 确认 Compressor / Turbine / Nozzle 的 iDesign = 2
%    4. 再运行:  results = run_throttle_char;
%
%  扫描顺序（几何与 iDesign 始终冻结）：
%    1) 先跑通设计点 3.00 pps
%    2) 用 3.00 的收敛解单独往上探 3.05 pps
%    3) 回到 3.00 的收敛解，按 0.1 pps 减到 1.80 pps
%    4) 若 1.80 仍收敛，再按 0.05 pps 往下，直到不收敛
%    任一步不收敛：在上一点与失败点之间对分；步长小于 dWfMin 则停止下行
%
%  每一档用上一档收敛的 [W; Rline; 涡轮PR; N] 作为 NR 初值。
%  退出时恢复燃油和 NR_IC，并删除本次临时加上的记录模块；不要保存官方 mdl。

    p = inputParser;
    addParameter(p, 'Model', 'GasTurbine_SS_Template', @ischar);
    addParameter(p, 'WfDes', 3.00, @isnumeric);
    addParameter(p, 'WfUp', 3.05, @isnumeric);
    addParameter(p, 'WfCoarseEnd', 1.80, @isnumeric);
    addParameter(p, 'dWfCoarse', 0.10, @isnumeric);
    addParameter(p, 'dWfFine', 0.05, @isnumeric);
    addParameter(p, 'dWfMin', 0.01, @isnumeric);
    addParameter(p, 'WfMinAbs', 0.50, @isnumeric);
    addParameter(p, 'SaveFig', true, @islogical);
    parse(p, varargin{:});

    mdl = p.Results.Model;
    WfDes = p.Results.WfDes;
    WfUp = p.Results.WfUp;
    WfCoarseEnd = p.Results.WfCoarseEnd;
    dWfCoarse = p.Results.dWfCoarse;
    dWfFine = p.Results.dWfFine;
    dWfMin = p.Results.dWfMin;
    WfMinAbs = p.Results.WfMinAbs;

    if evalin('base', 'exist(''MWS'',''var'')') ~= 1
        error(['工作区没有 MWS。请先在本目录运行 GasTurbine_SS_setup_everything，', ...
               '再调用 run_throttle_char。']);
    end
    MWS = evalin('base', 'MWS');
    NR_IC0 = MWS.Solve.NR_IC(:);

    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end

    assert_idesign_frozen(mdl);
    assert_sls_condition(mdl);

    wfBlk     = [mdl '/Fuel Flow Input [pps]'];
    solverBlk = [mdl '/SS NR Solver w JacobianCalc'];
    Wf0_str   = get_param(wfBlk, 'Value');
    added_blocks = ensure_throttle_logs(mdl); %#ok<NASGU>
    cleanupObj = onCleanup(@() cleanup_all()); %#ok<NASGU>

    results = empty_results();
    x_next = NR_IC0;

    fprintf('\n=== 地面节流扫描 (H=0, MN=0, iDesign=2) ===\n');
    fprintf('%6s  %8s  %8s  %10s  %8s  %s\n', ...
        'Wf', 'N', 'Fn', 'SFC', 'SM', 'status');

    % --- 1) 设计点 ---
    fprintf('-- 1) 设计点 %.2f pps --\n', WfDes);
    [results, x_des, okDes] = run_and_store(results, WfDes, x_next);
    if ~okDes
        error('设计点 %.2f pps 未收敛，停止扫描。', WfDes);
    end

    % --- 2) 单独往上探，探完仍用设计点解往下走 ---
    fprintf('-- 2) 往上探 %.2f pps（初值来自 %.2f）--\n', WfUp, WfDes);
    [results, ~, okUp] = run_and_store(results, WfUp, x_des);
    if ~okUp
        fprintf('    往上探未收敛，设计点已靠近高端。\n');
    end
    x_next = x_des;
    Wf_ok = WfDes;

    % --- 3) 从设计点按 0.1 pps 走向 1.80；失败则对分 ---
    fprintf('-- 3) 从 %.2f 按 %.2f pps 走向 %.2f，失败则对分 --\n', ...
        WfDes, dWfCoarse, WfCoarseEnd);
    [results, x_next, Wf_ok, reached_1p80] = sweep_down( ...
        results, Wf_ok, x_next, dWfCoarse, WfCoarseEnd, false, dWfMin);

    % --- 4) 到达 1.80 后再按 0.05 pps 往下直到对分也无法前进 ---
    if reached_1p80
        fprintf('-- 4) 已到达 %.2f pps，改 %.2f pps 继续往下 --\n', ...
            WfCoarseEnd, dWfFine);
        [results, ~, Wf_ok] = sweep_down( ...
            results, Wf_ok, x_next, dWfFine, WfMinAbs, true, dWfMin);
        fprintf('    下行结束，最后成功点 Wf = %.2f pps。\n', Wf_ok);
    else
        fprintf('-- 4) 未能收敛到 %.2f pps，最后成功点 Wf = %.2f，不再进入 0.05 段。\n', ...
            WfCoarseEnd, Wf_ok);
    end

    outdir = fileparts(mfilename('fullpath'));
    save(fullfile(outdir, 'throttle_char_results.mat'), 'results');
    plot_throttle(results);
    if p.Results.SaveFig
        figfile = fullfile(outdir, 'throttle_char.png');
        saveas(gcf, figfile);
        fprintf('图已保存: %s\n', figfile);
    end

    function [res, x_out, Wf_out, reached] = sweep_down(res, Wf_ok1, x_ok, dNom, Wf_bound, until_fail, dMin)
        x_out = x_ok;
        Wf_out = Wf_ok1;
        reached = (~until_fail) && (abs(Wf_ok1 - Wf_bound) < 1e-6);
        d = dNom;
        nGuard = 0;
        while nGuard < 200 && Wf_out > WfMinAbs + 1e-9
            nGuard = nGuard + 1;
            if ~until_fail && (Wf_out <= Wf_bound + 1e-6)
                reached = true;
                break
            end

            Wf_try = round(Wf_out - d, 4);
            if until_fail
                if Wf_try < WfMinAbs - 1e-9
                    fprintf('    已到脚本保护下限 %.2f pps。\n', WfMinAbs);
                    break
                end
            else
                if Wf_try < Wf_bound - 1e-9
                    Wf_try = Wf_bound;
                end
            end

            [res, x_try, ok] = run_and_store(res, Wf_try, x_out);
            if ok
                Wf_out = Wf_try;
                x_out = x_try;
                d = dNom;
                if ~until_fail && abs(Wf_out - Wf_bound) < 1e-6
                    reached = true;
                    break
                end
                continue
            end

            Wf_mid = round(0.5 * (Wf_out + Wf_try), 4);
            step = Wf_out - Wf_mid;
            if step < dMin - 1e-12
                fprintf(['    %.2f 未收敛，与上一成功点 %.2f 的间隔已小于 %.3f pps，', ...
                    '停止下行。\n'], Wf_try, Wf_out, dMin);
                break
            end
            d = step;
            fprintf('    %.2f 未收敛，步长减半，下一档试 %.2f pps\n', Wf_try, Wf_out - d);
        end
    end

    function [res, x_out, ok] = run_and_store(res, Wf, x_ic)
        [row, x_out, ok] = run_one_point(Wf, x_ic);
        res = append_result(res, row);
        if ok
            flag = 'OK';
        else
            flag = 'NOT CONVERGED';
        end
        fprintf('%6.2f  %8.1f  %8.1f  %10.4f  %8.2f  %s\n', ...
            row.Wf_pps, row.N_rpm, row.Fn_lbf, row.SFC_pph_lbf, row.SM_pct, flag);
    end

    function [row, x_out, ok] = run_one_point(Wf, x_ic)
        row = blank_row(Wf);
        x_out = x_ic(:);
        ok = false;

        MWS.Solve.NR_IC = x_ic(:);
        assignin('base', 'MWS', MWS);
        set_param(wfBlk, 'Value', sprintf('%.6g', Wf));
        set_param(solverBlk, 'SNR_IC_M', 'MWS.Solve.NR_IC');

        try
            simOut = sim(mdl, 'ReturnWorkspaceOutputs', 'on', ...
                'SrcWorkspace', 'base');
        catch ME
            fprintf('%6.2f  -- 仿真出错: %s\n', Wf, ME.message);
            return
        end

        try
            NR_X = get_sim_var(simOut, 'NR_X');
            Fg   = get_sim_var(simOut, 'Fg_log');
            Cdat = get_sim_var(simOut, 'C_Data');
            Sdat = get_sim_var(simOut, 'S_Data_log');
            s4   = get_sim_var(simOut, 's4');

            x = ts_last_vec(NR_X, 4);
            Fg_end = ts_last_scalar(Fg);
            ok = last_flag(Sdat, 'Converged');

            row.NR_X     = x.';
            row.W_pps    = x(1);
            row.Rline    = x(2);
            row.PR_turb  = x(3);
            row.N_rpm    = x(4);
            row.Fn_lbf   = Fg_end;
            row.Fn_N     = Fg_end * 4.4482216153;
            if Fg_end ~= 0 && isfinite(Fg_end)
                row.SFC_pph_lbf = 3600 * Wf / Fg_end;
                row.SFC_kgN_s   = (Wf * 0.45359237) / row.Fn_N;
            end
            row.converged = ok;
            row.PR_comp  = last_num(Cdat, 'PR');
            row.SM_pct   = last_num(Cdat, 'SMavail');
            row.Nc       = last_num(Cdat, 'Nc');
            row.Tt4_R    = last_num(s4, 'Tt');
            if ok
                x_out = x(:);
            end
        catch ME
            fprintf('%6.2f  -- 读结果失败: %s\n', Wf, ME.message);
            dump_obj('S_Data_log', get_sim_var_soft(simOut, 'S_Data_log'));
            dump_obj('C_Data', get_sim_var_soft(simOut, 'C_Data'));
            ok = false;
        end
    end

    function cleanup_all()
        try
            MWS.Solve.NR_IC = NR_IC0;
            assignin('base', 'MWS', MWS);
            if bdIsLoaded(mdl)
                set_param(wfBlk, 'Value', Wf0_str);
                set_param(solverBlk, 'SNR_IC_M', 'MWS.Solve.NR_IC');
                remove_throttle_logs(mdl);
            end
            fprintf(['已恢复燃油与 NR_IC，并删除临时记录模块。', ...
                '请不要保存 GasTurbine_SS_Template.mdl。\n']);
        catch ME
            warning('THROTTLE:Cleanup', '退出清理未完全成功: %s', ME.message);
        end
    end
end

function r = empty_results()
    r = struct('Wf_pps', [], 'Wf_kgs', [], 'N_rpm', [], 'Fn_lbf', [], ...
        'Fn_N', [], 'SFC_pph_lbf', [], 'SFC_kgN_s', [], 'W_pps', [], ...
        'Rline', [], 'PR_turb', [], 'PR_comp', [], 'SM_pct', [], ...
        'Nc', [], 'Tt4_R', [], 'converged', false(0, 1), 'NR_X', zeros(0, 4));
end

function row = blank_row(Wf)
    row.Wf_pps = Wf;
    row.Wf_kgs = Wf * 0.45359237;
    row.N_rpm = nan;
    row.Fn_lbf = nan;
    row.Fn_N = nan;
    row.SFC_pph_lbf = nan;
    row.SFC_kgN_s = nan;
    row.W_pps = nan;
    row.Rline = nan;
    row.PR_turb = nan;
    row.PR_comp = nan;
    row.SM_pct = nan;
    row.Nc = nan;
    row.Tt4_R = nan;
    row.converged = false;
    row.NR_X = nan(1, 4);
end

function r = append_result(r, row)
    r.Wf_pps(end+1, 1) = row.Wf_pps;
    r.Wf_kgs(end+1, 1) = row.Wf_kgs;
    r.N_rpm(end+1, 1) = row.N_rpm;
    r.Fn_lbf(end+1, 1) = row.Fn_lbf;
    r.Fn_N(end+1, 1) = row.Fn_N;
    r.SFC_pph_lbf(end+1, 1) = row.SFC_pph_lbf;
    r.SFC_kgN_s(end+1, 1) = row.SFC_kgN_s;
    r.W_pps(end+1, 1) = row.W_pps;
    r.Rline(end+1, 1) = row.Rline;
    r.PR_turb(end+1, 1) = row.PR_turb;
    r.PR_comp(end+1, 1) = row.PR_comp;
    r.SM_pct(end+1, 1) = row.SM_pct;
    r.Nc(end+1, 1) = row.Nc;
    r.Tt4_R(end+1, 1) = row.Tt4_R;
    r.converged(end+1, 1) = row.converged;
    r.NR_X(end+1, :) = row.NR_X;
end

function assert_idesign_frozen(mdl)
    blocks = {'Compressor', 'Turbine', 'Nozzle'};
    for i = 1:numel(blocks)
        v = str2double(get_param([mdl '/' blocks{i}], 'iDesign_M'));
        if v ~= 2
            error('%s 的 iDesign_M = %g，节流扫描必须为 2（几何冻结）。', ...
                blocks{i}, v);
        end
    end
end

function assert_sls_condition(mdl)
    alt = str2double(get_param([mdl '/Alt [ft]'], 'Value'));
    mn  = str2double(get_param([mdl '/Mach Number [frac]'], 'Value'));
    dT  = str2double(get_param([mdl '/dTamb [degF]'], 'Value'));
    if any(isnan([alt, mn, dT]))
        error('无法读取 Alt / Mach / dTamb，请确认模型里这三个 Constant 还在。');
    end
    if abs(alt) > 1e-6 || abs(mn) > 1e-6 || abs(dT) > 1e-6
        error(['地面节流要求 Alt=0、MN=0、dTamb=0。当前为 Alt=%g, MN=%g, dTamb=%g。'], ...
            alt, mn, dT);
    end
end

function added = ensure_throttle_logs(mdl)
    logs = {
        'Log_NR_X',    'NR_X',       [1180 930 1260 960]
        'Log_Fg',      'Fg_log',     [2140 250 2220 280]
        'Log_S_Data',  'S_Data_log', [1180 980 1260 1010]
        };
    added = {};
    for i = 1:size(logs, 1)
        bname = logs{i, 1};
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', bname))
            add_block('simulink/Sinks/To Workspace', [mdl '/' bname], ...
                'VariableName', logs{i, 2}, ...
                'MaxDataPoints', 'inf', ...
                'SaveFormat', 'Timeseries', ...
                'SampleTime', '-1', ...
                'Position', sprintf('[%d %d %d %d]', logs{i, 3}));
            added{end+1} = bname; %#ok<AGROW>
        end
    end
    try_add_line(mdl, 'SS NR Solver w JacobianCalc/1', 'Log_NR_X/1');
    try_add_line(mdl, 'Nozzle/2', 'Log_Fg/1');
    try_add_line(mdl, 'SS NR Solver w JacobianCalc/2', 'Log_S_Data/1');
end

function remove_throttle_logs(mdl)
    names = {'Log_NR_X', 'Log_Fg', 'Log_S_Data'};
    for i = 1:numel(names)
        if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', names{i}))
            continue
        end
        b = [mdl '/' names{i}];
        lh = get_param(b, 'LineHandles');
        if isfield(lh, 'Inport') && ~isempty(lh.Inport) && lh.Inport(1) > 0
            delete_line(lh.Inport(1));
        end
        delete_block(b);
    end
end

function try_add_line(mdl, src, dst)
    try
        add_line(mdl, src, dst, 'autorouting', 'on');
    catch
    end
end

function v = get_sim_var(simOut, name)
    v = get_sim_var_soft(simOut, name);
    if isempty(v) && ~(isnumeric(v) && isequal(v, []))
        % 空 timeseries 也算找到了；这里只拦“完全没有”
    end
    if is_missing_var(v)
        error('仿真后找不到变量 %s。请确认记录模块已加上。', name);
    end
end

function v = get_sim_var_soft(simOut, name)
    v = [];
    if ~isempty(simOut)
        try
            if has_element(simOut, name)
                v = simOut.get(name);
                return
            end
        catch
        end
        try
            if isa(simOut, 'Simulink.SimulationOutput')
                logs = simOut.who;
                if any(strcmp(logs, name))
                    v = simOut.get(name);
                    return
                end
            end
        catch
        end
    end
    if evalin('base', sprintf('exist(''%s'',''var'')', name))
        v = evalin('base', name);
    end
end

function tf = is_missing_var(v)
    tf = isempty(v) && ~isa(v, 'timeseries') && ~isstruct(v) ...
        && ~isa(v, 'Simulink.SimulationData.Signal') ...
        && ~isa(v, 'Simulink.SimulationData.Dataset');
end

function tf = has_element(simOut, name)
    tf = false;
    try
        nms = simOut.who;
        tf = any(strcmp(nms, name));
    catch
        try
            simOut.get(name);
            tf = true;
        catch
            tf = false;
        end
    end
end

function tf = last_flag(obj, fieldName)
    y = bus_field(obj, fieldName);
    s = ts_last_scalar(y);
    tf = s > 0.5;
end

function s = last_num(obj, fieldName)
    y = bus_field(obj, fieldName);
    s = ts_last_scalar(y);
end

function y = bus_field(busTs, fieldName)
    busTs = unwrap_signal(busTs);
    fnWant = fieldName;

    if isstruct(busTs) && isscalar(busTs)
        fn = fieldnames(busTs);
        hit = find(strcmp(fn, fnWant) | strcmpi(fn, fnWant), 1);
        if ~isempty(hit)
            y = unwrap_signal(busTs.(fn{hit}));
            return
        end
    end

    if isa(busTs, 'timeseries')
        d = busTs.Data;
        if isstruct(d)
            fn = fieldnames(d);
            hit = find(strcmp(fn, fnWant) | strcmpi(fn, fnWant), 1);
            if ~isempty(hit)
                col = {d.(fn{hit})};
                num = cellfun(@(c) double(c(end)), col);
                y = timeseries(num(:), busTs.Time);
                return
            end
        end
    end

    if isa(busTs, 'Simulink.SimulationData.Dataset')
        try
            el = busTs.getElement(fnWant);
            y = unwrap_signal(el.Values);
            return
        catch
        end
    end

    % 顶层已经是该物理量（例如单独的 timeseries）且调用者传了 Tt
    if isa(busTs, 'timeseries') && any(strcmpi({'Tt', 'W', 'Pt', 'ht', 'FAR'}, fnWant))
        y = busTs;
        return
    end

    extra = describe_obj(busTs);
    error('无法从记录数据中读取字段 %s。对象信息: %s', fnWant, extra);
end

function obj = unwrap_signal(obj)
    if isa(obj, 'Simulink.SimulationData.Signal')
        obj = obj.Values;
    end
end

function extra = describe_obj(obj)
    extra = class(obj);
    try
        if isstruct(obj)
            extra = [extra ' fields=' strjoin(fieldnames(obj), ',')];
        elseif isa(obj, 'timeseries') && isstruct(obj.Data)
            extra = [extra ' DataFields=' strjoin(fieldnames(obj.Data), ',')];
        elseif isa(obj, 'Simulink.SimulationData.Dataset')
            extra = [extra ' elements=' strjoin(obj.getElementNames, ',')];
        end
    catch
    end
end

function dump_obj(name, obj)
    fprintf('    [%s] %s\n', name, describe_obj(obj));
end

function x = ts_last_vec(ts, n)
    d = ts_data(ts);
    d = squeeze(d);
    if isempty(d)
        x = nan(n, 1);
        return
    end
    if isvector(d)
        x = d(:);
        if numel(x) > n
            x = x(end-n+1:end);
        elseif numel(x) < n
            x(end+1:n, 1) = nan;
        end
    elseif size(d, 1) == n
        x = d(:, end);
    elseif size(d, 2) == n
        x = d(end, :).';
    else
        x = d(end, 1:min(n, size(d, 2))).';
        if numel(x) < n
            x(end+1:n, 1) = nan;
        end
    end
    x = double(x(:));
end

function s = ts_last_scalar(ts)
    d = ts_data(ts);
    d = squeeze(d);
    if isempty(d)
        s = nan;
        return
    end
    s = double(d(end));
end

function d = ts_data(ts)
    ts = unwrap_signal(ts);
    if isa(ts, 'timeseries')
        d = ts.Data;
        if islogical(d)
            d = double(d);
        end
    elseif isnumeric(ts) || islogical(ts)
        d = double(ts);
    elseif isstruct(ts) && isfield(ts, 'Data')
        d = ts_data(ts.Data);
    else
        error('不支持的记录类型: %s', class(ts));
    end
end

function plot_throttle(results)
    ok = results.converged;
    if ~any(ok)
        warning('THROTTLE:NoPoints', '没有收敛点，无法画节流特性。');
        return
    end
    N = results.N_rpm(ok);
    Fn = results.Fn_lbf(ok);
    sfc = results.SFC_pph_lbf(ok);
    [N, idx] = sort(N);
    Fn = Fn(idx);
    sfc = sfc(idx);

    figure('Name', 'Throttle characteristic', 'Color', 'w');

    subplot(1, 2, 1);
    plot(N, Fn, 'o-', 'LineWidth', 1.5);
    grid on
    xlabel('N  [rpm]');
    ylabel('F_n  [lbf]');
    title('地面节流特性（推力）');

    subplot(1, 2, 2);
    plot(N, sfc, 'o-', 'LineWidth', 1.5);
    grid on
    xlabel('N  [rpm]');
    ylabel('SFC  [lbm/h/lbf]');
    title('地面节流特性（耗油率）');
end
