function results = run_speed_char(varargin)
%RUN_SPEED_CHAR  单轴涡喷稳态模型：固定高度与油门，扫马赫数，画速度特性
%
%  使用前（在 MATLAB 命令行）：
%    1. 已把 T-MATS 库加到路径（和平时跑示例一样）
%    2. 本文件夹下先运行:  GasTurbine_SS_setup_everything
%    3. 确认 Compressor / Turbine / Nozzle 的 iDesign = 2
%    4. 再运行:  results = run_speed_char;
%
%  几何与 iDesign 始终冻结（非设计点）。dTamb 保持 0（标准大气）。
%
%  教材里的速度特性：高度不变、油门（控制规律）不变、只改飞行马赫数。
%  默认控制规律：涡轮前总温 T4 保持为海平面设计点的值。
%  本模型油门是燃油常数、转速由牛顿法求出，所以每一马赫数微调 Wf 去钉住 T4。
%  仍可改 ThrottleMode：'T4'（默认）| 'NcMap' | 'N'。
%
%  默认 H = 0 km，Ma 从 0 扫到 1.2。高空速度特性用：
%    results = run_speed_char('HKm', 11);
%  若 HKm > 0：先在 Ma = 0 爬升到该高度并配平油门（不记入曲线），再扫 Ma。
%
%  按 0.10 往上扫马赫数；失败则对分，步长小于 dMNMin 则停止。
%  图：单位推力 Fs=Fn/W、净推力 Fn、空气流量 W、耗油率 SFC=3600*Wf/Fn，均对马赫数。
%  飞行特性用净推力 Fn = Fg − Fram（教材定义）。毛推力 Fg 仍写入结果。
%  只重画：  results = run_speed_char('PlotOnly', true);
%  退出时恢复高度、马赫数、燃油和 NR_IC；不要保存官方 mdl。

    p = inputParser;
    addParameter(p, 'Model', 'GasTurbine_SS_Template', @ischar);
    addParameter(p, 'HKm', 0, @isnumeric);
    addParameter(p, 'dHKm', 1.0, @isnumeric);
    addParameter(p, 'MNMax', 1.20, @isnumeric);
    addParameter(p, 'dMN', 0.10, @isnumeric);
    addParameter(p, 'dMNMin', 0.05, @isnumeric);
    addParameter(p, 'WfDes', 3.00, @isnumeric);
    addParameter(p, 'ThrottleMode', 'T4', @ischar);
    addParameter(p, 'ThrottleTarget', nan, @isnumeric);
    addParameter(p, 'WfMin', 0.05, @isnumeric);
    addParameter(p, 'WfMax', 4.00, @isnumeric);
    addParameter(p, 'SaveFig', true, @islogical);
    addParameter(p, 'PlotOnly', false, @islogical);
    parse(p, varargin{:});

    outdir = fileparts(mfilename('fullpath'));
    matfile = fullfile(outdir, 'speed_char_results.mat');
    figMain = fullfile(outdir, 'speed_char.png');

    if p.Results.PlotOnly
        if exist(matfile, 'file') ~= 2
            error('找不到 %s。请先完整跑一次 run_speed_char。', matfile);
        end
        S = load(matfile, 'results');
        results = S.results;
        print_speed_summary(results);
        hFigs = plot_speed(results);
        if p.Results.SaveFig && ~isempty(hFigs)
            save_char_figure(hFigs(1), figMain);
            fprintf('已用现有结果重画。查看：  openfig(''%s'');\n', ...
                strrep(figMain, '.png', '.fig'));
        end
        return
    end

    mdl = p.Results.Model;
    HKm = p.Results.HKm;
    dHKm = p.Results.dHKm;
    MNMax = p.Results.MNMax;
    dMN = p.Results.dMN;
    dMNMin = p.Results.dMNMin;
    WfDes = p.Results.WfDes;
    mode = validatestring(p.Results.ThrottleMode, {'NcMap', 'N', 'T4'});
    target = p.Results.ThrottleTarget;
    WfMin = p.Results.WfMin;
    WfMax = p.Results.WfMax;

    if evalin('base', 'exist(''MWS'',''var'')') ~= 1
        error(['工作区没有 MWS。请先在本目录运行 GasTurbine_SS_setup_everything，', ...
               '再调用 run_speed_char。']);
    end
    MWS = evalin('base', 'MWS');
    NR_IC0 = MWS.Solve.NR_IC(:);
    NR_dx0 = MWS.Solve.NR_dx;
    HPC0 = MWS.HPC;
    SimTime0 = MWS.in.SimTime;
    MWS.HPC = pad_hpc_speed_lines(MWS.HPC, 1.12);
    MWS.in.SimTime = max(SimTime0, 1000 * MWS.Solve.T);
    assignin('base', 'MWS', MWS);

    if ~bdIsLoaded(mdl)
        load_system(mdl);
    end

    assert_idesign_frozen(mdl);
    dT = str2double(get_param([mdl '/dTamb [degF]'], 'Value'));
    if abs(dT) > 1e-6
        error('速度特性使用标准大气，要求 dTamb=0。当前为 %g。', dT);
    end

    wfBlk     = [mdl '/Fuel Flow Input [pps]'];
    solverBlk = [mdl '/SS NR Solver w JacobianCalc'];
    altBlk    = [mdl '/Alt [ft]'];
    mnBlk     = [mdl '/Mach Number [frac]'];
    Wf0_str   = get_param(wfBlk, 'Value');
    Alt0_str  = get_param(altBlk, 'Value');
    MN0_str   = get_param(mnBlk, 'Value');
    added_blocks = ensure_altitude_logs(mdl); %#ok<NASGU>
    cu = struct('mdl', mdl, 'wfBlk', wfBlk, 'solverBlk', solverBlk, ...
        'altBlk', altBlk, 'mnBlk', mnBlk, 'NR_IC0', NR_IC0, ...
        'NR_dx0', NR_dx0, 'HPC0', HPC0, 'SimTime0', SimTime0, ...
        'Wf0_str', Wf0_str, 'Alt0_str', Alt0_str, 'MN0_str', MN0_str);
    cleanupObj = onCleanup(@() restore_altitude_cleanup(cu)); %#ok<NASGU>

    results = empty_alt_results();
    results.meta.HKm = HKm;
    results.meta.MNMax = MNMax;
    results.meta.ThrottleMode = mode;
    warnedMap = false;

    fprintf('\n=== 速度特性扫描 (H=%.2f km, 控制规律=%s, iDesign=2) ===\n', ...
        HKm, mode);
    fprintf('%7s  %7s  %8s  %8s  %8s  %8s  %8s  %6s  %s\n', ...
        'Ma', 'Wf', 'W', 'NcMap', 'Fn', 'Fs', 'SFC', 'SM', 'status');

    fprintf('-- 0) 海平面静止锚点 Wf=%.2f，作为牛顿初值并取 T4 --\n', WfDes);
    [rowSLS, x_next, okSLS] = run_one_point(0, 0, WfDes, NR_IC0);
    if ~okSLS
        error('海平面设计点未收敛，无法开始速度扫描。');
    end
    if ~isfinite(target)
        switch mode
            case 'NcMap'
                target = 1.0;
            case 'N'
                target = rowSLS.N_rpm;
            case 'T4'
                target = rowSLS.Tt4_R;
        end
    end
    results.meta.ThrottleTarget = target;
    if strcmp(mode, 'T4')
        fprintf('    控制规律：T4 = %.1f K（设计点涡轮前总温）保持不变\n', target * 5/9);
    else
        fprintf('    油门目标 %s = %.4g\n', mode, target);
    end
    Wf_next = WfDes;

    if HKm > 1e-9
        hList = unique([dHKm:dHKm:HKm, HKm]);
        fprintf('-- 1) 在 Ma=0 爬升到 %.2f km（不记入速度特性）--\n', HKm);
        for i = 1:numel(hList)
            hTry = hList(i);
            ps0 = tmats_ps_psi(km2ft(0));
            if i > 1
                ps0 = tmats_ps_psi(km2ft(hList(i-1)));
            end
            ps1 = tmats_ps_psi(km2ft(hTry));
            Wf_g = Wf_next * max(ps1, 0.05) / max(ps0, 0.05);
            Wf_g = min(max(Wf_g, WfMin), WfMax);
            [rowH, x_try, Wf_try, okH] = trim_point( ...
                hTry, 0, Wf_g, x_next, false);
            if ~okH
                error(['Ma=0、H=%.2f km 未能配平油门，停止。', ...
                    '可减小 dHKm 或改用更低 HKm。'], hTry);
            end
            x_next = x_try;
            Wf_next = Wf_try;
            fprintf('    H=%.2f km  配平  Wf=%.3f  N=%.1f  NcMap=%.4f  T4=%.1f R\n', ...
                hTry, rowH.Wf_pps, rowH.N_rpm, rowH.NcMap, rowH.Tt4_R);
        end
    else
        fprintf('-- 1) H=0 km，跳过爬升，直接从海平面静止开始 --\n');
    end

    fprintf('-- 2) 在 H=%.2f km 上扫马赫数，步长 %.2f --\n', HKm, dMN);
    [~, x_next, Wf_next, ok0] = trim_point(HKm, 0, Wf_next, x_next, true);
    if ~ok0
        error('H=%.2f km、Ma=0 配平失败。', HKm);
    end

    sweep_up_mn(0, x_next, Wf_next, dMN, MNMax, dMNMin);

    print_speed_summary(results);

    overwrite_file(matfile);
    save(matfile, 'results');
    fprintf('数据已覆盖保存: %s\n', matfile);

    hFigs = plot_speed(results);
    if p.Results.SaveFig && ~isempty(hFigs)
        save_char_figure(hFigs(1), figMain);
        fprintf('图已覆盖保存（png + fig）:\n  %s\n', figMain);
        fprintf('在 MATLAB 中查看：  openfig(''%s'');\n', ...
            strrep(figMain, '.png', '.fig'));
    end

    function sweep_up_mn(mn_ok, x_ok, Wf_ok, dNom, mnMax, dMin)
        x_out = x_ok;
        Wf_out = Wf_ok;
        d = dNom;
        nGuard = 0;
        while nGuard < 80 && mn_ok < mnMax - 1e-9
            nGuard = nGuard + 1;
            mn_try = round(mn_ok + d, 4);
            if mn_try > mnMax + 1e-9
                mn_try = mnMax;
            end
            Wf_g = Wf_out * ram_pt_ratio(mn_try) / ram_pt_ratio(mn_ok);
            Wf_g = min(max(Wf_g, WfMin), WfMax);
            x_g = x_out(:);
            x_g(1) = x_out(1) * ram_pt_ratio(mn_try) / ram_pt_ratio(mn_ok);

            [~, x_try, Wf_try, ok] = trim_point(HKm, mn_try, Wf_g, x_g, true);
            if ok
                mn_ok = mn_try;
                x_out = x_try;
                Wf_out = Wf_try;
                d = dNom;
                continue
            end

            mn_mid = round(0.5 * (mn_ok + mn_try), 4);
            step = mn_mid - mn_ok;
            if step < dMin - 1e-12
                fprintf(['    Ma=%.2f 未收敛，与上一成功点 %.2f 的间隔', ...
                    '已小于 %.2f，停止加速。\n'], mn_try, mn_ok, dMin);
                break
            end
            d = step;
            fprintf('    Ma=%.2f 失败，步长减半，下一档试 %.2f\n', ...
                mn_try, mn_ok + d);
        end
        if mn_ok >= mnMax - 1e-9
            fprintf('    已到达设定上限 Ma = %.2f。\n', mnMax);
        end
    end

    function [row, x_out, Wf_out, ok] = trim_point(H_km, mn, Wf_g, x_ic, store)
        row = blank_alt_row(H_km, mn, Wf_g);
        x_out = x_ic(:);
        Wf_out = Wf_g;
        ok = false;
        Wf = min(max(Wf_g, WfMin), WfMax);
        x = x_ic(:);
        Wf_prev = nan;
        y_prev = nan;
        for it = 1:10
            [row, x, conv] = run_one_point(H_km, mn, Wf, x);
            if ~conv
                fprintf('%7.2f  %7.3f  -- 仿真未收敛 (trim %d)\n', mn, Wf, it);
                if it < 8
                    x = x_ic(:);
                    if isfinite(Wf_prev)
                        Wf = 0.5 * (Wf + Wf_prev);
                    end
                    continue
                end
                if store
                    print_speed_row(row, 'NOT CONVERGED');
                end
                return
            end
            if row.NcMap > 1.12 || row.NcMap < 0.48
                fprintf(['%7.2f  %7.3f  -- NcMap=%.4f 远离特性图 0.50～1.05', ...
                    ' (trim %d)\n'], mn, Wf, row.NcMap, it);
                row.converged = false;
                if store
                    print_speed_row(row, 'OFF MAP');
                end
                return
            end
            if row.NcMap > 1.05 + 1e-6 && ~warnedMap
                fprintf(['    注: 换算转速已超过特性图上界 1.05（本机图最高转速线）。', ...
                    '插值按 1.05 封顶，T4 仍保持不变。\n']);
                warnedMap = true;
            end
            [y, ~] = throttle_mismatch(row, mode, target);
            tol = throttle_tol(mode);
            if abs(y) <= tol
                Wf_out = Wf;
                x_out = x;
                ok = true;
                row.converged = true;
                if store
                    results = append_alt_result(results, row);
                    print_speed_row(row, 'OK');
                end
                return
            end
            if it == 1 || ~isfinite(y_prev) || abs(y - y_prev) < 1e-12
                meas = throttle_meas(row, mode);
                if abs(meas) < 1e-9
                    Wf_new = Wf * 0.9;
                else
                    Wf_new = Wf * target / meas;
                end
            else
                Wf_new = Wf - y * (Wf - Wf_prev) / (y - y_prev);
            end
            Wf_prev = Wf;
            y_prev = y;
            Wf_new = min(max(Wf_new, WfMin), WfMax);
            if abs(Wf_new - Wf) < 1e-4
                Wf_new = Wf + 0.02 * sign(target - throttle_meas(row, mode));
                Wf_new = min(max(Wf_new, WfMin), WfMax);
            end
            Wf = Wf_new;
        end
        fprintf('%7.2f  -- 油门未配平到 %s=%.4g（最后 %s=%.4g）\n', ...
            mn, mode, target, mode, throttle_meas(row, mode));
        row.converged = false;
        if store
            print_speed_row(row, 'TRIM FAIL');
        end
    end

    function [row, x_out, ok] = run_one_point(H_km, mn, Wf, x_ic)
        row = blank_alt_row(H_km, mn, Wf);
        x_out = x_ic(:);
        ok = false;

        MWS.Solve.NR_IC = x_ic(:);
        assignin('base', 'MWS', MWS);
        set_param(altBlk, 'Value', sprintf('%.8g', km2ft(H_km)));
        set_param(mnBlk, 'Value', sprintf('%.8g', mn));
        set_param(wfBlk, 'Value', sprintf('%.8g', Wf));
        set_param(solverBlk, 'SNR_IC_M', 'MWS.Solve.NR_IC');

        try
            simOut = [];
            evalc(['simOut = sim(mdl, ''ReturnWorkspaceOutputs'', ''on'', ', ...
                '''SrcWorkspace'', ''base'');']);
        catch ME
            fprintf('%7.2f  -- 仿真出错: %s\n', mn, ME.message);
            return
        end

        try
            NR_X = get_sim_var(simOut, 'NR_X');
            Fg   = get_sim_var(simOut, 'Fg_log');
            Cdat = get_sim_var(simOut, 'C_Data');
            Sdat = get_sim_var(simOut, 'S_Data_log');
            s4   = get_sim_var(simOut, 's4');
            Adat = get_sim_var_soft(simOut, 'A_Data_log');

            x = ts_last_vec(NR_X, 4);
            Fg_end = ts_last_scalar(Fg);
            idxC = last_true_index(Sdat, 'Converged');
            ok = idxC > 0;
            tHit = [];
            if ok
                tHit = ts_time_at(bus_field(Sdat, 'Converged'), idxC);
                x = ts_vec_at_time(NR_X, 4, tHit);
                Fg_end = ts_scalar_at_time(Fg, tHit);
            end

            row.NR_X     = x.';
            row.W_pps    = x(1);
            row.Rline    = x(2);
            row.PR_turb  = x(3);
            row.N_rpm    = x(4);
            row.Fg_lbf   = Fg_end;
            row.Fg_N     = Fg_end * 4.4482216153;
            row.Fram_lbf = extract_fram(Adat, row.W_pps);
            if ~(isfinite(row.Fram_lbf) && row.Fram_lbf > 0) && mn > 1e-6
                row.Fram_lbf = ram_drag_lbf(row.W_pps, mn, H_km);
            end
            if ~isfinite(row.Fram_lbf)
                row.Fram_lbf = ram_drag_lbf(row.W_pps, mn, H_km);
            end
            row.Fn_lbf   = Fg_end - row.Fram_lbf;
            row.Fn_N     = row.Fn_lbf * 4.4482216153;
            if isfinite(row.Fn_lbf) && row.Fn_lbf > 1e-6
                row.SFC_pph_lbf = 3600 * Wf / row.Fn_lbf;
                row.SFC_kgN_s   = (Wf * 0.45359237) / row.Fn_N;
            end
            if isfinite(row.Fn_lbf) && isfinite(row.W_pps) && row.W_pps ~= 0
                row.Fs_lbf_pps = row.Fn_lbf / row.W_pps;
                row.Fs_N_kgs   = row.Fn_N / (row.W_pps * 0.45359237);
            end
            row.converged = ok;
            row.PR_comp  = num_at_time(Cdat, 'PR', tHit);
            row.SM_pct   = num_at_time(Cdat, 'SMavail', tHit);
            row.Nc       = num_at_time(Cdat, 'Nc', tHit);
            row.NcMap    = num_at_time(Cdat, 'NcMap', tHit);
            if ~isfinite(row.NcMap)
                sNc = num_at_time(Cdat, 's_C_Nc', tHit);
                if ~isfinite(sNc)
                    sNc = last_num_soft(Cdat, 's_C_Nc');
                end
                if isfinite(sNc) && sNc ~= 0 && isfinite(row.Nc)
                    row.NcMap = row.Nc / sNc;
                end
            end
            row.Tt4_R = num_at_time(s4, 'Tt', tHit);
            if ok
                x_out = x(:);
            end
        catch ME
            fprintf('%7.2f  -- 读结果失败: %s\n', mn, ME.message);
            ok = false;
            row.converged = false;
        end
    end
end

function restore_altitude_cleanup(cu)
    try
        if evalin('base', 'exist(''MWS'',''var'')') == 1
            MWSb = evalin('base', 'MWS');
            MWSb.Solve.NR_IC = cu.NR_IC0;
            if isfield(cu, 'NR_dx0') && isfinite(cu.NR_dx0)
                MWSb.Solve.NR_dx = cu.NR_dx0;
            end
            if isfield(cu, 'HPC0') && ~isempty(cu.HPC0)
                MWSb.HPC = cu.HPC0;
            end
            if isfield(cu, 'SimTime0') && isfinite(cu.SimTime0)
                MWSb.in.SimTime = cu.SimTime0;
            end
            assignin('base', 'MWS', MWSb);
        end
        if bdIsLoaded(cu.mdl)
            set_param(cu.wfBlk, 'Value', cu.Wf0_str);
            set_param(cu.altBlk, 'Value', cu.Alt0_str);
            set_param(cu.mnBlk, 'Value', cu.MN0_str);
            set_param(cu.solverBlk, 'SNR_IC_M', 'MWS.Solve.NR_IC');
            remove_altitude_logs(cu.mdl);
        end
        fprintf(['已恢复高度、马赫数、燃油与 NR_IC，并删除临时记录模块。', ...
            '请不要保存 GasTurbine_SS_Template.mdl。\n']);
    catch ME
        warning('ALTITUDE:Cleanup', '退出清理未完全成功: %s', ME.message);
    end
end

function r = empty_alt_results()
    r = struct('H_km', [], 'H_ft', [], 'MN', [], 'Wf_pps', [], 'Wf_kgs', [], ...
        'N_rpm', [], 'Fn_lbf', [], 'Fn_N', [], 'Fg_lbf', [], 'Fg_N', [], ...
        'Fram_lbf', [], 'Fs_lbf_pps', [], 'Fs_N_kgs', [], ...
        'SFC_pph_lbf', [], 'SFC_kgN_s', [], 'W_pps', [], 'Rline', [], ...
        'PR_turb', [], 'PR_comp', [], 'SM_pct', [], 'Nc', [], 'NcMap', [], ...
        'Tt4_R', [], 'converged', false(0, 1), 'NR_X', zeros(0, 4), ...
        'meta', struct());
end

function row = blank_alt_row(H_km, mn, Wf)
    row.H_km = H_km;
    row.H_ft = km2ft(H_km);
    row.MN = mn;
    row.Wf_pps = Wf;
    row.Wf_kgs = Wf * 0.45359237;
    row.N_rpm = nan;
    row.Fn_lbf = nan;
    row.Fn_N = nan;
    row.Fg_lbf = nan;
    row.Fg_N = nan;
    row.Fram_lbf = nan;
    row.Fs_lbf_pps = nan;
    row.Fs_N_kgs = nan;
    row.SFC_pph_lbf = nan;
    row.SFC_kgN_s = nan;
    row.W_pps = nan;
    row.Rline = nan;
    row.PR_turb = nan;
    row.PR_comp = nan;
    row.SM_pct = nan;
    row.Nc = nan;
    row.NcMap = nan;
    row.Tt4_R = nan;
    row.converged = false;
    row.NR_X = nan(1, 4);
end

function r = append_alt_result(r, row)
    r.H_km(end+1, 1) = row.H_km;
    r.H_ft(end+1, 1) = row.H_ft;
    r.MN(end+1, 1) = row.MN;
    r.Wf_pps(end+1, 1) = row.Wf_pps;
    r.Wf_kgs(end+1, 1) = row.Wf_kgs;
    r.N_rpm(end+1, 1) = row.N_rpm;
    r.Fn_lbf(end+1, 1) = row.Fn_lbf;
    r.Fn_N(end+1, 1) = row.Fn_N;
    r.Fg_lbf(end+1, 1) = row.Fg_lbf;
    r.Fg_N(end+1, 1) = row.Fg_N;
    r.Fram_lbf(end+1, 1) = row.Fram_lbf;
    r.Fs_lbf_pps(end+1, 1) = row.Fs_lbf_pps;
    r.Fs_N_kgs(end+1, 1) = row.Fs_N_kgs;
    r.SFC_pph_lbf(end+1, 1) = row.SFC_pph_lbf;
    r.SFC_kgN_s(end+1, 1) = row.SFC_kgN_s;
    r.W_pps(end+1, 1) = row.W_pps;
    r.Rline(end+1, 1) = row.Rline;
    r.PR_turb(end+1, 1) = row.PR_turb;
    r.PR_comp(end+1, 1) = row.PR_comp;
    r.SM_pct(end+1, 1) = row.SM_pct;
    r.Nc(end+1, 1) = row.Nc;
    r.NcMap(end+1, 1) = row.NcMap;
    r.Tt4_R(end+1, 1) = row.Tt4_R;
    r.converged(end+1, 1) = row.converged;
    r.NR_X(end+1, :) = row.NR_X;
end

function print_speed_row(row, flag)
    fprintf('%7.2f  %7.3f  %8.2f  %8.4f  %8.1f  %8.2f  %8.4f  %6.2f  %s\n', ...
        row.MN, row.Wf_pps, row.W_pps, row.NcMap, row.Fn_lbf, ...
        row.Fs_lbf_pps, row.SFC_pph_lbf, row.SM_pct, flag);
end

function [y, meas] = throttle_mismatch(row, mode, target)
    meas = throttle_meas(row, mode);
    y = meas - target;
end

function meas = throttle_meas(row, mode)
    switch mode
        case 'NcMap'
            meas = row.NcMap;
        case 'N'
            meas = row.N_rpm;
        case 'T4'
            meas = row.Tt4_R;
        otherwise
            meas = nan;
    end
end

function tol = throttle_tol(mode)
    switch mode
        case 'NcMap'
            tol = 0.005;
        case 'N'
            tol = 40;
        case 'T4'
            tol = 20;
        otherwise
            tol = inf;
    end
end

function assert_idesign_frozen(mdl)
    blocks = {'Compressor', 'Turbine', 'Nozzle'};
    for i = 1:numel(blocks)
        v = str2double(get_param([mdl '/' blocks{i}], 'iDesign_M'));
        if v ~= 2
            error('%s 的 iDesign_M = %g，速度扫描必须为 2（几何冻结）。', ...
                blocks{i}, v);
        end
    end
end

function added = ensure_altitude_logs(mdl)
    logs = {
        'Log_NR_X',    'NR_X',       [1180 930 1260 960]
        'Log_Fg',      'Fg_log',     [2140 250 2220 280]
        'Log_S_Data',  'S_Data_log', [1180 980 1260 1010]
        'Log_A_Data',  'A_Data_log', [520  450 600  480]
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
    try_add_line(mdl, 'Ambient/3', 'Log_A_Data/1');
end

function remove_altitude_logs(mdl)
    names = {'Log_NR_X', 'Log_Fg', 'Log_S_Data', 'Log_A_Data'};
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

function Fram = extract_fram(Adat, W)
    Fram = last_num_soft(Adat, 'Fdrag');
    if ~isfinite(Fram)
        Fram = last_num_soft(Adat, 'Fram');
    end
    if ~isfinite(Fram)
        Veng = last_num_soft(Adat, 'Veng');
        if isfinite(Veng) && isfinite(W)
            Fram = W * Veng / 32.174;
        end
    end
    if ~isfinite(Fram)
        try
            d = ts_last_vec(Adat, 5);
            if numel(d) >= 4 && isfinite(d(4))
                Fram = d(4);
            elseif numel(d) >= 3 && isfinite(W) && isfinite(d(3))
                Fram = W * d(3) / 32.174;
            end
        catch
        end
    end
    if ~isfinite(Fram)
        Fram = 0;
    end
end

function ft = km2ft(km)
    ft = km * 3280.839895;
end

function T_R = isa_T_R(h_km)
    T_K = 216.65 * ones(size(h_km));
    inTrop = h_km < 11;
    T_K(inTrop) = 288.15 - 6.5 * h_km(inTrop);
    T_R = T_K * 9/5;
end

function Fram = ram_drag_lbf(W, mn, h_km)
    if nargin < 3 || isempty(h_km)
        h_km = 0;
    end
    a = 1116.4505 * sqrt(isa_T_R(h_km) / 518.67);
    Fram = W .* (mn .* a) / 32.174;
    Fram(~isfinite(mn) | mn <= 0) = 0;
end

function ps = tmats_ps_psi(h_ft)
    alt = 5000 * [-1 0 1 2 3 4 5 6 7 8 9 10 12 14 16];
    p = [17.554 14.696 12.228 10.108 8.297 6.759 5.461 4.373 3.468 ...
         2.73 2.149 1.692 1.049 0.651 0.406];
    h_ft = min(max(h_ft, alt(1)), alt(end));
    ps = interp1(alt, p, h_ft, 'linear');
end

function v = get_sim_var(simOut, name)
    v = get_sim_var_soft(simOut, name);
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

function s = last_num_soft(obj, fieldName)
    try
        s = last_num(obj, fieldName);
    catch
        s = nan;
    end
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

function idx = last_true_index(obj, fieldName)
    idx = 0;
    try
        y = bus_field(obj, fieldName);
        d = squeeze(ts_data(y));
        d = d(:);
        hit = find(d > 0.5, 1, 'last');
        if ~isempty(hit)
            idx = hit;
        end
    catch
    end
end

function t = ts_time_at(ts, idx)
    t = [];
    ts = unwrap_signal(ts);
    try
        if isa(ts, 'timeseries') && idx >= 1 && idx <= numel(ts.Time)
            t = ts.Time(idx);
        end
    catch
    end
end

function s = ts_scalar_at_time(ts, tHit)
    if isempty(tHit)
        s = ts_last_scalar(ts);
        return
    end
    ts = unwrap_signal(ts);
    d = squeeze(ts_data(ts));
    d = d(:);
    if isempty(d)
        s = nan;
        return
    end
    if isa(ts, 'timeseries') && ~isempty(ts.Time)
        [~, idx] = min(abs(ts.Time(:) - tHit));
        idx = min(max(idx, 1), numel(d));
        s = double(d(idx));
    else
        s = double(d(end));
    end
end

function x = ts_vec_at_time(ts, n, tHit)
    if isempty(tHit)
        x = ts_last_vec(ts, n);
        return
    end
    ts = unwrap_signal(ts);
    d = ts_data(ts);
    d = squeeze(d);
    if isempty(d)
        x = nan(n, 1);
        return
    end
    idx = [];
    if isa(ts, 'timeseries') && ~isempty(ts.Time)
        [~, idx] = min(abs(ts.Time(:) - tHit));
    end
    if isvector(d)
        x = ts_last_vec(ts, n);
        return
    elseif size(d, 1) == n
        if isempty(idx) || idx > size(d, 2)
            idx = size(d, 2);
        end
        x = d(:, idx);
    elseif size(d, 2) == n
        if isempty(idx) || idx > size(d, 1)
            idx = size(d, 1);
        end
        x = d(idx, :).';
    else
        x = ts_last_vec(ts, n);
        return
    end
    x = double(x(:));
    if numel(x) < n
        x(end+1:n, 1) = nan;
    elseif numel(x) > n
        x = x(1:n);
    end
end

function s = num_at_time(obj, fieldName, tHit)
    try
        y = bus_field(obj, fieldName);
        s = ts_scalar_at_time(y, tHit);
    catch
        s = nan;
    end
end

function HPC = pad_hpc_speed_lines(HPC, ncMax)
    nc = HPC.NcVec(:).';
    add = (nc(end) + 0.025):0.025:ncMax;
    add = add(add > nc(end) + 1e-9);
    if isempty(add)
        return
    end
    nAdd = numel(add);
    HPC.NcVec = [nc, add];
    HPC.WcArray  = [HPC.WcArray;  repmat(HPC.WcArray(end, :),  nAdd, 1)];
    HPC.EffArray = [HPC.EffArray; repmat(HPC.EffArray(end, :), nAdd, 1)];
    HPC.PRArray  = [HPC.PRArray;  repmat(HPC.PRArray(end, :),  nAdd, 1)];
    if isfield(HPC, 'WcMapSurge') && ~isempty(HPC.WcMapSurge)
        HPC.WcMapSurge = [HPC.WcMapSurge(:).', repmat(HPC.WcMapSurge(end), 1, nAdd)];
    end
    if isfield(HPC, 'PRMapSurge') && ~isempty(HPC.PRMapSurge)
        HPC.PRMapSurge = [HPC.PRMapSurge(:).', repmat(HPC.PRMapSurge(end), 1, nAdd)];
    end
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

function overwrite_file(fpath)
    if exist(fpath, 'file')
        delete(fpath);
    end
end

function save_char_figure(h, pngPath)
    [folder, name] = fileparts(pngPath);
    figPath = fullfile(folder, [name '.fig']);
    overwrite_file(pngPath);
    overwrite_file(figPath);
    saveas(h, pngPath);
    savefig(h, figPath);
end

function print_speed_summary(results)
    ok = results.converged;
    nAll = numel(results.MN);
    nOk = nnz(ok);
    fprintf('\n=== 速度扫描小结 ===\n');
    if isfield(results, 'meta') && ~isempty(results.meta)
        Hkm = nan;
        if isfield(results.meta, 'HKm')
            Hkm = results.meta.HKm;
        end
        fprintf('H = %.2f km，控制规律 %s = %.4g', ...
            Hkm, results.meta.ThrottleMode, results.meta.ThrottleTarget);
        if strcmp(results.meta.ThrottleMode, 'T4')
            fprintf('  （T4 = %.1f K 不变）', results.meta.ThrottleTarget * 5/9);
        end
        fprintf('\n');
    end
    fprintf('收敛 %d / %d 点。\n', nOk, nAll);
    if nOk < 1
        return
    end
    Mok = results.MN(ok);
    fprintf('马赫数范围 %.2f～%.2f。最高 Ma 点 Fn = %.1f lbf，SFC = %.4f（按净推力）\n', ...
        min(Mok), max(Mok), results.Fn_lbf(find(ok, 1, 'last')), ...
        results.SFC_pph_lbf(find(ok, 1, 'last')));
end

function hFigs = plot_speed(results)
    ok = results.converged;
    hFigs = gobjects(0);
    if ~any(ok)
        warning('SPEED:NoPoints', '没有收敛点，无法画速度特性。');
        return
    end
    M = results.MN(ok);
    Fg = results.Fg_lbf(ok);
    W = results.W_pps(ok);
    Wf = results.Wf_pps(ok);

    Hkm = NaN;
    mode = '';
    if isfield(results, 'meta') && ~isempty(results.meta)
        if isfield(results.meta, 'HKm')
            Hkm = results.meta.HKm;
        end
        mode = results.meta.ThrottleMode;
    end
    if ~isfinite(Hkm) && isfield(results, 'H_km') && ~isempty(results.H_km)
        Hkm = results.H_km(find(ok, 1));
    end
    Fn = Fg - ram_drag_lbf(W, M, Hkm);
    Fs = Fn ./ W;
    sfc = (3600 * Wf) ./ Fn;
    [M, idx] = sort(M);
    Fn = Fn(idx);
    W = W(idx);
    Fs = Fs(idx);
    sfc = sfc(idx);

    ttl = sprintf('速度特性（H = %.2f km，控制规律 %s，净推力 F_n）', Hkm, mode);
    if strcmp(mode, 'T4') && isfield(results, 'meta') && isfield(results.meta, 'ThrottleTarget') ...
            && isfinite(results.meta.ThrottleTarget)
        ttl = sprintf('速度特性（H = %.2f km，T_4 = %.0f K 不变，净推力 F_n）', ...
            Hkm, results.meta.ThrottleTarget * 5/9);
    end

    xmax = max(1.2, ceil((max(M) + 0.001) * 10) / 10);
    xt = 0:0.1:xmax;

    h1 = figure('Name', 'Speed characteristic', 'Color', 'w');

    subplot(2, 2, 1);
    plot(M, Fs, 'o-', 'LineWidth', 1.5);
    grid on
    apply_MN_axis(xmax, xt);
    ylabel('F_s  [lbf/(lbm/s)]');
    title('单位推力（F_n / W）');

    subplot(2, 2, 2);
    plot(M, Fn, 'o-', 'LineWidth', 1.5);
    grid on
    apply_MN_axis(xmax, xt);
    ylabel('F_n  [lbf]');
    title('净推力');

    subplot(2, 2, 3);
    plot(M, W, 'o-', 'LineWidth', 1.5);
    grid on
    apply_MN_axis(xmax, xt);
    ylabel('W  [pps]');
    title('空气流量');

    subplot(2, 2, 4);
    plot(M, sfc, 'o-', 'LineWidth', 1.5);
    grid on
    apply_MN_axis(xmax, xt);
    ylabel('SFC  [lbm/h/lbf]');
    title('耗油率（3600 W_f / F_n）');
    sgtitle(ttl);

    hFigs = h1;
end

function apply_MN_axis(xmax, xt)
    xlabel('Ma');
    xlim([0 xmax]);
    xticks(xt);
end

function r = ram_pt_ratio(mn)
    r = (1 + 0.2 * mn.^2).^3.5;
end
