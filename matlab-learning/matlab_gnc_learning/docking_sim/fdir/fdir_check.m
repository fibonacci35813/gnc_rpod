function [code,msg]=fdir_check(F_cmd,F_act,valid,dropout,cov_tr,p)
code=0; msg='NOMINAL';
if norm(F_act)>p.thr_stuck_open && norm(F_cmd)<p.thr_cmd_zero
    code=1; msg='STUCK_OPEN'; return; end
if norm(F_cmd)>p.thr_cmd_min && norm(F_act)<p.thr_stuck_closed
    code=2; msg='STUCK_CLOSED'; return; end
if dropout>p.dropout_limit
    code=3; msg='DROPOUT'; return; end
if cov_tr>p.cov_max
    code=4; msg='NAV_DIVERGE'; return; end
end
