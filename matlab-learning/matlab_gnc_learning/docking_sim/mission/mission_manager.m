function [mode_out,override]=mission_manager(mode_in,fault,pos,timer,p)
override=[]; mode_out=mode_in;
switch mode_in
    case 0  % NOMINAL
        if fault>0; mode_out=1; end
    case 1  % HOLD
        if fault==1; mode_out=2;            % stuck-open: immediate retreat
        elseif fault==0&&timer>p.hold_timeout; mode_out=0;  % de-escalate
        elseif timer>p.hold_max; mode_out=2; end  % escalate
        override=pos;
    case 2  % RETREAT
        if fault==1; mode_out=2;            % stuck-open: latch in RETREAT
        elseif fault==0&&timer>p.hold_timeout; mode_out=0;  % fault cleared: return NOMINAL
        elseif timer>p.retreat_timeout; mode_out=3; end     % unresolved: escalate SAFE
        override=[0;200;0];
    case 3  % SAFE
        override=[0;50;0];
        if fault~=1 && timer>p.hold_timeout; mode_out=0; end  % exit when fault clears
end
end
