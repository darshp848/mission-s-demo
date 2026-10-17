function [command,state]=ms_pulse_schedule(demand,pressure,state,P,enabled)
%MS_PULSE_SCHEDULE Impulse-error modulation, fixed thrust and finite dwell.
% state=[impulse credit/full-thrust seconds; valve binary; dwell seconds].
n=numel(demand);credit=state(1:n);gate=state(n+(1:n));age=state(2*n+(1:n));
command=zeros(n,1);pressure=max(.2,min(1,pressure));
for k=1:n
    if ~enabled
        credit(k)=0;gate(k)=0;age(k)=P.pulse_min_off;
    else
        duty=min(1,max(0,demand(k)/(P.Tmax(k+2)*pressure)));
        credit(k)=min(P.pulse_credit_max,credit(k)+duty*P.Ts);
        if gate(k)>.5
            if age(k)>=P.pulse_min_on-1e-10 && credit(k)<=0
                gate(k)=0;age(k)=0;
            end
        elseif age(k)>=P.pulse_min_off-1e-10 && credit(k)>=P.pulse_min_on-1e-10
            gate(k)=1;age(k)=0;
        end
        credit(k)=max(-P.pulse_credit_max,credit(k)-gate(k)*P.Ts);
        age(k)=min(1,age(k)+P.Ts);
    end
    command(k)=gate(k)*P.Tmax(k+2);
end
state=[credit;gate;age];
end
