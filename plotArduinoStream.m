port = "COM9";
baud = 115200;

s = serialport(port, baud);
configureTerminator(s,"CR/LF");
flush(s);

f = figure('Name','Velocity'); hold on; grid on;
hRef = animatedline('DisplayName','ref','Color',[0,0,1]);
hPos = animatedline('DisplayName','y','Color',[1,0,0]);
legend show
xlabel('Time(s)'), ylabel('counts'); t0 = tic;


while isvalid(hRef) && isvalid(hPos)
    ln = readline(s);
    v = sscanf(ln,'%f');
    if numel(v) == 2
        sgn = sign(v(1)); if sgn == 0, sgn =1; end
        ref = sgn*v(1);
        y = sgn*v(2);
        
        t = toc(t0);
        addpoints(hRef,t,ref);
        addpoints(hPos,t,y);
        xlim([0 max(5,t)]);
        drawnow limitrate
    end
end
clear s
