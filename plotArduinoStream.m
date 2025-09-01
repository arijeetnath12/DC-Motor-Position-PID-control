port = "COM9";
baud = 115200;

s = serialport(port, baud);
configureTerminator(s,"CR/LF");
flush(s);

f = figure('Name','Velocity'); hold on; grid on;
hRef = animatedline('DisplayName','setPoint');
hPos = animatedline('DisplayName','position');
legend show
xlabel('Time(s)'), ylabel('revolutions'); t0 = tic;

while isvalid(hRef) && isvalid(hPos)
    ln = readline(s);
    v = sscanf(ln,'%f');
    if numel(v) == 2
        ref = v(1);
        y = v(2);
        
        t = toc(t0);
        addpoints(hRef,t,ref);
        addpoints(hPos,t,y);
        xlim([0 max(5,t)]);
        drawnow limitrate
    end
end
clear s
