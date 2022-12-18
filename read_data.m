clear all
clc
vref_value = 330;
device = serialport("COM6", 9600);
signal = [];
while(1)
    aux = read(device, 1, "uint16");
    if(aux == -1)
        continue
    end
    temp = aux ;
    signal = [signal temp];
    plot(signal);
    ylim([0 100])
    drawnow;
end