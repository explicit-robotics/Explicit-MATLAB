function t = comms(down, up)
    
    bt1 = 10/38400;
    bt2 = 10/1e6;
    
    rd = 250;
    
    t = (6+down)*(bt1+bt2) + 250*2e-6 + (6+up)*(bt1+bt2);
end