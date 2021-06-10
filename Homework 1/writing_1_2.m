c = 0:0.01:1;
figure, plot(c, (0.24*c + 0.36)./(0.16*c + 0.44))
xlabel('c')
ylabel('bel(x_{t+1}=clean)')