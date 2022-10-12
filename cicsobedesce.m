function y = cicsobedesce(theta,beta,L)

if theta < beta/2
   y = L*(theta/beta*2 - sin(2*pi*theta/beta*2)/2/pi);
else
    y = L*(1 - (theta - beta/2)/beta*2 + sin(2*pi*(theta - beta/2)/beta*2)/2/pi);
end