function y = cicvelsobedesce(theta,beta,L)

if theta < beta/2
   y = L/beta*2*(1 - cos(2*pi*theta/beta*2));
else
   y = -L/beta*2*(1 - cos(2*pi*theta/beta*2));
end