function o = drawBody2D(l, h, c)
v =  [-l/2, -l/2,  l/2,  l/2;
      h/2,  -h/2,  -h/2,  h/2]';
p = patch(v(:,1), v(:,2), c);
set(p, 'FaceLighting','flat'); % Set the renderer
set(p, 'EdgeColor',[.1 .1 .1]);     % Don't show edges
set(p,'AmbientStrength',.6);
o.v = v;
o.p = p;
end