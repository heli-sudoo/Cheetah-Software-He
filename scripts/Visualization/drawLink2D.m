function o = drawLink2D(w, l, c)
v = [-w/2,    -w/2,  w/2,  w/2;
    0,         -l,    -l,   0]';
p = patch(v(:,1), v(:,2), c);
set(p, 'FaceLighting','flat'); % Set the renderer
set(p, 'EdgeColor',[.1 .1 .1]);     % Don't show edges
set(p,'AmbientStrength',.6);
o.v = v;
o.p = p;
end