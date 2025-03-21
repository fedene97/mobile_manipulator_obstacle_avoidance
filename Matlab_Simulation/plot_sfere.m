
function plot_sfere(ostacoli, d_soglia)

for i=1:size(ostacoli, 2)
    
centro=ostacoli(:,i);
plot3(centro(1), centro(2), centro(3),'or');hold on
[x,y,z] = sphere;
surf(d_soglia*x+centro(1),d_soglia*y+centro(2),d_soglia*z+centro(3),...
   'FaceColor',[0.5 0.7 0.8],'EdgeColor',[0.4 0.6 0.7]) 
end