% fid = fopen('data_NOfly', 'r');
% raw = fread(fid, inf);
% str = char(raw');
% fclose(fid);
% 
% data = jsondecode(str);
dataNoFLy = readJSON('data_FLY_Fmax_May8');
dataFLy   = readJSON('data_NOFLY_Fmax_May8'); 


%%
[frc_x,idx] = sort(dataNoFLy.frc_x_unique);
frc_y   = dataNoFLy.frc_y_unique(idx); 
norbtfly = dataNoFLy.Fz_max(idx); 
FRC_x   = reshape(frc_x,20,20);
FRC_y   = reshape(frc_y,20,20);

rbtfly = dataFLy.Fz_max(idx); 




FLYRBT = reshape(rbtfly,20,20);
FLYRBT (FLYRBT> 1500)  = 1500; 
TotFLYFails = sum(sum(FLYRBT == 1));
TotFLYSucc = sum(sum(FLYRBT == 0));

FLYRBT(FLYRBT == 0) = nan; 
% 
NOFLYRBT = reshape(norbtfly,20,20);
NOFLYRBT (NOFLYRBT> 1500)  = 1500; 
TotNOFLYFails = sum(sum(NOFLYRBT == 1));
TotNOFLYSuccs = sum(sum(NOFLYRBT == 0));

NOFLYRBT(NOFLYRBT == 0 ) = nan; 

sprintf("fly wheel fails: %i with %i tot succs ",TotFLYFails,TotFLYSucc)
sprintf("NO fly wheel fails: %i with %i tot succs",TotNOFLYFails,TotNOFLYSuccs)


figure; 
HTMP = heatmap(FRC_x(1,:),FRC_y(:,1) ,FLYRBT);
colors = [linspace(0, 1, 256)', linspace(1, 0, 256)', zeros(256, 1)];
HTMP.Colormap = colors;
HTMP.CellLabelColor="none"; 
HTMP.XLabel = "Velocity in +X";
HTMP.YLabel = "Velocity in +Y";
HTMP.YDisplayData = flip(HTMP.YDisplayData); 
HTMP.GridVisible = "off";
title("FLY")

figure; 
HTMP = heatmap(FRC_x(1,:),FRC_y(:,1) ,NOFLYRBT);
colors = [linspace(0, 1, 256)', linspace(1, 0, 256)', zeros(256, 1)];
HTMP.Colormap = colors;
HTMP.CellLabelColor="none"; 
HTMP.XLabel = "Velocity in +X";
HTMP.YLabel = "Velocity in +Y";
HTMP.YDisplayData = flip(HTMP.YDisplayData); 
HTMP.GridVisible = "off";
title("NO FLY")




function data = readJSON(filename) 
    fid = fopen(filename, 'r');
    raw = fread(fid, inf);
    str = char(raw');
    fclose(fid);
    data = jsondecode(str);
   
end

