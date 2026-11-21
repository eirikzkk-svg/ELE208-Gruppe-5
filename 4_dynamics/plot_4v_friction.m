function plot_4v_friction(here,tr,run)
%PLOT_4V_FRICTION Plot current applied torques, retaining both event limits.
% AI assistance with drafting and adapting the torque plots, event-limit
% connectors and figure-export code.
set(groot,'defaultAxesFontSize',11,'defaultLineLineWidth',1.5);
colors=[0.08 0.32 0.66;0.85 0.28 0.12];names={'nominal','maximum'};
fig=figure('Visible','off','Color','w','Position',[30 30 1200 1250]);
tiledlayout(5,1,'TileSpacing','compact','Padding','compact');
for j=1:5
    nexttile;hold on;h=gobjects(1,2);
    for c=1:2
        h(c)=drawPieces(tr,run.(names{c}),j,colors(c,:),'-');
    end
    markEvents();ylabel(sprintf('J%d torque [N m]',j));grid on;xlim([0 6]);
    if j==1,legend(h,{'0.10 kg object','5 kg object'},'Location','best');end
end
xlabel('Time [s]');
sgtitle({'Full five-joint RNE | adopted shoulder viscous friction', ...
    'Same 2 kg gripper | prescribed motion | object carried 1.8-4.2 s'});
saveFigure(fig,fullfile(here,'torques.png'));close(fig);

end

function h=drawPieces(tr,out,j,color,style)
for p=1:numel(tr.pieces)
    a=plot(tr.pieces(p).t,out.pieces(p).tau(:,j),'Color',color,'LineStyle',style);
    if p==1,h=a;end
    if p<numel(tr.pieces)
        t=tr.pieces(p).t(end);
        plot([t t],[out.pieces(p).tau(end,j) out.pieces(p+1).tau(1,j)], ...
            ':','Color',color,'HandleVisibility','off');
    end
end
end
function markEvents()
xline(1.8,'--','Color',[.3 .3 .3],'HandleVisibility','off');
xline(4.2,'--','Color',[.3 .3 .3],'HandleVisibility','off');
end
function saveFigure(fig,file)
if isprop(fig,'Theme'),fig.Theme='light';end
set(findall(fig,'Type','axes'),'Color','w','XColor',[.12 .12 .12], ...
    'YColor',[.12 .12 .12],'GridColor',[.65 .65 .65]);
set(findall(fig,'Type','text'),'Color',[.1 .1 .1]);
set(findall(fig,'Type','legend'),'Color','w','TextColor',[.1 .1 .1],'EdgeColor',[.5 .5 .5]);
exportgraphics(fig,file,'Resolution',160,'BackgroundColor','white');
end
