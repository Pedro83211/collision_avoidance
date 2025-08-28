clf

ctrl = [[453.24964570999146; 452.80157470703125; 448.9611220359802; 452.33608078956604; 456.95875120162964], [300; 200; 400; 500; 300]]
dra = [[262.3055112361908; 283.5412611961365; 288.3907947540283; 265.29141640663147; 282.06239652633667], [300; 200; 400; 500; 400]];
%plot the bar graph
h = bar([mean(ctrl, 'Omitnan'); mean(dra, 'Omitnan')]');
hold on
%h(1).FaceColor='r';
%h(2).FaceColor='y';
%plot the error bars
%plot the individual points
scatter(repmat(h(1).XEndPoints(1), size(ctrl, 1), 1),ctrl(:, 1),30,'MarkerFaceColor','g','MarkerEdgeColor','k','LineWidth',0.2)
scatter(repmat(h(1).XEndPoints(2), size(ctrl, 1), 1),ctrl(:, 2),30,'MarkerFaceColor','g','MarkerEdgeColor','k','LineWidth',0.2)
scatter(repmat(h(2).XEndPoints(1), size(dra, 1), 1),dra(:, 1),30,'MarkerFaceColor','g','MarkerEdgeColor','k','LineWidth',0.2)
scatter(repmat(h(2).XEndPoints(2), size(dra, 1), 1),dra(:, 2),30,'MarkerFaceColor','g','MarkerEdgeColor','k','LineWidth',0.2)
%Give appropriate labels
set(gca,'XTickLabel',["400" "1600"],'fontweight','bold')
ylabel('Relative Power (dB)','fontweight','bold','fontsize',11)
legend(["Ctrl", "Dra"])
hold off