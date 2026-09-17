function moveLegendToTop(fig, margin)
% Move Legend To Top
% 범례를 subplot 안에서 빼내 그림 맨 위에 가로로 한 줄로 놓는다.
%
% subplot 안에 있으면 데이터를 가리는데, 위로 옮기면
% 위치가 항상 같고 아무것도 가리지 않는다.
% 축 위에 자리가 모자랄 때만 축을 아래로 눌러 자리를 만든다.
%
% fig    : 대상 figure. 생략하면 현재 figure.
% margin : figure 맨 위와 범례 사이 간격 (figure 높이 대비 비율).
%          생략하면 0.02. 키우면 범례가 아래로 내려간다.
%          너무 큰 값은 0.15에서 잘라낸다.
%
% 화면 그림에 바로 쓸 수도 있다:
%   moveLegendToTop(figure(1))

if nargin < 1 || isempty(fig)
    fig = gcf;
end
if nargin < 2 || isempty(margin)
    margin = 0.01;
end
margin = min(max(margin, 0), 0.15);

lg = findall(fig, 'Type', 'legend');
if isempty(lg)
    return;
end
lg = lg(1);  % 그림당 범례 하나를 전제로 한다

lg.Orientation = 'horizontal';
lg.Box = 'off';
drawnow;

% figure와 범례 크기를 cm로 읽어 직접 정규화한다.
% 숨겨진 figure는 레이아웃이 덜 돌아 Position이 엉뚱하게 나올 때가 있어서,
% 읽은 값을 그대로 믿지 않고 아래에서 글자 크기 기준으로 잘라낸다.
old_units = fig.Units;
fig.Units = 'centimeters';
fig_cm    = fig.Position(3:4);
fig.Units = old_units;

lg.Units = 'centimeters';
lg_cm    = lg.Position(3:4);

% 한 줄짜리 가로 범례의 높이는 글자 크기로 정해진다.
% 이 범위를 벗어나면 잘못 읽은 값으로 보고 잘라낸다.
font_cm = lg.FontSize / 72 * 2.54;
h_cm    = min(max(lg_cm(2), 1.2 * font_cm), 2.5 * font_cm);

lg_w = min(lg_cm(1) / fig_cm(1), 0.95);
lg_h = min(h_cm     / fig_cm(2), 0.15);

if ~isfinite(lg_w) || ~isfinite(lg_h) || lg_w <= 0 || lg_h <= 0
    return;  % 크기를 못 읽으면 손대지 않는다
end

% 범례를 놓을 위쪽 한계. sgtitle이 있으면 그 아래로 내린다.
top = 1 - margin;
sg  = findall(fig, 'Type', 'subplottext');
if ~isempty(sg)
    sg(1).Units = 'normalized';
    top = min(top, sg(1).Position(2) - margin);
end

lg_y  = top - lg_h;      % 범례 아래쪽 y
limit = lg_y - margin;   % 축이 올라올 수 있는 상한

% 축이 범례 자리를 침범할 때만 그만큼 눌러 넣는다.
% 이미 여유가 있으면 축은 건드리지 않는다.
ax = findall(fig, 'Type', 'axes');
ax_top = 0;
for k = 1:length(ax)
    ax(k).Units = 'normalized';
    p = ax(k).Position;
    ax_top = max(ax_top, p(2) + p(4));
end

if ax_top > limit && ax_top > 0 && limit > 0
    scale = limit / ax_top;
    for k = 1:length(ax)
        p = ax(k).Position;
        p(2) = p(2) * scale;
        p(4) = p(4) * scale;
        ax(k).Position = p;
    end
end

% 가운데 정렬해서 올린다
lg.Units    = 'normalized';
lg.Position = [0.5 - lg_w/2, lg_y, lg_w, lg_h];
end
