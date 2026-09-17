function setYLimMargin(figs, ratio, force)
% Set Y Limit Margin
% subplot의 y축 범위를 데이터 최소/최대에 여백을 붙여 정한다.
%
% y레이블이 같은 subplot끼리 묶어서 같은 범위를 준다.
% 네 다리 그림(FL, FR, RL, RR)은 레이블이 전부 같으므로 네 개 데이터를
% 모두 훑어 하나의 범위로 맞춘다. 다리끼리 크기를 바로 비교할 수 있다.
% figure 5처럼 subplot마다 물리량이 다르면 레이블이 다르므로 따로 계산한다.
%
% 범위는 현재 xlim 안에 들어오는 데이터만 보고 정한다.
% 따라서 setXLimRange를 먼저 부르고 이 함수를 나중에 불러야 한다.
%
% figs  : 대상 figure 또는 figure 배열. 생략하면 열려 있는 모든 figure.
% ratio : 여백 비율. 생략하면 0.01 (데이터 범위의 1%).
% force : true면 스크립트에서 ylim을 직접 건 축까지 덮어쓴다.
%         생략하면 false = 직접 건 축은 건드리지 않는다.
%
% 데이터에서 매번 다시 계산하므로 여러 번 불러도 여백이 누적되지 않는다.

if nargin < 1 || isempty(figs)
    figs = findall(groot, 'Type', 'figure');
end
if nargin < 2 || isempty(ratio)
    ratio = 0.01;
end
if nargin < 3 || isempty(force)
    force = false;
end

for f = reshape(figs, 1, [])
    if ~isvalid(f)
        continue;
    end

    ax = findall(f, 'Type', 'axes');

    % 스크립트에서 ylim을 직접 건 축은 그 의도를 존중한다
    keep = false(size(ax));
    for k = 1:length(ax)
        keep(k) = force || strcmp(ax(k).YLimMode, 'auto');
    end
    ax = ax(keep);
    if isempty(ax)
        continue;
    end

    % y레이블이 같은 것끼리 묶는다
    labels = cell(1, length(ax));
    for k = 1:length(ax)
        labels{k} = labelText(ax(k).YLabel.String);
    end
    [groups, ~, gidx] = unique(labels);

    for g = 1:length(groups)
        grp = ax(gidx == g);

        % 묶인 축 전체에서 최소/최대를 찾는다
        y = [];
        for k = 1:length(grp)
            y = [y; collectYData(grp(k))]; %#ok<AGROW>
        end
        if isempty(y)
            continue;
        end

        lo = min(y);
        hi = max(y);

        span = hi - lo;
        if span <= 0
            % 상수 데이터. 값 크기를 기준으로 최소한의 여백을 준다.
            span = max(abs(hi), 1);
        end
        pad = span * ratio;

        for k = 1:length(grp)
            ylim(grp(k), [lo - pad, hi + pad]);
        end
    end
end
end


function y = collectYData(ax)
% 축 안의 모든 선에서 y값을 모은다.
% 현재 xlim 밖의 구간과 NaN, Inf는 뺀다.

y  = [];
xl = ax.XLim;
h  = findall(ax, 'Type', 'line');

for k = 1:length(h)
    xd = h(k).XData(:);
    yd = h(k).YData(:);

    if length(xd) == length(yd)
        yd = yd(xd >= xl(1) & xd <= xl(2));
    end

    y = [y; yd(isfinite(yd))]; %#ok<AGROW>
end
end


function s = labelText(s)
% y레이블을 비교할 수 있는 문자열로 바꾼다

if iscell(s)
    s = strjoin(s, ' ');
end
s = char(s);
if isempty(s)
    s = '';
end
end
