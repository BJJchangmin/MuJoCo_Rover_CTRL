function setXLimRange(figs, x_range, force)
% Set X Limit Range
% 시간 축을 쓰는 subplot의 x 범위를 지정한 값으로 통일한다.
%
% figs    : 대상 figure 또는 figure 배열. 생략하면 열려 있는 모든 figure.
% x_range : [시작 끝]. 생략하면 아무것도 하지 않는다.
% force   : true면 스크립트에서 xlim을 직접 건 축까지 덮어쓴다.
%           생략하면 false = 직접 건 축은 건드리지 않는다.
%           (Trunk Map처럼 x가 시간이 아닌 그림을 지키기 위한 것이다)
%
% 예)
%   setXLimRange([], [0 30]);        모든 figure를 0 ~ 30 s
%   setXLimRange([], [5 30]);        5초부터 보기

if nargin < 2 || isempty(x_range)
    return;
end
if nargin < 1 || isempty(figs)
    figs = findall(groot, 'Type', 'figure');
end
if nargin < 3 || isempty(force)
    force = false;
end

if numel(x_range) ~= 2 || ~all(isfinite(x_range)) || x_range(2) <= x_range(1)
    error('x_range는 [시작 끝] 이어야 하고 끝이 시작보다 커야 한다.');
end

for f = reshape(figs, 1, [])
    if ~isvalid(f)
        continue;
    end

    ax = findall(f, 'Type', 'axes');
    for k = 1:length(ax)
        % x가 시간이 아닌 그림(예: Trunk Map)은 xlim을 직접 걸어 두었다
        if ~force && strcmp(ax(k).XLimMode, 'manual')
            continue;
        end
        xlim(ax(k), x_range);
    end
end
end
