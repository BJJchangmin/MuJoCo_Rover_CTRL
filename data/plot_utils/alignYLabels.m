function alignYLabels(fig)
    % Align Y Labels
    % subplot들의 y축 레이블을 세로로 한 줄에 맞춘다.
    %
    % 눈금 숫자 폭이 subplot마다 다르면(0.3 vs -0.02) 레이블이 제각각 밀린다.
    % 가장 왼쪽으로 밀린 위치를 기준으로 전부 맞춘다.
    %
    % fig : 대상 figure. 생략하면 현재 figure.
    %
    % 화면 그림에 바로 쓸 수도 있다:
    %   alignYLabels(figure(5))

    if nargin < 1 || isempty(fig)
        fig = gcf;
    end

    ax = findall(fig, 'Type', 'axes');
    if length(ax) < 2
        return;
    end

    drawnow;  % 눈금 폭이 확정된 뒤에 위치를 읽어야 한다

    % 레이블이 있는 axes만 모은다
    target = gobjects(0);
    for k = 1:length(ax)
        if ~isempty(ax(k).YLabel.String)
            target(end+1) = ax(k); %#ok<AGROW>
        end
    end
    if length(target) < 2
        return;
    end

    % 위치를 normalized로 읽는다. axes 크기가 같으므로 서로 비교할 수 있다.
    x = zeros(1, length(target));
    for k = 1:length(target)
        target(k).YLabel.Units = 'normalized';
        x(k) = target(k).YLabel.Position(1);
    end

    % 가장 왼쪽 위치로 통일한다
    x_min = min(x);
    for k = 1:length(target)
        p = target(k).YLabel.Position;
        p(1) = x_min;
        target(k).YLabel.Position = p;
    end
end
