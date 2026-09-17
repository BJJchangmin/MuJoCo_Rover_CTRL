function saveAllFigures(folder_name, resolution, fig_size_cm, font_pt, keep_title)
    % Save All Figures as PNG
    % 열려 있는 모든 figure를 PNG로 저장한다.
    %
    % folder_name : 저장 폴더 이름. 생략하면 'figure png'
    % resolution  : 해상도 [dpi]. 생략하면 300
    % fig_size_cm : 저장할 그림 크기 [가로 세로] (cm). 생략하면 화면 크기 그대로
    %               크기를 지정해도 화면에 떠 있는 창은 건드리지 않는다.
    %               숨겨진 복사본을 만들어 저장하고 바로 지운다.
    % font_pt     : 저장본 글자 크기 [pt]. 항목별로 따로 지정하는 구조체다.
    %                 .sgtitle  그림 전체 제목 (sgtitle)
    %                 .title    subplot 제목
    %                 .label    x, y 축 레이블
    %                 .legend   범례
    %                 .tick     눈금 숫자
    %               생략하면 saveFigureFontSize()의 값을 쓴다.
    %               개별 항목을 []로 두면 그 항목만 자동 축소된다.
    % keep_title  : 그림 안에 제목을 남길지 여부. 생략하면 false = 제목을 지운다.
    %               PPT나 논문에서 제목을 따로 다는 경우를 기본으로 본다.
    %               파일 이름에는 제목이 그대로 쓰이므로 어느 그림인지는 알 수 있다.
    %               subplot마다 붙은 제목(FL, FR 등)은 지우지 않는다.
    %
    % 예) saveAllFigures('figure png', 300, [8.5 6])           8.5 x 6 cm
    %     saveAllFigures('figure png', 300, [8.5 6], [], true) 제목까지 남기기
    %
    %     f = saveFigureFontSize(); f.tick = 10;               일부만 바꿔 쓰기
    %     saveAllFigures('figure png', 300, [8.5 6], f);
    %
    % 파일 이름은 figure의 제목을 그대로 쓴다. sgtitle이 있으면 그것을,
    % 없으면 axes title을, 둘 다 없으면 figure 번호를 쓴다.
    % 같은 이름이 이미 있으면 덮어쓴다.

    if nargin < 1 || isempty(folder_name)
        folder_name = 'figure png';
    end
    if nargin < 2 || isempty(resolution)
        resolution = 300;
    end
    if nargin < 3
        fig_size_cm = [];
    end
    if nargin < 4 || isempty(font_pt)
        font_pt = saveFigureFontSize();
    end
    if nargin < 5 || isempty(keep_title)
        keep_title = false;
    end
    if ~isempty(fig_size_cm) && numel(fig_size_cm) ~= 2
        error('fig_size_cm은 [가로 세로] 두 개여야 한다.');
    end

    % 이 함수는 data/plot_utils 안에 있다. 저장은 그 상위인 data 폴더 기준으로 한다.
    % 스크립트를 어디서 실행하든 항상 같은 곳에 저장된다.
    base_dir = fileparts(fileparts(mfilename('fullpath')));
    save_dir = fullfile(base_dir, folder_name);
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end

    % Get all figure handles
    fig_handles = findall(groot, 'Type', 'figure');
    fig_handles = flipud(fig_handles);  % figure 번호 오름차순

    % 저장용 임시 figure는 제외한다.
    % 이전 실행이 중간에 죽으면 숨겨진 채로 남아 있을 수 있다.
    keep = true(size(fig_handles));
    for i = 1:length(fig_handles)
        keep(i) = isvalid(fig_handles(i)) && ...
                  ~strcmp(fig_handles(i).Tag, tmpFigureTag());
    end
    fig_handles = fig_handles(keep);

    use_exportgraphics = ~isempty(which('exportgraphics'));  % R2020a 이상

    used_names = {};
    saved = 0;
    for i = 1:length(fig_handles)
        fig = fig_handles(i);
        if ~isvalid(fig)
            continue;  % 저장 도중에 닫힌 창은 건너뛴다
        end

        name = getFigureTitle(fig);

        % 한 번 실행하는 동안 제목이 겹치면 figure 번호를 붙여 구분한다
        if any(strcmp(used_names, name))
            name = sprintf('%s_fig%d', name, fig.Number);
        end
        used_names{end+1} = name; %#ok<AGROW>

        file_path = fullfile(save_dir, [name '.png']);

        % 화면 창은 손대지 않는다. 숨겨진 복사본에서만 크기, 폰트, 제목을 바꾼다.
        % 하나가 실패해도 나머지는 계속 저장한다.
        target = gobjects(0);
        try
            target = copyFigureHidden(fig, fig_size_cm, font_pt, keep_title);

            if use_exportgraphics
                exportgraphics(target, file_path, 'Resolution', resolution);
            else
                print(target, file_path, '-dpng', sprintf('-r%d', resolution));
            end
            saved = saved + 1;
        catch err
            warning('%s 저장 실패: %s', name, err.message);
        end

        if ~isempty(target) && isvalid(target)
            delete(target);
        end
    end

    fprintf('%d개 figure 저장 완료: %s\n', saved, save_dir);
end


function tag = tmpFigureTag()
    % 저장용 임시 figure를 알아보기 위한 표식
    tag = 'saveAllFigures_tmp';
end


function tmp = copyFigureHidden(fig, fig_size_cm, font_pt, keep_title)
    % 화면 창을 건드리지 않기 위해, 내용만 복사한 숨겨진 figure를 만든다

    % 원본 크기를 cm로 읽는다. Units만 바꿔 읽는 것이라 창은 움직이지 않는다.
    old_units = fig.Units;
    fig.Units = 'centimeters';
    size_cm   = fig.Position(3:4);
    fig.Units = old_units;

    if isempty(fig_size_cm)
        fig_size_cm = size_cm;  % 크기 지정이 없으면 화면 크기 그대로
    end

    % 따로 지정하지 않은 항목에 적용할 자동 배율
    auto_scale = fig_size_cm(1) / size_cm(1);

    tmp = figure('Visible', 'off', 'Color', fig.Color, 'Tag', tmpFigureTag());
    tmp.Units = 'centimeters';
    tmp.Position(3:4) = fig_size_cm(:).';

    % axes, legend, sgtitle을 한 번에 복사해야 서로의 연결이 유지된다
    copyobj(allchild(fig), tmp);

    if ~keep_title
        dropTitle(tmp);
    end

    % 화면에서 이미 걸었으면 manual이라 그대로 복사돼 있고, 아니면 여기서 맞춘다
    setYLimMargin(tmp);

    applyFonts(tmp, font_pt, auto_scale);

    % 범례를 위쪽 여백으로 빼서 데이터를 가리지 않게 한다
    moveLegendToTop(tmp);

    % 폰트를 바꾸면 눈금 폭이 달라지므로 그 뒤에 레이블을 맞춘다
    alignYLabels(tmp);
end


function dropTitle(fig)
    % 그림 전체의 제목만 지운다. subplot마다 붙은 제목은 남긴다.

    sg = findall(fig, 'Type', 'subplottext');
    if ~isempty(sg)
        % sgtitle이 그림의 제목이다. subplot 제목은 건드리지 않는다.
        delete(sg);
        return;
    end

    % sgtitle이 없으면 axes title이 그림의 제목이다
    ax = findall(fig, 'Type', 'axes');
    for k = 1:length(ax)
        ax(k).Title.String = '';
    end
end


function applyFonts(fig, f, s)
    % 저장본의 글자 크기를 항목별로 맞춘다.
    % f에 값이 있으면 그 값을 pt 단위 절대 크기로 쓰고,
    % 없으면 화면 크기에 자동 배율 s를 곱한 값을 쓴다.

    % axes: 눈금 숫자와 그 axes에 딸린 제목, 축 레이블
    ax = findall(fig, 'Type', 'axes');
    for k = 1:length(ax)
        ax(k).FontSize = fontValue(f, 'tick', ax(k).FontSize, s);

        ax(k).Title.FontSize = fontValue(f, 'title', ax(k).Title.FontSize, s);

        for h = [ax(k).XLabel, ax(k).YLabel, ax(k).ZLabel]
            h.FontSize = fontValue(f, 'label', h.FontSize, s);
        end
    end

    % sgtitle
    sg = findall(fig, 'Type', 'subplottext');
    for k = 1:length(sg)
        sg(k).FontSize = fontValue(f, 'sgtitle', sg(k).FontSize, s);
    end

    % 범례
    lg = findall(fig, 'Type', 'legend');
    for k = 1:length(lg)
        lg(k).FontSize = fontValue(f, 'legend', lg(k).FontSize, s);
    end

    % colorbar나 text 주석처럼 위에 안 걸린 나머지는 자동 배율만 적용한다
    others = [findall(fig, 'Type', 'colorbar'); findall(fig, 'Type', 'text')];
    for k = 1:length(others)
        others(k).FontSize = others(k).FontSize * s;
    end
end


function v = fontValue(f, field, current, s)
    % 구조체에 값이 있으면 그 값을, 없으면 원래 크기에 자동 배율을 곱한 값을 준다

    if isstruct(f) && isfield(f, field) && ~isempty(f.(field))
        v = f.(field);
    else
        v = current * s;
    end
end


function name = getFigureTitle(fig)
    % figure에서 파일 이름으로 쓸 제목을 뽑아낸다

    name = '';

    % 1순위: sgtitle
    sg = findall(fig, 'Type', 'subplottext');
    if ~isempty(sg)
        name = sg(1).String;
    end

    % 2순위: axes title (subplot이면 첫 번째 것)
    if isempty(name)
        ax = findall(fig, 'Type', 'axes');
        for k = length(ax):-1:1
            if ~isempty(ax(k).Title.String)
                name = ax(k).Title.String;
                break;
            end
        end
    end

    % 3순위: figure 번호
    if isempty(name)
        name = sprintf('figure_%d', fig.Number);
    end

    name = sanitizeFileName(name);
end


function s = sanitizeFileName(s)
    % 제목을 파일 이름으로 쓸 수 있게 정리한다

    if iscell(s)
        s = strjoin(s, ' ');
    end
    s = char(s);

    % latex 문법 제거
    s = strrep(s, '$', '');
    s = strrep(s, '\', '');
    s = strrep(s, '{', '');
    s = strrep(s, '}', '');

    % 파일 이름에 못 쓰는 문자 제거
    s = regexprep(s, '[<>:"/\\|?*]', '');

    % 공백 정리: 앞뒤 제거 후 밑줄로
    s = strtrim(s);
    s = regexprep(s, '\s+', '_');

    if isempty(s)
        s = 'figure';
    end
end
