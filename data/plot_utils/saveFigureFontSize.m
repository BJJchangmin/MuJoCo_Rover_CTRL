function f = saveFigureFontSize()
% Save Figure Font Size
% 저장되는 png의 글자 크기 [pt]. 여기 숫자만 바꾸면 된다.
% 화면에 뜨는 figure는 이 값과 무관하다. Rover_data_code의
% lw, sgT, Faxis, fl 이 화면용이고, 여기는 저장용이다.
%
% 값을 []로 두면 그 항목만 자동 계산된다.
% (그림이 줄어든 비율만큼 화면 크기에서 축소)

f.sgtitle = 10;   % 그림 전체 제목. keep_title을 켰을 때만 보인다
f.title   = 9;    % subplot 제목 (FL, FR, RL, RR 등)
f.label   = 12;    % x, y 축 레이블
f.legend  = 8;    % 범례
f.tick    = 5;    % 눈금 숫자
end
