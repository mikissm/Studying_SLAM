# Studying SLAM

SLAM과 점군 기반 지형 분석을 공부하고 실험하는 저장소입니다.

## Elevation Lab

PCD 파일에서 지면 표면을 추출하고, 로봇 높이에 따른 장애물과 통과 가능 후보를 3D로 확인하는 로컬 도구입니다.

- 높이별 표면 후보와 연결성을 이용한 바닥 선택
- 바닥 높이와 전체 원본 점군의 장애물 판정 분리
- 희소 지면 복구·보간과 출처 구분
- 파라미터 조정, 3D 회전·이동, 표면/통과 가능성/진단 보기
- 원형 로봇 반경·높이·단차·경사 검사와 지형 A* 경로 계획
- NPZ·PCD·CSV·파라미터 JSON 및 경로 JSON 저장

```bash
git clone https://github.com/mikissm/Studying_SLAM.git
cd Studying_SLAM
python -m pip install -r tools/elevation_viewer/requirements.txt
python tools/elevation_viewer/server.py
```

브라우저에서 http://127.0.0.1:8765 를 열고 PCD를 선택합니다.
상세 사용법과 알고리즘 한계는 [Elevation Lab 사용 설명서](tools/elevation_viewer/README.md)를 참고하세요.

원본 지도 파일은 포함하지 않습니다. 통과 가능 후보는 실제 로봇의 안전을 보장하지 않습니다.
