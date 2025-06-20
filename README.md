# Suspension

### 프로그램 실행 
1. `python -m suspension.utils.bump_generator` 를 실행하여 speedbump STL을 생성합니다.
2. `python -m suspension.utils.update_mjcf` 를 실행하여 `generated_scene.xml`을 생성합니다.
3. `python scripts/run_sim.py` 를 실행하여 차량 주행을 확인합니다.
   - 생성된 STL은 `models/speedbumps/` 폴더에 저장됩니다.
   - MJCF 파일은 `models/generated_scene.xml`로 저장됩니다.

### 작업 참고 사항
1. `scripts/main.py`에서 bump 생성, MJCF 갱신, 시뮬레이션 실행을 한 번에 수행합니다.
2. 현재 PID.py 작업 중입니다. 여러 버전이 존재하여 혼란을 줄 우려가 있어 업로드 하지 않았습니다.
3. run_sim.py는 PID 적용 및 출력값 수정 등의 이유로 구조가 크게 바뀔 수 있습니다.
4. Commit Message에 구체적인 역할 정리해두었으니, 확인해주세요! 
