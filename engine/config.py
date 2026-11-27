PIXELS_PER_METER = 50.0 # 1m는 50px

DT = 1.0 / 120.0 # 한 스텝의 시간

FRAME_BUDGET_MS = 1000.0 / 60.0 # 60FPS 한 프레임 예산

GRAVITY = 9.8 * PIXELS_PER_METER # 중력

REST_LINEAR_SPEED = 0.05 * PIXELS_PER_METER # 0.05 m/s 이하 속도는 정지! -> 픽셀로 환산
# Box2D 3.x b2BodyDef.sleepThreshold 기본값 0.05 m/s (include/box2d/types.h) → 50 px/m 환산

REST_ANGULAR_SPEED = 2.0 * 3.141592653589793 / 180.0 # 2도/s 이하 각속도는 정지! -> 라디안으로 환산
# Box2D 2.4 b2_angularSleepTolerance = 2도/s

SOLVER_ITERATIONS = 8 # 속도 솔버 반복 횟수 (practice code에선 1번이었음!)
# Box2D 2.4 권장 velocityIterations = 8

RESTITUTION_THRESHOLD = 1.0 * PIXELS_PER_METER # 이보다 느리게 닿으면 튀지 않음 (반발 0)
# Box2D 2.4 b2_velocityThreshold = 1 m/s → 50 px/s

POSITION_PERCENT = 0.2 # 겹침 중 한 스텝에 밀어내는 비율!

POSITION_SLOP = 0.25 # 일부러 남겨 두는 겹침 (px) - 겹침을 0까지 다 없애면 다음 스텝에 접촉이 떨어졌다가 다시 붙으면서 떨리는 현상 발생했음!
# Box2D b2_linearSlop = 0.005 m → 0.25 px

WARM_START_MATCH_DIST = 2.0 # 지난 스텝 접촉점과 이 거리(px) 안이면 같은 점으로 본다

SLEEP_LINEAR_SPEED = REST_LINEAR_SPEED # 잠드는 선속도 기준 = 측정의 '정지' 기준 (2.5 px/s)

SLEEP_ANGULAR_SPEED = REST_ANGULAR_SPEED # 잠드는 각속도 기준 (2도/s)

TIME_TO_SLEEP = 0.5 # 이 시간 동안 계속 느려야 멈추기!
# Box2D 2.4 b2_timeToSleep = 0.5 s

WAKE_SPEED = 4.0 * SLEEP_LINEAR_SPEED # 잠든 물체에 닿은 상대가 이보다 빠를 때만 깨우기(10px/s)

