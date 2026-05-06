import os

NUM_MARKERS = 20       # 배치할 마커의 총 개수
SPACING_X = 0.595      # 마커 간 x축 간격 [m]       # 서버랙 가로 길이 595mm
START_X = -2.6775      # 첫 번째 마커쌍의 x 시작 좌표
START_Y = 0.0          # 첫 번째 마커의 y 시작 좌표
Z_POS = 0.13           # 마커의 높이
Y_POS_1 = 0.6          # 마주보는 벽면 좌표
Y_POS_2 = -0.6

# 회전 설정
ROLL = 0.0
PITCH = 1.570796
YAW_1 = 0.0
YAW_2 = 3.141592

# 출력 파일 경로 설정
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.join(SCRIPTS_DIR, '..', 'data')
OUTPUT_FILE = os.path.join(BASE_DIR, 'world_includes.txt')

def generate_include_tags():
    pairs_count = NUM_MARKERS // 2
    print(f"총 {NUM_MARKERS}개의 마커를 {pairs_count}쌍으로 대칭 배치합니다.")
    
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as file:
        file.write("\n")
        
        for i in range(pairs_count):
            # X 좌표는 동일하게 증가
            pos_x = START_X + (i * SPACING_X)
            
            # 1. 한쪽 면 마커 (ID: 0 ~ pairs_count - 1)
            id_1 = i
            tag_1 = f"""    <include>
      <name>aruco_marker_{id_1}</name>
      <uri>model://generated_markers/aruco_marker_{id_1}</uri>
      <pose>{pos_x:.2f} {Y_POS_1:.2f} {Z_POS:.2f} {ROLL:.2f} {PITCH:.6f} {YAW_1:.6f}</pose>
    </include>\n"""
            
            # 2. 맞은편 대칭 마커 (ID: pairs_count ~ TOTAL_MARKERS - 1)
            # (예: 총 50개일 때, 0번 마커의 맞은편은 25번 마커)
            id_2 = i + pairs_count
            tag_2 = f"""    <include>
      <name>aruco_marker_{id_2}</name>
      <uri>model://generated_markers/aruco_marker_{id_2}</uri>
      <pose>{pos_x:.2f} {Y_POS_2:.2f} {Z_POS:.2f} {ROLL:.2f} {PITCH:.6f} {YAW_2:.6f}</pose>
    </include>\n"""
            
            file.write(tag_1)
            file.write(tag_2)
            
    print(f"완료되었습니다. 아래 파일에서 결과를 확인하세요:\n{OUTPUT_FILE}")

if __name__ == "__main__":
    generate_include_tags()