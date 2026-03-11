import os
import cv2

NUM_MARKERS = 20                     # 생성할 마커 개수
MARKER_PIXEL_SIZE = 400              # 마커 픽셀 사이즈
ARUCO_DICT = cv2.aruco.DICT_4X4_250  # 마커 딕셔너리

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, '../models/templates/aruco_marker_template')
OUTPUT_BASE_DIR = os.path.join(BASE_DIR, '../models/generated_markers')

def create_marker_model(marker_id):
    """단일 ArUco 마커 모델 폴더, 이미지, 설정 파일 생성"""
    
    # 1. 출력 디렉토리 구조 생성
    model_name = f"aruco_marker_{marker_id}"
    model_dir = os.path.join(OUTPUT_BASE_DIR, model_name)
    texture_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(texture_dir, exist_ok=True)

    # 2. ArUco 마커 이미지 생성 및 저장
    try:
        # OpenCV 최신 버전
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    except AttributeError:
        # 구버전
        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
        
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_PIXEL_SIZE)
    image_path = os.path.join(texture_dir, f"{model_name}.png")
    cv2.imwrite(image_path, marker_image)

    # 3. 템플릿 파일 읽기 및 {{MARKER_ID}} 치환 후 저장
    for filename in ["model.config", "model.sdf"]:
        template_path = os.path.join(TEMPLATE_DIR, filename)
        output_path = os.path.join(model_dir, filename)

        # 템플릿 파일이 존재하는지 확인
        if not os.path.exists(template_path):
            print(f"[ERROR] 템플릿 파일을 찾을 수 없습니다 -> {template_path}")
            return

        with open(template_path, 'r', encoding='utf-8') as file:
            content = file.read()

        # 플레이스홀더 치환
        content = content.replace("{{MARKER_ID}}", str(marker_id))

        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(content)

    print(f"[{marker_id}/{NUM_MARKERS-1}] {model_name} 생성 완료")

def main():
    print(f"총 {NUM_MARKERS}개의 ArUco 마커 생성을 시작합니다.")
    os.makedirs(OUTPUT_BASE_DIR, exist_ok=True)
    
    for i in range(NUM_MARKERS):
        create_marker_model(i)
        
    print(f"\n모든 작업이 완료되었습니다. 생성 위치: {os.path.abspath(OUTPUT_BASE_DIR)}")

if __name__ == "__main__":
    main()