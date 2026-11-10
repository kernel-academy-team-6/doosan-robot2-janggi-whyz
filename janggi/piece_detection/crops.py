"""학습 데이터 제작과 실시간 분류에서 공유하는 원본 프레임 Crop."""
import math
from PIL import Image, ImageEnhance

def crop_geometry(image, box, angle=0.):
    """고정된 타이트 정사각형을 회전한 뒤 원본 프레임에서 샘플링할 좌표."""
    values = [float(v) for v in box]
    if len(values) != 4 or not all(math.isfinite(v) for v in values) or not math.isfinite(angle):
        raise ValueError('유효한 박스 좌표와 회전 각도가 필요합니다.')
    x1, y1, x2, y2 = values
    if x2 <= x1 or y2 <= y1:
        raise ValueError('검출 박스의 크기가 잘못되었습니다.')
    cx, cy = (x1+x2)/2, (y1+y2)/2
    side = max(x2-x1, y2-y1)
    radians = math.radians(angle)
    cosine, sine = math.cos(radians), math.sin(radians)
    # 출력 이미지를 반시계 방향으로 회전: 출력 좌표 -> 원본 좌표의 역변환.
    corners = [[cx+cosine*x-sine*y, cy+sine*x+cosine*y]
               for x, y in ((-side/2, -side/2), (side/2, -side/2),
                            (side/2, side/2), (-side/2, side/2))]
    if any(x < -1e-7 or y < -1e-7 or x > image.width+1e-7 or y > image.height+1e-7
           for x, y in corners):
        raise ValueError('회전할 영역이 원본 프레임 밖입니다. 해당 이미지를 보류/제외하거나 다른 각도를 사용하세요.')
    return cx, cy, side, cosine, sine, corners


def prepare_image(image, size=224, angle=0., brightness=1., contrast=1., *, box=None):
    """Crop을 회전하지 않고 원본에서 회전된 정사각형을 직접 추출한다. 인위적 여백 없음."""
    box = box if box is not None else (0, 0, image.width, image.height)
    cx, cy, side, cosine, sine, _ = crop_geometry(image, box, angle)
    scale = side/size
    matrix = (cosine*scale, -sine*scale, cx-cosine*side/2+sine*side/2,
              sine*scale, cosine*scale, cy-sine*side/2-cosine*side/2)
    result = image.convert('RGB').transform((size, size), Image.Transform.AFFINE, matrix,
                                          resample=Image.Resampling.BICUBIC)
    if brightness != 1:
        result = ImageEnhance.Brightness(result).enhance(brightness)
    if contrast != 1:
        result = ImageEnhance.Contrast(result).enhance(contrast)
    return result


