"""장기말 라벨. models/weights_14class.pt의 클래스 순서와 같다 (모델 메타데이터로 확인).

blue = 초(楚), red = 한(漢).
"""

from __future__ import annotations

CLASS_NAMES = [
    "blue cha", "blue horse", "blue king", "blue po", "blue sa", "blue sang", "blue solder",
    "red cha", "red horse", "red king", "red po", "red sa", "red sang", "red solder",
]

SIDES = ("blue", "red")
# 모델 라벨의 말 이름 → 내부 종류 이름
KIND_OF = {"cha": "cha", "horse": "horse", "king": "king", "po": "po", "sa": "sa", "sang": "sang",
           "solder": "soldier"}
KINDS = ("king", "cha", "po", "horse", "sang", "sa", "soldier")

# 화면 표시용 글자
KOREAN = {
    ("blue", "king"): "초", ("red", "king"): "한",
    ("blue", "soldier"): "졸", ("red", "soldier"): "병",
}
KOREAN_KIND = {"cha": "차", "po": "포", "horse": "마", "sang": "상", "sa": "사", "king": "궁", "soldier": "졸"}
SIDE_KOREAN = {"blue": "초", "red": "한"}


def parse_label(name: str) -> tuple[str, str]:
    """'blue horse' → ('blue', 'horse')"""
    side, kind = name.split(" ", 1)
    if side not in SIDES or kind not in KIND_OF:
        raise ValueError(f"알 수 없는 라벨: {name}")
    return side, KIND_OF[kind]


def piece_char(side: str, kind: str) -> str:
    if (side, kind) == ("unknown", "piece"):
        return "알"
    return KOREAN.get((side, kind), KOREAN_KIND[kind])
