from dataclasses import dataclass, field
from typing import List

@dataclass
class DetectionConfig:
    # 模型选择
    model_version: str = "11"
    model_scale: str = "n"
    model_weight: str = "pt"

    # 模型参数
    conf_threshold: float = 0.4
    iou_threshold: float = 0.45
    imgsz: int = 640
    device: str = "cpu"

    classes: List[int] = field(default_factory=list)

    # 可视化参数
    line_width: int = 2
    font_scale: float = 0.5
    font_thickness: int = 1

    show_confidence: bool = True
    
