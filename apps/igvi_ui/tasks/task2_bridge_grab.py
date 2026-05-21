from __future__ import annotations

from igvi_ui.clients.host_client import HostClient
from igvi_ui.tasks.base_page import BaseTaskPage


class Task2BridgeGrabPage(BaseTaskPage):
    ACCENT = "#22c55e"
    TASK_NUM = "2"
    TASK_ID = "bridge_grab"

    def __init__(self, client: HostClient) -> None:
        super().__init__(
            title="上橋抓熊下橋",
            description="導航上橋，手臂抓取橋上的目標熊，再安全下橋",
            client=client,
        )

    def _step_defs(self) -> list[tuple[str, str]]:
        return [
            ("確認定位就緒",       "確保定位系統運作正常"),
            ("導航至橋入口",       "移動至橋的起點"),
            ("上橋移動到熊旁",     "緩速通過橋面到達熊的位置"),
            ("手臂抓取目標熊",     "展開手臂→夾取→收回"),
            ("下橋",               "攜帶目標熊安全下橋"),
            ("返回起點",           "返回出發位置完成任務"),
        ]
