from __future__ import annotations

from igvi_ui.clients.host_client import HostClient
from igvi_ui.tasks.base_page import BaseTaskPage


class Task1BridgePage(BaseTaskPage):
    ACCENT = "#3b82f6"
    TASK_NUM = "1"
    TASK_ID = "bridge"

    def __init__(self, client: HostClient) -> None:
        super().__init__(
            title="上下橋",
            description="自主導航至橋面，穩定通過後下橋返回起點",
            client=client,
        )

    def _step_defs(self) -> list[tuple[str, str]]:
        return [
            ("確認定位就緒",   "確保 /pose topic 持續更新"),
            ("導航至橋入口",   "移動至橋的起點座標"),
            ("通過橋面",       "低速穩定通過橋面"),
            ("下橋",           "下橋至對面平台"),
            ("返回起點",       "導航回到出發位置"),
        ]
