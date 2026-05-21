from __future__ import annotations

from igvi_ui.clients.host_client import HostClient
from igvi_ui.tasks.base_page import BaseTaskPage


class Task3DoorPage(BaseTaskPage):
    ACCENT = "#f59e0b"
    TASK_NUM = "3"
    TASK_ID = "door"

    def __init__(self, client: HostClient) -> None:
        super().__init__(
            title="導航開門",
            description="自主導航至目標門，操作門把開門後通過並繼續前進",
            client=client,
        )

    def _step_defs(self) -> list[tuple[str, str]]:
        return [
            ("確認定位就緒", "確保定位系統就緒"),
            ("導航至門前",   "移動至門把前方適當距離"),
            ("對準門把",     "微調位置使手臂能觸及門把"),
            ("手臂開門",     "抓握門把→下壓→推開門"),
            ("通過門口",     "導航穿過已開啟的門"),
            ("返回起點",     "導航回到出發位置"),
        ]
