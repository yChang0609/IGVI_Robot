from __future__ import annotations

from igvi_ui.clients.host_client import HostClient
from igvi_ui.tasks.base_page import BaseTaskPage


class Task4FindGrabPage(BaseTaskPage):
    ACCENT = "#a855f7"
    TASK_NUM = "4"
    TASK_ID = "find_grab"

    def __init__(self, client: HostClient) -> None:
        super().__init__(
            title="導航找熊抓熊",
            description="啟動視覺偵測，搜索場地中目標熊後導航並抓取",
            client=client,
        )

    def _step_defs(self) -> list[tuple[str, str]]:
        return [
            ("確認定位就緒",   "確保定位系統運作正常"),
            ("啟動視覺偵測",   "確認相機串流正常，偵測節點就緒"),
            ("搜索目標熊位置", "巡邏預設路徑，視覺偵測目標熊"),
            ("導航至熊旁",     "移動至偵測到的熊坐標附近"),
            ("手臂抓取目標熊", "展開手臂→夾取→舉起收回"),
            ("返回起點",       "攜帶目標物返回出發位置"),
        ]
