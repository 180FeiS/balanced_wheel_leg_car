# 06 双核轮腿车人机交互与无线遥控调试软件

## 功能概述
- 哈希表菜单树（Menu）与 IPS200 界面（UI）
- 按键/串口/遥控命令路由
- LORA 接收（CM7_1）与应用（CM7_0）
- dualcore UI 命令 FIFO 与 ctrl 快照
- VOFA 无线调试上发

## 架构
```
MenuInit -> selectMenu_Key/selectMenu
remote_lora_init -> dualcore_remote_publish
dualcore_ui_cmd_push -> dualcore_ui_cmd_consume_all (CM7_0)
ui_pull_ctrl_snapshot / dualcore_ctrl_to_ui_publish
```
