# 标定文件目录

本目录中的文件由 `calibrate_tcp` 和 `calibrate_camera` 生成。`hardware.yaml`
默认引用四份相机外参和两份 TCP 标定；缺少任意一份时，遥操作节点会拒绝启动。

推荐顺序：左右 TCP pivot 标定 → front_cam 对左右 base 各标定一次 → side_cam 对左右
base 各标定一次。相机标定板必须刚性安装在已标定 TCP 上，并准确填写 TCP 到 ArUco
标记的刚性变换。
