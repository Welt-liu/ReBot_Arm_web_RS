# web_mujoco（M1）

Mac 上的纯网页运动学演示：用官方 `@mujoco/mujoco` 在浏览器里加载
`rs_grasp_scene.xml`，滑块直接写 `qpos` 再调用 `mj_forward`。

这一层不启动 ROS、不跑 `mj_step`、也不连真机。

## 环境

- macOS，Node.js 18+
- Chrome 或 Edge

## 启动

```bash
cd web_mujoco
npm install
npm run dev
```

打开 [http://127.0.0.1:5173](http://127.0.0.1:5173)。首次会下载约 62 MB STL，之后走浏览器缓存。

## GitHub Pages

推送到 `main` 或 `feat/web-mujoco-m1` 后，Actions 会构建并发布静态站。仓库需在
**Settings → Pages → Source** 选择 **GitHub Actions**（只需设一次）。

地址为：

`https://<owner>.github.io/ReBot_Arm_web_RS/`

首次打开会下载全部网格，请用 Chrome 或 Edge。

本地预览生产包：

```bash
cd web_mujoco
GITHUB_PAGES_BASE=/ReBot_Arm_web_RS/ npm run build
npx vite preview
```

## M1 验收

- 手臂、桌面、红/蓝/黄物体可见
- J1–J6 连续转动，模型不瞬移（运动学直连）
- 夹爪滑块同时驱动 `joint7` / `joint_left` / `joint_right`
- 复位回到零位
