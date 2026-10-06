from pathlib import Path
import json,csv,hashlib,gzip,os
os.environ.setdefault('MPLCONFIGDIR','/tmp/comparison-matplotlib')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
b=Path(__file__).resolve().parent
modes=['annual','racer','gvp'];names=['Annual-Project','RACER','GVP-MREP'];colors=['#2563eb','#d97706','#059669']
results=[json.loads((b/f'trial-{m}-900/result.json').read_text()) for m in modes]
protocol=json.loads((b/'protocol.json').read_text());world=json.loads((b/'map.json').read_text())
checks={};summaries=[]
for m,d in zip(modes,results):
 out=b/f'trial-{m}-900';rows=list(csv.DictReader((out/'trajectory.csv').open()));refs=[json.loads(x) for x in (out/'executed-commands.jsonl').open()]
 vmax=max(np.linalg.norm(r['v']) for r in refs);amax=max(np.linalg.norm(r['a']) for r in refs)
 counts={};first={};last={}
 with gzip.open(out/'sensor-stream.jsonl.gz','rt') as f:
  for line in f:
   r=json.loads(line);k=r['type']+('_'+str(r['i']) if 'i' in r else '');counts[k]=counts.get(k,0)+1;first.setdefault(k,r['t']);last[k]=r['t']
 rates={k:((v-1)/(last[k]-first[k]) if last[k]>first[k] else 0) for k,v in counts.items()}
 pos=np.array([[float(r[k]) for k in ['x','y','z']] for r in rows if float(r['t'])>=20]);outside=bool(np.any((pos<np.array(world['bounds'][0])-.01)|(pos>np.array(world['bounds'][1])+.01)))
 checks[m]={'reference_speed_cap_pass':bool(vmax<=.600001),'reference_acceleration_cap_pass':bool(amax<=.800001),'world_sha256':hashlib.sha256((out/'scene/scene.sdf').read_bytes()).hexdigest(),'sensor_stream_gzip_verified':True,'recorded_counts':counts,'recorded_sim_hz':rates,'native_commands_from_both_uavs':d['native_seen']==[0,1],'outside_center_bounds_after_startup':outside,'result_status':d['status']}
 summaries.append(dict(system=names[modes.index(m)],**d))
checks['identical_generated_world']=len({checks[m]['world_sha256'] for m in modes})==1
frozen=json.loads((b/'frozen-sha256.json').read_text());checks['frozen_files_unchanged']=all(hashlib.sha256((b/p).read_bytes()).hexdigest()==sha for p,sha in frozen.items())
(b/'validation.json').write_text(json.dumps(checks,indent=2));(b/'comparison.json').write_text(json.dumps(summaries,indent=2))
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'path','axes.spines.top':False,'axes.spines.right':False,'font.size':10})
fig,axs=plt.subplots(2,2,figsize=(12,12),layout='constrained');ax=axs.flat[0]
for m,name,c,d in zip(modes,names,colors,results):
 log=[json.loads(l) for l in (b/f'trial-{m}-900/coverage.jsonl').open()];ax.plot([r['t'] for r in log],[100*r['coverage'] for r in log],color=c,lw=2,label=name)
 if d['status']=='SAFETY_FAILURE':ax.scatter([d['t']],[100*d['coverage']],color=c,marker='x',s=80,zorder=5)
ax.axhline(95,color='#6b7280',ls='--',lw=1);ax.axvspan(0,20,color='#e5e7eb',alpha=.6);ax.set(xlabel='Simulation time (s), including 20 s initialization',ylabel='Externally measured free-space coverage (%)',ylim=(0,102),title='Coverage until success, safety stop, or time limit');ax.legend(loc='lower right');ax.grid(alpha=.15)
for ax,m,name,d in zip(list(axs.flat)[1:],modes,names,results):
 for ob in world['obstacles']:
  lo,hi=np.array(ob['min']),np.array(ob['max']);ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],facecolor='#cbd5e1',edgecolor='#94a3b8',lw=.5))
 rows=list(csv.DictReader((b/f'trial-{m}-900/trajectory.csv').open()))
 for i,c in enumerate(['#0e7490','#a21caf']):
  pts=np.array([[float(r['x']),float(r['y'])] for r in rows if int(r['i'])==i]);ax.plot(pts[:,0],pts[:,1],c=c,lw=1.3,label=f'UAV {i+1}');ax.scatter(*pts[0],marker='^',s=45,color=c);ax.scatter(*pts[-1],marker='o',s=30,color=c)
 for hit in d['geometric_collision_events'][:1]:
  r=min((r for r in rows if int(r['i'])==hit['i']),key=lambda r:abs(float(r['t'])-hit['t']));ax.scatter(float(r['x']),float(r['y']),marker='x',s=100,color='#dc2626',zorder=6)
 status={'COVERAGE_TARGET':'95% target reached','SAFETY_FAILURE':'Safety stop','TIME_LIMIT':'Time limit'}[d['status']]
 ax.set(title=f"{name}: {status}\n{d['t']:.1f} s, coverage {d['coverage']*100:.1f}%",xlabel='x (m), XY projection',ylabel='y (m)',xlim=(-6.3,6.3),ylim=(-6.3,6.3),aspect='equal');ax.legend(loc='lower right',fontsize=9)
fig.suptitle('Single paired trial on a common physical simulator\nSeed 900 | 2 UAVs | common LiDAR, estimator, reference limits and PID',fontsize=14)
fig.savefig(b/'comparison.svg');fig.savefig(b/'comparison-preview.png',dpi=150);plt.close(fig)
status_cn={'COVERAGE_TARGET':'覆盖率达标','SAFETY_FAILURE':'安全失败','TIME_LIMIT':'超时未达标'}
lines=['# 三系统同条件闭环实验（单次）','', '本报告比较接入公共物理仿真与执行层后的完整探索闭环：各仓库自己的建图、任务分配、路径规划和轨迹采样仍在运行。结果不是分配器替身测试，也不是三套原生仿真演示的直接拼表。','', '本轮三组均达标，GVP 43.00 s、RACER 50.16 s、Annual 107.45 s（均含统一初始化）。图中的轨迹为 XY 投影，穿过矮柜轮廓可能是从上方飞越；碰撞核查使用完整三维机体。两机航程仅是执行负载参考，不等价于任务工作量。','', '只运行一组 seed=900，不支持统计显著性或普遍优劣结论。所有正式试验使用冻结配置，预检没有计入成绩。','', '| 系统 | 结果 | 结束时间，s | 覆盖率 | T95，s | 两机航程，m |','|---|---|---:|---:|---:|---|']
for name,d in zip(names,results):
 dist=d['distance_m'];t95=f"{d['t95']:.2f}" if d['status']=='COVERAGE_TARGET' else '未达成有效成功'
 lines.append(f"| {name} | {status_cn[d['status']]} | {d['t']:.2f} | {100*d['coverage']:.2f}% | {t95} | {dist.get('0',0):.2f} / {dist.get('1',0):.2f} |")
lines+=['','时间从仿真开始计，包含统一的 20 秒起飞/扫描阶段；覆盖率约每秒采样一次。安全失败后的覆盖率不外推，不将停止早的系统与完成者直接按结束覆盖率排名。','','## 实验条件','','- 地图：12×12×3 m，中央带门隔墙、左侧隔墙和右侧矮柜；2 架相同物理无人机，起点与初始姿态相同。','- 传感器：真实 Gazebo GPU LiDAR，4.5 m 量程，水平/垂直视场各 120°，181×31 束，5 Hz，距离高斯噪声标准差 0.005 m。','- 状态估计：相同的噪声定位输入与 IMU 滤波器；不是完整视觉/LiDAR SLAM 的对照。','- 公共执行层：相同参考指令整形器和 PID 电机控制器；三维速度范数≤0.6 m/s、加速度范数≤0.8 m/s²、航向角速度≤0.65 rad/s。整形器不接触地图，不选择任务。','- 覆盖率：公共外部映射器的已观测自由体素 / 真值中几何完全自由体素；0.3 m 体素，分母 12,063；真值仅用于评估。目标 95%，最大 300 仿真秒。','- 安全停止：机体 OBB 与障碍相交（忽略≤5 mm 数值擦边），或参考跟踪误差持续 1 秒超过 0.8 m。接触消息计数单独保留，不把没有接触消息当成无碰撞证明。','','## 速度与跟踪实测','','相同参考约束不代表实测速度完全一样。以下同时给出整形前规划器指令、执行指令和真实物理速度的最大范数；超调没有隐藏。','','| 系统 | 原始规划指令峰值 m/s | 执行指令峰值 m/s | 实测峰值 m/s | 跟踪误差 P95 m | 指令接收年龄 P95 s |','|---|---:|---:|---:|---:|---:|']
for name,d in zip(names,results):lines.append(f"| {name} | {d['raw_planner_max_speed_mps']:.3f} | {d['max_command_speed_mps']:.3f} | {d['max_measured_speed_mps']:.3f} | {d['tracking_error_p95_m']:.3f} | {d['adapter_command_age_p95_s']:.3f} |")
lines+=['','因此，本轮满足相同地图、传感器模型与执行指令约束；**不能称作实际飞行速度完全相等的严格同速排名**。ROS 1/ROS 2 运行时及算力分配也不同，不比较 CPU 性能或墙钟耗时。','','## 适配边界与预检发现','','1. RACER 原生点云/轨迹接口通过传输桥接到公共仿真；探索管理、HGrid、ACVRP/LKH 和 B-spline 规划器保留。碰撞膨胀按公共 0.64×0.64×0.16 m 机体配置。','2. GVP 的独立源码副本将墙钟改成仿真时钟，使用原生点云映射入口与有限视场模型，保留 MR-DTG、图 Voronoi、目标选择和轨迹优化；恢复其原生消息类型转换桥。','3. 公共宽视场会看到旋翼。三组都过滤已知机体内的自回波，不把遮挡后的空间涂成自由；GVP 加入与公共映射器一致的“机体已占用空间可视为环境自由”处理。所有这些空间证据来自测得位姿与已知机体尺寸，不来自世界真值。','4. 预检证明 RACER 参数 max_vel=0.6 并非硬性三维限速，曾输出约 1.08 m/s。故正式组统一使用公共参考整形器，同时保留整形前日志。整形可能改变轨迹跟踪行为，因此失败只能归于本次适配闭环，不能单独归因于任务分配算法。','5. 预检还发现消息类型不匹配、缓存损坏、未清理进程、点云传输阻塞等问题；smoke-* 目录仅作为诊断记录，不得与正式成绩混用。','','## 安全事件','']
for name,d in zip(names,results):lines.append(f"- {name}：{d['failure_reason'] or '本次未触发安全停止'}；机体几何碰撞记录 {len(d['geometric_collision_events'])} 条；两机最小间距 {d['min_separation_m']:.3f} m。")
lines+=['','## 可复核文件','','- [冻结协议](protocol.json)、[配置 SHA-256](frozen-sha256.json)、[运行环境与二进制](environment.json)、[自动核查](validation.json)。','- [方形 SVG 对照图](comparison.svg)、[机器可读结果](comparison.json)。','- 每个 trial-*-900 目录包含 result.json、coverage.jsonl、真实 trajectory.csv、原始规划 commands.jsonl、执行 executed-commands.jsonl、点云/估计里程计/时钟输入 sensor-stream.jsonl.gz、仿真/规划器日志。','- [GVP 独立适配差异](gvp-adaptation.patch)、[公共参考约束](motion_envelope.py)、[几何碰撞核查](geometry_check.py)。','', '复现入口是 run.py；依赖当前已构建的两个专用容器及挂载工作区。用新的输出目录名运行，例如 `python3 run.py annual repeat-annual-900 300`；注意种子当前固定为 900，目录名不会改变种子。prepare.py 是早期脚手架，不是冻结试验的再生成入口。','']
(b/'REPORT.md').write_text('\n'.join(lines))
print(json.dumps({'results':[(n,r['status'],r['t'],r['coverage']) for n,r in zip(names,results)],'checks':checks},indent=2))
