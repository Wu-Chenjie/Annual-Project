#!/usr/bin/env python3
"""Analyze existing runs and cut traceable clips from one audited native recording."""
import argparse
import bisect
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from execution_diagnostics import diagnose, records


def clock_anchor(executions,simulation,origin):
    rows=sorted((p['time'],p['receipt_wall_time']-origin) for p in executions if p['drone']==0)
    times=np.asarray([r[0] for r in rows]);walls=np.asarray([r[1] for r in rows])
    if simulation<times[0] or simulation>times[-1]:raise ValueError('Clip anchor is outside recorded execution clock')
    return float(np.interp(simulation,times,walls))


def lifecycle_pair(events):
    lows={}
    for event in sorted(events,key=lambda e:e['time']):
        key=(event['drone'],event.get('region'))
        if event['type']=='region_low_yield':lows[key]=event
        if event['type']=='task_reactivated' and key in lows and event.get('reason') in ('actual_evidence_changed','different_view_with_verified_ray_gain','gain_improved'):
            return lows[key],event
    return None


def late_support(events,cutoff):
    owners={};changes=[]
    for event in sorted(events,key=lambda e:e['time']):
        if event['type']=='pair_cvrp_committed':
            for region,owner in event.get('assignments',{}).items():
                old=owners.get(region)
                if old is not None and old!=owner and event['time']>=cutoff:
                    changes.append((int(region),owner,event))
                owners[region]=owner
        elif event['type']=='path_committed':
            for region,owner,change in reversed(changes):
                if region==event.get('region') and owner==event['drone'] and event['time']-change['time']<=25.:
                    return change,event
    return None


def package(project,output,make_clips=True):
    project=Path(project);output=Path(output);output.mkdir(parents=True,exist_ok=True)
    study=project/'artifacts/todo-execution/formal-10'
    selected=json.loads((study/'study-report-10e-resumed.json').read_text())
    summaries=[]
    paths={e['job']:study/e['directory'] for e in selected['infrastructure_attempt_history'] if e['selected']}
    for row in selected['attempts']:
        if row['outcome']=='COMPLETE':
            result=diagnose(paths[row['run']],output/'diagnostics'/row['run'])
            summaries.append(dict(run=row['run'],low_speed_fraction=result['actual_low_speed_fraction'],
                                  low_by_phase=result['low_speed_by_phase_fleet_s'],residence=result['residence_by_phase_fleet_s'],
                                  handoff=result['handoff'],overlap_status=result['request_overlap_status'],
                                  planning_during_motion_s=result['planning_during_actual_motion_fleet_s']))
    recording=project/'artifacts/todo-execution/candidate-10c-physical-demo-900'
    physical=diagnose(recording,output/'diagnostics/physical-demo-900')
    events=sorted([e for p in recording.glob('drone_*/events.jsonl') for e in records(p)],key=lambda e:e['time'])
    executions=list(records(recording/'execution-evidence.jsonl'))
    windows=list(csv.DictReader((recording/'service-windows.csv').open()))
    summary=json.loads((recording/'summary.json').read_text())
    coverage=json.loads((recording/'coverage.json').read_text())
    t90=next(t for t,c in coverage if c>=.9);t95=next(t for t,c in coverage if c>=.95)
    source=recording/'gazebo-rviz-raw.mp4'
    origin=json.loads((recording/'recording-clock.json').read_text())['recorded_monotonic']
    probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(source)]))
    duration=float(probe['format']['duration'])
    clips=[];missing=[]
    def add(name,begin,end,evidence):
        # Five seconds before / after anchors preserve context; no cuts inside a clip.
        a=max(0.,clock_anchor(executions,begin,origin)-5.)
        b=min(duration,clock_anchor(executions,end,origin)+5.)
        if b<=a:raise ValueError('Invalid native recording clip interval')
        destination=output/(name+'.mp4')
        if make_clips and not destination.exists():
            subprocess.run(['ffmpeg','-nostdin','-v','error','-ss',str(a),'-i',str(source),'-t',str(b-a),
                            '-an','-c:v','libx264','-preset','veryfast','-crf','21',str(destination)],check=True)
        clips.append(dict(name=name,simulation_start=begin,simulation_end=end,video_start=a,video_end=b,
                          playback_multiplier=1,continuous=True,file=destination.name,evidence=evidence))
    handoff=next(e for e in events if e['type']=='handoff_consumed' and e.get('speed',0)>.1)
    add('moving-handoff',handoff['time']-2.,handoff['time']+2.,handoff)
    dynamic=json.loads((recording/'dynamic-backup-audit.json').read_text())
    switch=next(e for e in events if e['type']=='cached_route_switched')
    add('dynamic-backup',switch['time']-7.,switch['time']+8.,switch)
    transit=next((w for w in windows if w['purpose']=='transit_reobserve' and float(w['distance_m'])>.5),None)
    if transit:add('necessary-transit',float(transit['start']),float(transit['end']),transit)
    else:missing.append('necessary_transit')
    pair=lifecycle_pair(events)
    if pair:
        add('low-yield-exit',pair[0]['time']-2.,pair[0]['time']+2.,pair[0])
        add('evidence-reactivation',pair[1]['time']-2.,pair[1]['time']+2.,pair[1])
    else:missing.append('verified_exit_and_reactivation_pair')
    support=late_support(events,t90)
    if support:add('tail-coordination',support[0]['time'],support[1]['time']+2.,dict(assignment=support[0],execution=support[1]))
    else:missing.append('late_assignment_followed_by_actual_execution')
    add('tail-90-to-95',t90,t95,dict(coverage_start=.9,coverage_end=.95))
    def sha(file):
        h=hashlib.sha256()
        with file.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
        return h.hexdigest()
    for clip in clips:
        file=output/clip['file']
        if file.exists():clip['sha256']=sha(file)
    index=dict(schema='annual.todo-evidence-package/1',frozen_video_flight_policy='candidate-10a',new_code_flight_tested=False,
               source_video_sha256=sha(source),source_video=str(source),source_duration_s=duration,clips=clips,missing_clips=missing,
               native_dynamic_audit_passed=dynamic.get('passed'),run_diagnostics=summaries,
               physical_low_speed_fraction=physical['actual_low_speed_fraction'],
               clock_alignment='Simulation time interpolated against recorded execution receipt_wall_time minus recording-clock.recorded_monotonic; FFmpeg startup/frame offset requires visual HUD QA.',
               limits=['All clips use the same audited native physical-demo run, normal speed and no internal cuts.',
                       'Late coordination shows an agreed region assignment followed by that aircraft committing the task; it is not proof of arrival-prediction causal benefit.',
                       'Reactivation reason is online telemetry; actual service yield is separately stored in the original service-windows.csv.',
                       'These existing recordings do not validate the newly changed handoff/dwell code.'])
    (output/'evidence-index.json').write_text(json.dumps(index,indent=2)+'\n')
    rows=['# 现有日志与同次原生录像补证据','',
          '仅复算和剪辑现有候选10a数据；不把旧运行作为新交接提前量/观测完成代码的飞行验收。','',
          '| 运行 | 实测低速时间占比（<0.1m/s） | 交接提案→实际消费 | 移动期间规划重叠（机秒） |',
          '|---|---:|---:|---:|']
    for row in summaries:
        h=row['handoff'];concurrent=row['planning_during_motion_s']
        rows.append(f"| {row['run']} | {100*row['low_speed_fraction']:.1f}% | {h['proposed']}→{h['consumed']} | {concurrent:.3f} |" if concurrent is not None else
                    f"| {row['run']} | {100*row['low_speed_fraction']:.1f}% | {h['proposed']}→{h['consumed']} | 缺请求边界，未推算 |")
    rows+=['','低速基于采样真实位姿差分；状态原因和低速区间按仿真时间对齐。`execution-timeline.csv`保存逐机时间轴，`diagnostics.json`保存驻留分解、交接取消原因、请求/飞行重叠及输入哈希。转向/加减速期间的低速不能全部算作等待规划。','',
           '| 原速连续片段 | 仿真时间范围（秒） | 视频时间范围（秒） |','|---|---:|---:|']
    for clip in clips:
        rows.append(f"| [{clip['name']}]({clip['file']}) | {clip['simulation_start']:.3f}–{clip['simulation_end']:.3f} | {clip['video_start']:.3f}–{clip['video_end']:.3f} |")
    rows+=['','同次源视频、精确事件、服务窗口及哈希见 `evidence-index.json`。剪辑为1倍速、内部无跳切；仿真事件通过单调墙钟接收记录定位，截取前后各保留5秒，画面HUD用于复核。','',
           '在线重激活原因与离线实测收益分开记录；动态备选本次执行只新增1个自由体素，不把后继服务收益归给备选。']
    if missing:rows+=['','缺失且未伪造的片段：'+', '.join(missing)]
    (output/'README.md').write_text('\n'.join(rows)+'\n')
    return index


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--project',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--no-clips',action='store_true');a=p.parse_args();r=package(a.project,a.output,not a.no_clips)
    print(json.dumps(dict(runs=len(r['run_diagnostics']),clips=[c['name'] for c in r['clips']],missing=r['missing_clips'])))
