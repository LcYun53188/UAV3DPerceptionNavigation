"""Explain the observed startup gate without granting control or flight admission."""
import json
from pathlib import Path
import time


def read_record(folder, name):
    try:
        value = json.loads((Path(folder)/(name+'.json')).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def startup_status(folder, *, active=True, now_ns=None):
    now_ns = time.time_ns() if now_ns is None else now_ns

    def fresh(record, limit=2.):
        stamp = record.get('system_ns')
        return isinstance(stamp, (int, float)) and 0 <= (now_ns-stamp)/1e9 <= limit

    def result(phase, message, **details):
        return dict(phase=phase, message=message, **details)

    if not active:
        return result('STOPPED', '仿真已停止；目录内 READY 和准入记录仅是历史结果')
    live = read_record(folder, 'pocket-live')
    session = read_record(folder, 'live-session')
    progress = read_record(folder, 'map-startup-stability')
    map_admission = read_record(folder, 'map-admission')
    if session.get('state') == 'POCKET_PREFLIGHT_BLOCKED':
        return result('PREFLIGHT_BLOCKED', '启动预检已阻断：'+str(session.get('reason', '未知原因')),
                      hint='保留故障记录；排除原因后重启会话，后续 READY 不会自动解除此阻断')
    if fresh(live):
        if live.get('source_preflight_admitted'):
            if not live.get('source_ready'):
                return result('SOURCE_UNAVAILABLE', '定位当前不可用：'+str(live.get('source_reason')))
            if not live.get('operator_input_ready'):
                native = live.get('input_evidence', {}).get('native', {})
                return result('WAITING_OPERATOR_INPUT', '定位已准入，等待 QGC 遥控输入',
                              native_samples=native.get('samples', 0),
                              hint='检查 QGC 所选摇杆、校准、启用状态与 Fly 页面；继续等待 VIO 不会解决输入缺失')
            return result('READY_FOR_OPERATOR', '定位与遥控输入已就绪，等待操作者指令',
                          auto_prepared=live.get('auto_prepared', False),
                          hint='自动任务仍须通过准备档准入；此状态不代表飞行验收')
    elif session or read_record(folder, 'flight-admission'):
        return result('TELEMETRY_STALE', '遥控网关状态已过期；历史准入不能证明当前就绪')
    if fresh(map_admission, 15.):
        return result('MAP_SETTLING', '深度建图已开放，正在复核定位稳定性')
    if fresh(progress, 10.):
        sources = progress.get('sources', {})
        sdk = progress.get('sdk_startup') or {}
        return result('STABILITY_WINDOW', '等待原始 VIO、PX4 与 SDK 连续稳定窗口',
                      source_reason=progress.get('source_reason'),
                      window_s=progress.get('window_s'), elapsed_s=progress.get('elapsed_s'),
                      sources=sources, sdk_checks=sdk.get('checks', {}))
    return result('STARTING' if not progress else 'TELEMETRY_STALE',
                  '等待启动遥测' if not progress else '启动进度已过期，检查运行器与日志')


def status_text(status):
    lines = [status['message']]
    if status['phase'] == 'STABILITY_WINDOW':
        lines.append(f"定位源：{status.get('source_reason')}；窗口：{status.get('window_s')} s")
        for name, source in status.get('sources', {}).items():
            span = source.get('position_span_m')
            position = '无数据' if span is None else f'{span*100:.2f} cm / 3 cm'
            lines.append(f"{name}: {source.get('span_s', 0):.1f} s，位置跨度 {position}")
            if source.get('drift_stable') is False and source.get('drift_rate_m_s') is not None:
                lines.append(f"{name} 仍在缓慢漂移：{source['drift_rate_m_s']*1000:.2f} mm/s")
        pending = [name for name, passed in status.get('sdk_checks', {}).items() if not passed]
        if pending:
            lines.append('SDK 待满足：'+', '.join(pending))
    if status.get('hint'):
        lines.append(status['hint'])
    return '\n'.join(lines)
